`default_nettype none

//-----------------------------------------------------------------------------
// N1-HEAL Top Level v1.0
// 16-Core Adaptive Precision AI Accelerator
// Interface: Wishbone B4 Slave
// Features: Thermal-Aware Adaptive Precision (TAAP)
// Author: Ahmed Mhmoude
// Date: 2026-04-26
//-----------------------------------------------------------------------------

module n1_heal_top #(
    parameter NUM_CORES         = 16,
    parameter DATA_WIDTH        = 512,
    parameter MAX_PRECISION     = 32,
    parameter SRAM_WEIGHT_DEPTH = 1024,
    parameter SRAM_DATA_DEPTH   = 2048,
    parameter SRAM_BIAS_DEPTH   = 32
)(
    `ifdef USE_POWER_PINS
        inout  wire vccd1,
        inout  wire vssd1,
    `endif
    input  wire        clk,
    input  wire        rst_n,
    input  wire        wb_cyc_i,
    input  wire        wb_stb_i,
    input  wire        wb_we_i,
    input  wire [3:0]  wb_sel_i,
    input  wire [31:0] wb_adr_i,
    input  wire [31:0] wb_dat_i,
    output reg         wb_ack_o,
    output reg  [31:0] wb_dat_o,
    output reg         wb_err_o,
    output reg         irq_done_o,
    output reg         irq_thermal_o,
    input  wire [7:0]  liquid_temp_sensor,
    output reg  [7:0]  pump_ctrl,
    output reg         thermal_crit_flag_o,
    input  wire        sleep_mode_i,
    output wire        busy_o
);

    wire [NUM_CORES-1:0] core_done_flags;
    wire [NUM_CORES-1:0] core_thermal_flags;
    reg  [NUM_CORES-1:0] core_enable;
    reg  [NUM_CORES-1:0] core_clock_gate;
    reg                  start_signal;
    reg  [63:0]          cycle_counter;
    reg  [31:0]          active_cycles;
    reg  [15:0]          thermal_events;
    reg  [7:0]           max_temp_recorded;
    reg  [2:0]           state;
    wire                 internal_reset;
    reg  [4:0]           global_precision;

    reg [MAX_PRECISION-1:0] weight_mem [0:SRAM_WEIGHT_DEPTH-1];
    reg [MAX_PRECISION-1:0] data_mem   [0:SRAM_DATA_DEPTH-1];
    reg [MAX_PRECISION-1:0] bias_mem   [0:SRAM_BIAS_DEPTH-1];

    assign internal_reset = ~rst_n;
    assign busy_o = start_signal | (|core_done_flags);

    wire [NUM_CORES-1:0] core_clk;
    generate
        genvar cg;
        for (cg = 0; cg < NUM_CORES; cg = cg + 1) begin : clk_gate_gen
            assign core_clk[cg] = clk & core_clock_gate[cg];
        end
    endgenerate

    localparam TEMP_NORMAL   = 8'd70;
    localparam TEMP_WARNING  = 8'd80;
    localparam TEMP_CRITICAL = 8'd90;
    localparam TEMP_HALT     = 8'd100;

    always @(posedge clk) begin
        if (!rst_n) begin
            start_signal       <= 1'b0;
            pump_ctrl          <= 8'h44;
            core_enable        <= {NUM_CORES{1'b1}};
            core_clock_gate    <= {NUM_CORES{1'b0}};
            thermal_crit_flag_o <= 1'b0;
            irq_thermal_o      <= 1'b0;
            thermal_events     <= 16'd0;
            max_temp_recorded  <= 8'd0;
            state              <= 3'd0;
            global_precision   <= 5'b10000;
        end else begin
            irq_thermal_o <= 1'b0;
            if (liquid_temp_sensor > max_temp_recorded)
                max_temp_recorded <= liquid_temp_sensor;

            case (state)
                3'd0: begin
                    pump_ctrl       <= 8'h66;
                    core_enable     <= {NUM_CORES{1'b1}};
                    core_clock_gate <= core_enable & {NUM_CORES{~sleep_mode_i}};
                    global_precision <= 5'b10000;
                    if (liquid_temp_sensor > TEMP_WARNING)
                        state <= 3'd1;
                end
                3'd1: begin
                    pump_ctrl <= 8'hFF;
                    core_clock_gate <= core_enable & {NUM_CORES{~sleep_mode_i}};
                    global_precision <= 5'b01000;
                    if (liquid_temp_sensor > TEMP_CRITICAL) begin
                        state <= 3'd2;
                        thermal_events <= thermal_events + 1'b1;
                    end else if (liquid_temp_sensor < TEMP_NORMAL)
                        state <= 3'd0;
                end
                3'd2: begin
                    pump_ctrl       <= 8'hFF;
                    core_enable     <= 16'h00FF;
                    core_clock_gate <= core_enable;
                    global_precision <= 5'b00100;
                    thermal_crit_flag_o <= 1'b1;
                    irq_thermal_o   <= 1'b1;
                    if (liquid_temp_sensor > TEMP_HALT)
                        state <= 3'd3;
                    else if (liquid_temp_sensor < TEMP_WARNING) begin
                        state <= 3'd1;
                        thermal_crit_flag_o <= 1'b0;
                    end
                end
                3'd3: begin
                    start_signal    <= 1'b0;
                    pump_ctrl       <= 8'hFF;
                    core_enable     <= {NUM_CORES{1'b0}};
                    core_clock_gate <= {NUM_CORES{1'b0}};
                    global_precision <= 5'b00010;
                    thermal_crit_flag_o <= 1'b1;
                    irq_thermal_o   <= 1'b1;
                    thermal_events  <= thermal_events + 1'b1;
                end
                default: state <= 3'd0;
            endcase
        end
    end

    wire [DATA_WIDTH-1:0] hbm3_bus_in;
    generate
        genvar d;
        for (d = 0; d < 16; d = d + 1) begin : data_concat
            assign hbm3_bus_in[d*32 +: 32] = data_mem[d];
        end
    endgenerate

    always @(posedge clk) begin
        if (!rst_n) begin
            wb_ack_o      <= 1'b0;
            wb_dat_o      <= 32'h0;
            wb_err_o      <= 1'b0;
            irq_done_o    <= 1'b0;
            start_signal  <= 1'b0;
            cycle_counter <= 64'd0;
            active_cycles <= 32'd0;
        end else begin
            wb_ack_o   <= 1'b0;
            wb_err_o   <= 1'b0;
            irq_done_o <= 1'b0;
            cycle_counter <= cycle_counter + 1'b1;
            if (start_signal)
                active_cycles <= active_cycles + 1'b1;
            if (|core_done_flags)
                irq_done_o <= 1'b1;

            if (wb_cyc_i && wb_stb_i && !wb_ack_o) begin
                wb_ack_o <= 1'b1;
                if (wb_we_i) begin
                    case (wb_adr_i[15:0])
                        16'h0000: begin
                            start_signal <= wb_dat_i[0];
                            core_enable  <= wb_dat_i[15:8];
                        end
                        16'h0004: pump_ctrl <= wb_dat_i[15:8];
                        16'h0008: global_precision <= wb_dat_i[4:0];
                        16'h2000, 16'h2004, 16'h2008, 16'h200C,
                        16'h2010, 16'h2014, 16'h2018, 16'h201C: begin
                            data_mem[wb_adr_i[11:2]] <= wb_dat_i;
                        end
                        16'h3000, 16'h3004, 16'h3008, 16'h300C,
                        16'h3010, 16'h3014, 16'h3018, 16'h301C: begin
                            weight_mem[wb_adr_i[11:2]] <= wb_dat_i;
                        end
                        16'h4000, 16'h4004, 16'h4008, 16'h400C,
                        16'h4010, 16'h4014, 16'h4018, 16'h401C: begin
                            bias_mem[wb_adr_i[11:2]] <= wb_dat_i;
                        end
                        default: wb_err_o <= 1'b1;
                    endcase
                end else begin
                    case (wb_adr_i[15:0])
                        16'h0000: wb_dat_o <= {8'h0, core_thermal_flags, core_done_flags, core_enable, 3'b0, start_signal};
                        16'h0004: wb_dat_o <= {max_temp_recorded, 7'h0, thermal_crit_flag_o, pump_ctrl, liquid_temp_sensor};
                        16'h0008: wb_dat_o <= {27'h0, global_precision};
                        16'h000C: wb_dat_o <= cycle_counter[31:0];
                        16'h0010: wb_dat_o <= cycle_counter[63:32];
                        16'h0014: wb_dat_o <= active_cycles;
                        16'h0018: wb_dat_o <= {16'h0, thermal_events};
                        default: begin
                            wb_dat_o <= 32'hDEAD_BEEF;
                            wb_err_o <= 1'b1;
                        end
                    endcase
                end
            end
        end
    end

    wire [DATA_WIDTH-1:0] core_outputs [0:NUM_CORES-1];

    generate
        genvar i;
        for (i = 0; i < NUM_CORES; i = i + 1) begin : core_gen
            n1_heal_core #(
                .MAX_PRECISION (MAX_PRECISION),
                .DATA_WIDTH    (DATA_WIDTH)
            ) heal_unit (
                .clk                (core_clk[i]),
                .reset              (internal_reset | ~core_enable[i]),
                .start_compute      (start_signal & core_enable[i]),
                .done_flag          (core_done_flags[i]),
                .data_in            (hbm3_bus_in),
                .weight             (weight_mem[i]),
                .bias               (bias_mem[i]),
                .data_out           (core_outputs[i]),
                .liquid_temp_sensor (liquid_temp_sensor),
                .pump_ctrl          (),
                .thermal_crit_flag  (core_thermal_flags[i]),
                .precision_ctrl     (global_precision),
                .thermal_override   (1'b0)
            );
        end
    endgenerate

endmodule