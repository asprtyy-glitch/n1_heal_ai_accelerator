`default_nettype none

//-----------------------------------------------------------------------------
// N1-HEAL Core v1.0
// Single-Core Adaptive Precision MAC Engine with Thermal Awareness
// Process: SkyWater 130nm (SKY130A)
// Author: Ahmed Mhmoude
// Date: 2026-04-26
//-----------------------------------------------------------------------------

module n1_heal_core #(
    parameter MAX_PRECISION = 32,
    parameter MIN_PRECISION = 2,
    parameter DATA_WIDTH    = 512
)(
    input  wire                  clk,
    input  wire                  reset,
    input  wire                  start_compute,
    output reg                   done_flag,
    input  wire [DATA_WIDTH-1:0] data_in,
    input  wire [MAX_PRECISION-1:0] weight,
    input  wire [MAX_PRECISION-1:0] bias,
    output reg  [DATA_WIDTH-1:0] data_out,
    input  wire [7:0]            liquid_temp_sensor,
    output reg  [7:0]            pump_ctrl,
    output reg                   thermal_crit_flag,
    input  wire [4:0]            precision_ctrl,
    input  wire                  thermal_override
);

    localparam PREC_2BIT  = 5'b00001;
    localparam PREC_4BIT  = 5'b00010;
    localparam PREC_8BIT  = 5'b00100;
    localparam PREC_16BIT = 5'b01000;
    localparam PREC_32BIT = 5'b10000;

    localparam TEMP_NORMAL   = 8'd70;
    localparam TEMP_WARNING  = 8'd80;
    localparam TEMP_CRITICAL = 8'd90;
    localparam TEMP_HALT     = 8'd100;

    localparam PUMP_IDLE   = 8'h44;
    localparam PUMP_NORMAL = 8'h66;
    localparam PUMP_MAX    = 8'hFF;

    reg [DATA_WIDTH-1:0]   mul_stage;
    reg [MAX_PRECISION-1:0] mac_result;
    reg [2:0]              thermal_state;
    reg                    computing;
    reg [4:0]              active_precision;

    always @(*) begin
        if (thermal_override)
            active_precision = precision_ctrl;
        else begin
            case (thermal_state)
                3'd0: active_precision = PREC_32BIT;
                3'd1: active_precision = PREC_16BIT;
                3'd2: active_precision = PREC_8BIT;
                3'd3: active_precision = PREC_4BIT;
                default: active_precision = PREC_32BIT;
            endcase
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            data_out   <= {DATA_WIDTH{1'b0}};
            done_flag  <= 1'b0;
            computing  <= 1'b0;
            mul_stage  <= {DATA_WIDTH{1'b0}};
            mac_result <= {MAX_PRECISION{1'b0}};
        end else begin
            done_flag <= 1'b0;
            if (start_compute && !computing) begin
                computing <= 1'b1;
                case (active_precision)
                    PREC_2BIT: begin
                        mul_stage <= data_in[1:0] * weight[1:0];
                        mac_result <= mul_stage[1:0] + bias[1:0];
                    end
                    PREC_4BIT: begin
                        mul_stage <= data_in[3:0] * weight[3:0];
                        mac_result <= mul_stage[3:0] + bias[3:0];
                    end
                    PREC_8BIT: begin
                        mul_stage <= data_in[7:0] * weight[7:0];
                        mac_result <= mul_stage[7:0] + bias[7:0];
                    end
                    PREC_16BIT: begin
                        mul_stage <= data_in[15:0] * weight[15:0];
                        mac_result <= mul_stage[15:0] + bias[15:0];
                    end
                    PREC_32BIT: begin
                        mul_stage <= data_in[31:0] * weight[31:0];
                        mac_result <= mul_stage[31:0] + bias[31:0];
                    end
                    default: begin
                        mul_stage <= data_in[31:0] * weight[31:0];
                        mac_result <= mul_stage[31:0] + bias[31:0];
                    end
                endcase
                data_out <= data_in ^ {{(16){mac_result[31:0]}}};
                done_flag <= 1'b1;
                computing <= 1'b0;
            end
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pump_ctrl         <= PUMP_IDLE;
            thermal_crit_flag <= 1'b0;
            thermal_state     <= 3'd0;
        end else begin
            case (thermal_state)
                3'd0: begin
                    pump_ctrl         <= PUMP_NORMAL;
                    thermal_crit_flag <= 1'b0;
                    if (liquid_temp_sensor > TEMP_WARNING)
                        thermal_state <= 3'd1;
                end
                3'd1: begin
                    pump_ctrl <= PUMP_MAX;
                    if (liquid_temp_sensor > TEMP_CRITICAL)
                        thermal_state <= 3'd2;
                    else if (liquid_temp_sensor < TEMP_NORMAL)
                        thermal_state <= 3'd0;
                end
                3'd2: begin
                    pump_ctrl         <= PUMP_MAX;
                    thermal_crit_flag <= 1'b1;
                    if (liquid_temp_sensor > TEMP_HALT)
                        thermal_state <= 3'd3;
                    else if (liquid_temp_sensor < TEMP_WARNING) begin
                        thermal_state     <= 3'd1;
                        thermal_crit_flag <= 1'b0;
                    end
                end
                3'd3: begin
                    pump_ctrl         <= PUMP_MAX;
                    thermal_crit_flag <= 1'b1;
                end
                default: thermal_state <= 3'd0;
            endcase
        end
    end

endmodule