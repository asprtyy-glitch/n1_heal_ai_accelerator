`default_nettype none

module user_project_wrapper (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif
    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,

    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (not used)
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent Clock (not used)
    input user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq
);

    // Internal Reset Signal
    wire rst_n = ~wb_rst_i;

    // N1-HEAL Top Instance
    n1_heal_top #(
        .NUM_CORES(16),
        .DATA_WIDTH(512)
    ) uut (
    `ifdef USE_POWER_PINS
        .vccd1(vccd1),
        .vssd1(vssd1),
    `endif
        .clk                (wb_clk_i),
        .rst_n              (rst_n),
        .wb_stb_i           (wbs_stb_i),
        .wb_cyc_i           (wbs_cyc_i),
        .wb_we_i            (wbs_we_i),
        .wb_sel_i           (wbs_sel_i),
        .wb_dat_i           (wbs_dat_i),
        .wb_adr_i           (wbs_adr_i),
        .wb_ack_o           (wbs_ack_o),
        .wb_dat_o           (wbs_dat_o),
        .wb_err_o           (), // Unused error signal
        .irq_done_o         (user_irq[0]),
        .irq_thermal_o      (user_irq[1]),
        .liquid_temp_sensor (la_data_in[7:0]), // Input via Logic Analyzer
        .pump_ctrl          (io_out[7:0]),     // Output via GPIOs
        .thermal_crit_flag_o(user_irq[2]),
        .sleep_mode_i       (la_data_in[8]),
        .busy_o             (la_data_out[9])
    );

    // GPIO configuration
    assign io_oeb[7:0] = 8'b00000000; // Set as outputs
    assign io_oeb[`MPRJ_IO_PADS-1:8] = {(`MPRJ_IO_PADS-8){1'b1}}; // Set others as inputs

endmodule
`default_nettype wire
