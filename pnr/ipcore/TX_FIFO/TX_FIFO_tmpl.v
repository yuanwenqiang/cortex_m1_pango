// Created by IP Generator (Version 2020.2-Beta2 build 58463)
// Instantiation Template
//
// Insert the following codes into your Verilog file.
//   * Change the_instance_name to your own instance name.
//   * Change the signal names in the port associations


ipml_fifo_wrapper_v1_5 the_instance_name (
  .clk(clk),                      // input
  .rst(rst),                      // input
  .wr_en(wr_en),                  // input
  .wr_data(wr_data),              // input [127:0]
  .wr_full(wr_full),              // output
  .almost_full(almost_full),      // output
  .rd_en(rd_en),                  // input
  .rd_data(rd_data),              // output [7:0]
  .rd_empty(rd_empty),            // output
  .almost_empty(almost_empty)     // output
);
