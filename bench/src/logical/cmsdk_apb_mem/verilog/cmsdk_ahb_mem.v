module cmsdk_ahb_mem(
// ----------------------------------------------------------------------------
// Port Definitions
// ----------------------------------------------------------------------------
   // AHB Inputs
   input  wire                 HCLK,      // system bus clock
   input  wire                 HRESETn,   // system bus reset
   input  wire                 FCLK,      // system bus clock
   input  wire                 HSEL,      // AHB peripheral select
   input  wire                 HREADY,    // AHB ready input
   input  wire  [1:0]          HTRANS,    // AHB transfer type
   input  wire  [2:0]          HSIZE,     // AHB hsize
   input  wire                 HWRITE,    // AHB hwrite
   input  wire [31:0]          HADDR,     // AHB address bus
   input  wire [31:0]          HWDATA,    // AHB write data bus

   input  wire  [3:0]          ECOREVNUM,  // Engineering-change-order revision bits

   // AHB Outputs
   output wire                 HREADYOUT, // AHB ready output to S->M mux
   output wire                 HRESP,     // AHB response
   output wire [31:0]          HRDATA,

   output wire  [31:0]         wdata,
   output wire  [21:0]         waddr,
   output wire                 w_en,
   input  wire  [31:0]         rdata,
   output wire  [21:0]         raddr,
   output wire                 r_en,
   output wire  [15:0]         mem_cs);   // Combined interrupt

//write data
   wire data_valid;
   assign data_valid = HSEL & HREADY & HTRANS[1];

   wire   w_data_en;
   assign w_data_en = data_valid & HWRITE;

   wire   r_data_en;
   assign r_data_en = data_valid & ~HWRITE;

   reg  w_data_en_t;
   always @ (posedge HCLK or negedge HRESETn) begin
     if(!HRESETn) w_data_en_t <= 1'b0;
     else w_data_en_t <= w_data_en;     
   end

   reg [31:0] addr;
   always @ (posedge HCLK or negedge HRESETn) begin
     if(!HRESETn) addr <= 32'h0;
     else addr <= HADDR;     
   end

   assign wdata = HWDATA;
   assign waddr = addr[23:2];
   assign w_en  = w_data_en_t;

   assign HRDATA = rdata;
   assign raddr  = HADDR[23:2];
   assign r_en   = r_data_en;

   assign mem_cs[0]  = ((addr[27:24] == 4'h0 && w_data_en_t) || (HADDR[27:24] == 4'h0 && r_data_en));  
   assign mem_cs[1]  = ((addr[27:24] == 4'h1 && w_data_en_t) || (HADDR[27:24] == 4'h1 && r_data_en));  
   assign mem_cs[2]  = ((addr[27:24] == 4'h2 && w_data_en_t) || (HADDR[27:24] == 4'h2 && r_data_en));  
   assign mem_cs[3]  = ((addr[27:24] == 4'h3 && w_data_en_t) || (HADDR[27:24] == 4'h3 && r_data_en));  
   assign mem_cs[4]  = ((addr[27:24] == 4'h4 && w_data_en_t) || (HADDR[27:24] == 4'h4 && r_data_en));  
   assign mem_cs[5]  = ((addr[27:24] == 4'h5 && w_data_en_t) || (HADDR[27:24] == 4'h5 && r_data_en));  
   assign mem_cs[6]  = ((addr[27:24] == 4'h6 && w_data_en_t) || (HADDR[27:24] == 4'h6 && r_data_en));  
   assign mem_cs[7]  = ((addr[27:24] == 4'h7 && w_data_en_t) || (HADDR[27:24] == 4'h7 && r_data_en));  
   assign mem_cs[8]  = ((addr[27:24] == 4'h8 && w_data_en_t) || (HADDR[27:24] == 4'h8 && r_data_en));  
   assign mem_cs[9]  = ((addr[27:24] == 4'h9 && w_data_en_t) || (HADDR[27:24] == 4'h9 && r_data_en));  
   assign mem_cs[10] = ((addr[27:24] == 4'ha && w_data_en_t) || (HADDR[27:24] == 4'ha && r_data_en));   
   assign mem_cs[11] = ((addr[27:24] == 4'hb && w_data_en_t) || (HADDR[27:24] == 4'hb && r_data_en));  
   assign mem_cs[12] = ((addr[27:24] == 4'hc && w_data_en_t) || (HADDR[27:24] == 4'hc && r_data_en));  
   assign mem_cs[13] = ((addr[27:24] == 4'hd && w_data_en_t) || (HADDR[27:24] == 4'hd && r_data_en));  
   assign mem_cs[14] = ((addr[27:24] == 4'he && w_data_en_t) || (HADDR[27:24] == 4'he && r_data_en));  
   assign mem_cs[15] = ((addr[27:24] == 4'hf && w_data_en_t) || (HADDR[27:24] == 4'hf && r_data_en));  

   assign HREADYOUT = 1'b1;
   assign HRESP     = 1'b0;

endmodule