module m1_soc_top (
	input             ex_clk_50m,
	input             rst_key,
     input             gpio_in0,
//    input             gpio_in1,
//debug
//    input             pad_nTRST,
//    input             pad_nRST,    
//    input             pad_TCK,
//    input             pad_TMS,  
//    input             pad_TDI,  
//    output            pad_TDO, 

//    output  [7:0]     gpio_out,

    input             RX,
    output            TX,

    output            spi0_clk,
    output            spi0_cs,
    output            spi0_mosi,
    input             spi0_miso,

    inout             i2c0_sck,
    inout             i2c0_sda,
//ddr
	input  		      pad_loop_in,
    input  		      pad_loop_in_h,
    output 		      pad_rstn_ch0,
    output 		      pad_ddr_clk_w,
    output 		      pad_ddr_clkn_w,
    output 		      pad_csn_ch0,
    output [15:0]     pad_addr_ch0,
    inout  [16-1:0]   pad_dq_ch0,
    inout  [16/8-1:0] pad_dqs_ch0,
    inout  [16/8-1:0] pad_dqsn_ch0,
    output [16/8-1:0] pad_dm_rdqs_ch0,
    output 		      pad_cke_ch0,
    output 		      pad_odt_ch0,
    output 		      pad_rasn_ch0,
    output 		      pad_casn_ch0,
    output 		      pad_wen_ch0,
    output [2:0]      pad_ba_ch0,
    output 		      pad_loop_out,
    output 		      pad_loop_out_h,

   // output            phy_rst_n,        
    input             rx_clki,          
    input             phy_rx_dv,        
    input             phy_rxd0,         
    input             phy_rxd1,         
    input             phy_rxd2,         
    input             phy_rxd3,         

    output           SD_nCS,                  
    output           SD_DCLK,               
    output           SD_MOSI,               
    input            SD_MISO,               

    input             cmos_vsync,       
    input             cmos_href,        
    input             cmos_pclk,       
    output            cmos_xclk,         
    input [7:0]       cmos_db,

//lcd output
	output            lcd_dclk,	
	output            lcd_hs,            //lcd horizontal synchronization
	output            lcd_vs,            //lcd vertical synchronization        
	output            lcd_de,            //lcd data enable     
	output[7:0]       lcd_r,             //lcd red
	output[7:0]       lcd_g,             //lcd green
	output[7:0]       lcd_b,	          //lcd blue
	

    output            l0_sgmii_clk_shft,
    output            phy_tx_en,
    output            phy_txd0,
    output            phy_txd1,
    output            phy_txd2,
    output            phy_txd3,

    output  [3:0]     LED

); 

parameter VS_DIV				= 8'd10;
parameter DATA_RAM_BITS			= 13;

parameter MEM_DATA_BITS         = 64  ;            //external memory user interface data width
parameter ADDR_BITS             = 25  ;            //external memory user interface address width
parameter BUSRT_BITS            = 10  ;            //external memory user interface burst width
parameter DATA_BITS				= 16  ;
parameter BURST_SIZE			= 64;


    `ifdef SIMULATION
        `include "../bench/src/m1_core/cm1_option_defs.v"
    `else
        `include "m1_core/cm1_option_defs.v"
    `endif

    parameter TH_1S = 27'd33000000;

    reg [26:0] cnt;
    reg heart_beat_led;

    wire              HCLK;           // AHB Memory clock
    wire              HREADYOUT;      // AHB Memory ready
    wire              HRESP;          // AHB Memory response
    wire   [31:0]     HRDATA;         // AHB Memory read data bus
    wire              HSEL;           // AHB Memory select
    wire   [1:0]      HTRANS;         // AHB Memory transaction type
    wire   [2:0]      HBURST;         // AHB Memory burst information
    wire   [3:0]      HPROT;          // AHB Memory protection control
    wire   [2:0]      HSIZE;          // AHB Memory transfer size
    wire              HWRITE;         // AHB Memory transfer direction
    wire              HMASTLOCK;      // AHB Memory locked transfer
    wire   [31:0]     HADDR;          // AHB Memory transfer address
    wire   [31:0]     HWDATA;         // AHB Memory write data bus
    wire              HREADY;         // AHB Memory bus ready   
    wire   [31:0]     HRDATAmux;                 

    wire   [11:0]     PADDR;   
    wire              PWRITE;  
    wire   [31:0]     PWDATA;  
    wire              PENABLE; 
    wire              PSEL;    
    wire   [31:0]     PRDATA;  
    wire              PREADY; 
    wire              PSLVERR;
  

    wire   [15:0]     p0_in;          // GPIO 0 inputs
    wire   [15:0]     p0_out;         // GPIO 0 outputs
    wire   [15:0]     p0_outen;       // GPIO 0 output enables
    wire   [15:0]     p0_altfunc;     // GPIO 0 alternate function (pin mux)
                     
    wire              watchdog_reset;
                     
    wire   [31:0]     wdata;
    wire   [21:0]     waddr;
    wire              w_en;
    wire   [31:0]     rdata;
    wire   [21:0]     raddr;
    wire              r_en;
    wire   [15:0]     mem_cs;
              
    wire   [7:0]      tsmac_tdata;
    wire              tsmac_tstart;
    wire              tsmac_tpnd;
    wire              tsmac_tlast;

    wire   [7:0]      tsmac_rdata;
    wire              tsmac_rvalid;
    wire              tsmac_rlast;
       
    //AXI WRITE
    wire              pll_pclk;
    wire              ddr_pll_lock;
    wire              ddrphy_rst_done;
    wire              ddrc_init_done;
                      
    wire              aclk_mux;
    wire   [31:0]     awaddr_mux;
    wire   [7:0]      awlen_mux;
    wire              awvalid_mux;
    wire              awready_mux;
    wire   [127:0]    wdata_mux;
    wire   [15:0]     wstrb_mux;
    wire              wlast_mux;
    wire              wvalid_mux;
    wire              wready_mux;
                      
    //AXI RAD         
    wire   [31:0]     araddr_mux;
    wire   [7:0]      arlen_mux;
    wire              arvalid_mux;
    wire              arready_mux;
    wire   [127:0]    rdata_mux;
    wire              rlast_mux;
    wire              rvalid_mux;
    wire              rready_mux;

    wire              aclk_1;
    wire   [31:0]     awaddr_1;
    wire              awvalid_1;
    wire              awready_1;
    wire   [63:0]     wdata_1;
    wire              wlast_1;
    wire              wvalid_1;
    wire              wready_1;
                      
    //AXI RAD         
    wire   [31:0]     araddr_1;
    wire              arvalid_1;
    wire              arready_1;
    wire   [63:0]     rdata_1;
    wire              rlast_1;
    wire              rvalid_1;
    wire              rready_1;
   
    wire              tsmac_phy_pselx;
    wire              tsmac_phy_pwrite;
    wire              tsmac_phy_penable;
    wire   [7:0]      tsmac_phy_paddr;
    wire   [31:0]     tsmac_phy_pwdata;
                      
    wire              tsmac_phy_mdo;
    wire              tsmac_phy_mdoen;
    wire              tsmac_phy_mdi;
    wire              tsmac_phy_mdio;
                      
    wire   [3:0]      phy_txd;
    wire   [3:0]      phy_rxd;

    wire   [7:0]      m1_tdata;
    wire              m1_tstart;
    wire              m1_tpnd;
    wire              m1_tlast;

    wire   [9:0]      ethernet_fifo_rd_data;
    wire   [9:0]      tsmac_fifo_wr_data;

    wire              HREADYOUT_udp;
    wire              HRESP_udp;
    wire   [31:0]     HRDATA_udp;

    wire   [7:0]      udp_tdata;
    wire              udp_tstart;
    wire              udp_tpnd;
    wire              udp_tlast;

    wire              udp_cs;

    //wire             gpio_in0;
    wire             gpio_in1;
      

wire                            ui_clk;
wire                            ui_clk_sync_rst;
wire                            init_calib_complete;
wire[7:0]                       read_data_eep;


wire                            wr_burst_data_req;
wire                            wr_burst_finish;
wire                            rd_burst_finish;
wire                            rd_burst_req;
wire                            wr_burst_req;
wire[BUSRT_BITS - 1:0]          rd_burst_len;
wire[BUSRT_BITS - 1:0]          wr_burst_len;
wire[ADDR_BITS - 1:0]           rd_burst_addr;
wire[ADDR_BITS - 1:0]           wr_burst_addr;
wire                            rd_burst_data_valid;
wire[MEM_DATA_BITS - 1 : 0]     rd_burst_data;
wire[MEM_DATA_BITS - 1 : 0]     wr_burst_data;
wire                            read_req;
wire                            read_req_ack;
wire                            read_en;
wire[15:0]                      read_data/*synthesis syn_keep=1*/;
wire                            write_en;
wire[15:0]                      write_data/*synthesis syn_keep=1*/;
wire                            write_req;
wire                            write_req_ack;

wire                            video_clk;         //video pixel clock
wire                            video_clk5x;
wire                            hs;
wire                            vs;
wire                            de;
wire[15:0]                      vout_data;
wire[15:0]                      cmos_16bit_data;
wire                            cmos_16bit_wr;
wire[1:0]                       write_addr_index;
wire[1:0]                       read_addr_index;
    // Master Write Address
wire [3:0]                      s00_axi_awid;
wire [63:0]                     s00_axi_awaddr;
wire [7:0]                      s00_axi_awlen;    // burst length: 0-255
wire [2:0]                      s00_axi_awsize;   // burst size: fixed 2'b011
wire [1:0]                      s00_axi_awburst;  // burst type: fixed 2'b01(incremental burst)
wire                            s00_axi_awlock;   // lock: fixed 2'b00
wire [3:0]                      s00_axi_awcache;  // cache: fiex 2'b0011
wire [2:0]                      s00_axi_awprot;   // protect: fixed 2'b000
wire [3:0]                      s00_axi_awqos;    // qos: fixed 2'b0000
wire [0:0]                      s00_axi_awuser;   // user: fixed 32'd0
wire                            s00_axi_awvalid;
wire                            s00_axi_awready;
// master write data
wire [63:0]                     s00_axi_wdata;
wire [7:0]                      s00_axi_wstrb;
wire                            s00_axi_wlast;
wire [0:0]                      s00_axi_wuser;
wire                            s00_axi_wvalid;
wire                            s00_axi_wready;
// master write response
wire [3:0]                      s00_axi_bid;
wire [1:0]                      s00_axi_bresp;
wire [0:0]                      s00_axi_buser;
wire                            s00_axi_bvalid;
wire                            s00_axi_bready;
// master read address
wire [3:0]                      s00_axi_arid;
wire [63:0]                     s00_axi_araddr;
wire [7:0]                      s00_axi_arlen;
wire [2:0]                      s00_axi_arsize;
wire [1:0]                      s00_axi_arburst;
wire [1:0]                      s00_axi_arlock;
wire [3:0]                      s00_axi_arcache;
wire [2:0]                      s00_axi_arprot;
wire [3:0]                      s00_axi_arqos;
wire [0:0]                      s00_axi_aruser;
wire                            s00_axi_arvalid;
wire                            s00_axi_arready;
// master read data
wire [3:0]                      s00_axi_rid;
wire [63:0]                     s00_axi_rdata;
wire [1:0]                      s00_axi_rresp;
wire                            s00_axi_rlast;
wire [0:0]                      s00_axi_ruser;
wire                            s00_axi_rvalid;
wire                            s00_axi_rready;
wire                            clk_200MHz;

wire                             cmos_xclk_w;
wire                             cmos_pclk_g;

//=========================================================//
//========================wires============================//
//=========================================================//

//====================frame wires=======================//
wire                            wr_burst_req_frame;
wire[BUSRT_BITS - 1:0]          wr_burst_len_frame;
wire[ADDR_BITS - 1:0]           wr_burst_addr_frame;
wire[MEM_DATA_BITS - 1 : 0]     wr_burst_data_frame;

wire                            rd_burst_req_frame;
wire[BUSRT_BITS - 1:0]          rd_burst_len_frame;
wire[ADDR_BITS - 1:0]           rd_burst_addr_frame;
//====================down sample wires=======================//
wire [DATA_RAM_BITS-1:0]		down_rd_addr;
wire [15:0]			            down_rd_data;
//================== flash wires=====================//
wire 							wr_burst_req_flash;
wire [BUSRT_BITS-1:0]			wr_burst_len_flash;
wire [ADDR_BITS-1:0]			wr_burst_addr_flash;
wire 							wr_burst_data_req_flash;
wire [MEM_DATA_BITS-1:0]		wr_burst_data_flash;
wire 							wr_burst_finish_flash;

wire							flash_read_finish;
wire							uart_tx_flash;
//================== coe wires=====================//
//read
wire							rd_burst_req_coe;
wire [BUSRT_BITS-1:0]			rd_burst_len_coe;
wire [ADDR_BITS-1:0]			rd_burst_addr_coe;
wire [3:0]						debug;

wire							read_req_coe;
wire							read_req_ack_coe;
wire [ADDR_BITS-1:0]			read_addr_coe;
wire [ADDR_BITS-1:0]			read_len_coe;
wire							read_finish_coe;
wire [DATA_BITS-1:0]			read_data_coe;
wire							read_data_valid_coe;
wire							read_en_coe;
wire							uart_tx_coetb;
//write
wire 							wr_burst_req_coe;
wire [BUSRT_BITS-1:0]			wr_burst_len_coe;
wire [ADDR_BITS-1:0]			wr_burst_addr_coe;
wire 							wr_burst_data_req_coe;
wire [MEM_DATA_BITS-1:0]			wr_burst_data_coe;
wire 							wr_burst_finish_coe;

wire 							write_req_coe;
wire 							write_req_ack_coe;
wire [ADDR_BITS-1:0]			write_addr_coe;
wire [ADDR_BITS-1:0]			write_len_coe;
wire 							write_en_coe;
wire [DATA_BITS-1:0]			write_data_coe;
wire 							write_finish_coe;




//=====================lcd wires====================//
assign lcd_hs = hs;
assign lcd_vs = vs;
assign lcd_de = de;
assign lcd_r  = {vout_data[15:11],3'd0};
assign lcd_g  = {vout_data[10:5],2'd0};
assign lcd_b  = {vout_data[4:0],3'd0};
assign lcd_dclk = ~video_clk;

//=======================cmos data=====================//
assign write_en = cmos_16bit_wr;
assign write_data = {cmos_16bit_data[4:0],cmos_16bit_data[10:5],cmos_16bit_data[15:11]};


 //CLK----------------------------------------------------------------
    // sysclk source
	`ifdef UNCACHE
	PLL u_PLL (
        .clkin1               (ex_clk_50m), // input
        .pll_lock             (pll_lock),   // output
        .clkout0              (HCLK),       // output  50M
        
    );
	`else
		assign HCLK = aclk_mux;
	`endif

PLL cmos_PLL (
        .clkin1               (ex_clk_50m), // input
        .clkout0              (cmos_xclk_w),       // output
        .clkout1              (video_clk)   // output  9M
    );



GTP_CLKBUFG cmos_pclkbufg            //PCK����BUF
(
  .CLKOUT                (cmos_pclk_g    ),      
  .CLKIN                 (cmos_pclk      )
);

GTP_CLKBUFG u_cmos_xclk               //XCLK���BUF
(
   .CLKOUT               (cmos_xclk),
   .CLKIN                (cmos_xclk_w)
 );

//Soft Reset---------------------------------------------------------  
    wire   DBGRESETn;
    wire   SYSRESETREQ;
    wire   SYSRESETn;

    rst_gen u_rst_gen(
        .HCLK                 (HCLK),           // input
        .pad_nRST             (1'b1),       // input  
        .ddrc_init_done       (ddrc_init_done), // input
        .watchdog_reset       (watchdog_reset), // input
        .SYSRESETREQ          (SYSRESETREQ),    // input
    
        .DBGRESETn            (DBGRESETn),      // output
        .SYSRESETn            (SYSRESETn)       // output
    );

//M1 CORE------------------------------------------------------------ 
    assign LED[1:0]    = p0_outen[1:0]  & p0_out[1:0];
    assign LED[2]      = ddrc_init_done;
    assign LED[3]      = heart_beat_led;
    assign gpio_out = p0_outen[15:8] & p0_out[15:8];

    wire [7:0] spi0_cs_0;
    assign spi0_cs = spi0_cs_0[0];

    wire scl_pad_i;
    wire scl_pad_o;
    wire sda_pad_i;
    wire sda_pad_o;

    assign i2c0_sck  = scl_pad_o ? 1'bz : 1'b0;
    assign i2c0_sda  = sda_pad_o ? 1'bz : 1'b0;
    assign scl_pad_i = i2c0_sck;
    assign sda_pad_i = i2c0_sda;

    assign m1_tpnd = tsmac_tpnd && ~udp_cs;

// wire done_init;
// wire[9:0]                       lut_index;
// wire[31:0]                      lut_data;
// //=========================iic======================//
// i2c_config i2c_config_m0(
// 	.rst                        (~rst_key                   ),
// 	.clk                        (ex_clk_50m                ),
// 	.clk_div_cnt                (16'd499                  ),
// 	.i2c_addr_2byte             (1'b1                     ),
// 	.lut_index                  (lut_index                ),
// 	.lut_dev_addr               (lut_data[31:24]          ),
// 	.lut_reg_addr               (lut_data[23:8]           ),
// 	.lut_reg_data               (lut_data[7:0]            ),
// 	.error                      (                         ),
// 	.done                       (done_init                ),//keep high after init
// 	.i2c_scl                    (i2c0_sck                 ),
// 	.i2c_sda                    (i2c0_sda                 )
// );
// lut_ov5640_rgb565_480_272 lut_ov5640_rgb565_480_272_m0(
// 	.lut_index                  (lut_index                ),
// 	.lut_data                   (lut_data                 )
// );

//cnn-----------------------------------------------------------------------
    cnn_top u_cnn_top(
        .HCLK              (HCLK),
        .HRESETn           (SYSRESETn),
        .HSEL              (HSEL),
        .HTRANS            (HTRANS),
        .HSIZE             (HSIZE),
        .HBURST		       (HBURST),
        .HPROT             (HPROT),
        .HWDATA            (HWDATA),
        .HWRITE            (HWRITE),
        .HADDR             (HADDR),
        .HREADY            (HREADY),

        .HREADYOUT         (HREADYOUT),
        .HRESP             (HRESP),
        .HRDATA            (HRDATA)
);

//======================M1 core=========================//
    integration_kit_dbg u_integration_kit_dbg(   
		 // Inputs
        .HCLK              (HCLK),           // System Clock
        .SYSRESETn         (SYSRESETn),      // System Reset
        .DBGRESETn         (DBGRESETn),      // Debug Reset
      
        .nTRST             (pad_nTRST),      // JTAG reset
        .SWCLKTCK          (pad_TCK),        // Serial wire and JTAG clock
        .SWDITMS           (pad_TMS),        // SW Data / JTAG Test Mode Select
        .TDI               (pad_TDI),        // JTAG data input
        
        .EDBGRQ            (1'b0),           // Debug request
        .DBGRESTART        (1'b0),           // Restart from halt request

        .SYSRESETREQ       (SYSRESETREQ),    // Cortex-M1 SYSRESET request
        .LOCKUP            (),               // Cortex-M1 lockup
        .HALTED            (),               // Cortex-M1 halted
        .DBGRESTARTED      (),               // Restart from halt acknowledge
        
        .JTAGNSW           (),               // JTAG = 1, serial wire = 0
        .JTAGTOP           (),               // state controller indicator
        .TDO               (pad_TDO),        // JTAG data output
        .nTDOEN            (),               // JTAG data out enable
        .SWDO              (),               // Serial wire data out
        .SWDOEN            (),               // Serial data output enable

        //AHB AHB[31:28] = 4'h7,4'h8,4'h9    AHB[27:0] = 28'h000_0000 ~ 28'hfff_ffff
        .HSEL              (HSEL),           // AHB Memory select
        .HTRANS            (HTRANS),         // AHB Memory transaction type
        .HBURST            (HBURST),         // AHB Memory burst information
        .HPROT             (HPROT),          // AHB Memory protection control
        .HSIZE             (HSIZE),          // AHB Memory transfer size
        .HWRITE            (HWRITE),         // AHB Memory transfer direction
        .HMASTLOCK         (HMASTLOCK),      // AHB Memory locked transfer
        .HADDR             (HADDR),          // AHB Memory transfer address
        .HWDATA            (HWDATA),         // AHB Memory write data bus
        .HREADY            (HREADY),         // AHB Memory bus ready
        .HREADYOUT         (HREADYOUT),  // AHB Memory ready
        .HRESP             (HRESP),      // AHB Memory response
        .HRDATA            (HRDATA),     // AHB Memory read data bus  
        .HRDATAmux         (HRDATAmux),

        //APB AHB[11:0]    AHB[31:0] = 32'h5000c000 ~ 32'h5000cfff
        .PADDR             (PADDR),   
        .PWRITE            (PWRITE),    
        .PWDATA            (PWDATA),    
        .PENABLE           (PENABLE),   
        .PSEL              (PSEL),      
        .PRDATA            (PRDATA),   
        .PREADY            (1'b1),  
        .PSLVERR           (1'b0),   

        //Periphral
        //GPIO
        .p0_out            (p0_out),         // GPIO 0 outputs
        .p0_outen          (p0_outen),       // GPIO 0 output enables
        .p0_altfunc        (),               // GPIO 0 alternate function (pin mux)
        .p0_in             ({{14{1'b0}}, gpio_in1, gpio_in0}),          // GPIO 0 inputs

        //UART0
        .RX0               (RX),
        .TX0               (TX),
        //UART1
        .RX1               (),
        .TX1               (),
        //WATCHDOG
        .watchdog_reset    (watchdog_reset),
        //SPI
        .spi0_clk          (spi0_clk),
        .spi0_cs           (spi0_cs_0),
        .spi0_mosi         (spi0_mosi),
        .spi0_miso         (spi0_miso),

        //I2C 
        .scl_pad_i         (scl_pad_i),
        .scl_pad_o         (scl_pad_o),
        .sda_pad_i         (sda_pad_i),
        .sda_pad_o         (sda_pad_o),

        //MEM        
        .wdata             (wdata),
        .waddr             (waddr),
        .w_en              (w_en),
        .rdata             (rdata),
        .raddr             (raddr),
        .r_en              (r_en),
        .mem_cs            (mem_cs),

        //TSMAC
        .tsmac_tdata       (m1_tdata),
        .tsmac_tstart      (m1_tstart),
        .tsmac_tpnd        (m1_tpnd),
        .tsmac_tlast       (m1_tlast),

        .tsmac_rdata       (ethernet_fifo_rd_data[7:0]),
        .tsmac_rvalid      (ethernet_fifo_rd_data[8]),
        .tsmac_rlast       (ethernet_fifo_rd_data[9]),

        //DDR    
        .aclk_mux          (HCLK),              
        .awaddr_mux        (awaddr_mux),      
        .awlen_mux         (awlen_mux),            
        .awvalid_mux       (awvalid_mux),     
        .awready_mux       (awready_mux),        
        .wdata_mux         (wdata_mux),       
        .wstrb_mux         (wstrb_mux),       
        .wlast_mux         (wlast_mux),       
        .wvalid_mux        (wvalid_mux),      
        .wready_mux        (wready_mux),      
       
        .araddr_mux        (araddr_mux),      
        .arlen_mux         (arlen_mux),           
        .arvalid_mux       (arvalid_mux),     
        .arready_mux       (arready_mux),            
        .rdata_mux         (rdata_mux),            
        .rlast_mux         (rlast_mux),       
        .rvalid_mux        (rvalid_mux),      
        .rready_mux        (rready_mux)      
    );   


//UDP_HW_SPEEDUP--------------------------------------------------------
    wire HSEL_temp;
    assign HSEL_temp = HSEL && (HADDR[31:28] == 4'h7);

	reg [7:0] test_datai;
	reg       test_dbusy;
    wire      dready;

	always @(posedge HCLK or negedge SYSRESETn) begin
      if (~SYSRESETn) begin
        test_datai <= {8{1'b0}};
	    test_dbusy <= 1'b0;
	  end
      else if(dready) begin
        test_datai <= test_datai + 1'b1;
		test_dbusy <= 1'b1;
	  end
      else begin
        test_datai <= {8{1'b0}};
	    test_dbusy <= 1'b0;
	  end
    end

    assign udp_tpnd = tsmac_tpnd && udp_cs;

    // UDP SPEEDUP is driven from the AHB
    generate if (`CORTEXM1_AHB_UDP == 1) begin : gen_udp_hw_speedup_0
    udp_hw_speedup u_udp_hw_speedup(
        .HCLK                   (HCLK),         // system bus clock
        .HRESETn                (SYSRESETn),    // system bus reset
                                      
        .datai                  (test_datai),   // source data
        .dbusy                  (test_dbusy),   // source data valid  
        .dready                 (dready),       // hw_speedup_ready
                                                                                            
        .HSEL                   (HSEL_temp),    // AHB peripheral select
        .HREADY                 (HREADY),       // AHB ready input
        .HTRANS                 (HTRANS),       // AHB transfer type
        .HSIZE                  (HSIZE),        // AHB hsize
        .HWRITE                 (HWRITE),       // AHB hwrite
        .HADDR                  (HADDR),        // AHB address bus
        .HWDATA                 (HWDATA),       // AHB write data bus  
                         
        .HREADYOUT              (HREADYOUT_udp),// AHB ready output to S->M mux
        .HRESP                  (HRESP_udp),    // AHB response
        .HRDATA                 (HRDATA_udp),
    
        .udp_tdata              (udp_tdata),    // TSMAC tx data
        .udp_tstart             (udp_tstart),   // TSMAC tx start
        .udp_tpnd               (udp_tpnd),     // TSMAC tx going
        .udp_tlast              (udp_tlast),    // TSMAC tx end

        .udp_cs                 (udp_cs)        // UDP tx valid
    );
    end else
    begin : gen_no_udp_hw_speedup_0
      assign dready               = 1'b0;
    
      assign HREADYOUT_udp        = 1'b1;
      assign HRESP_udp            = 1'b0;
      assign HRDATA_udp           = {32{1'b0}};
    
      assign udp_tdata            = {8{1'b0}};
      assign udp_tstart           = 1'b0;
      assign udp_tlast            = 1'b0;
    
      assign udp_cs               = 1'b0;
    end endgenerate
    
//MEM-------------------------------------------------------------------
    wire a_wr_en;
    assign a_wr_en = w_en | ~r_en;

    wire [31:0] rdata0;
    assign rdata = rdata0;

    TEST_RAM u_TEST_RAM (
      .wr_data                  (wdata),        // input  [31:0]
      .wr_addr                  (waddr[7:0]),   // input  [7:0]
      .wr_en                    (a_wr_en),      // input
      .wr_clk                   (HCLK),         // input
      .wr_clk_en                (mem_cs[0]),    // input
      .wr_rst                   (1'b0),         // input

      .rd_addr                  (raddr[7:0]),   // input  [7:0]
      .rd_data                  (rdata0),       // output [31:0]
      .rd_clk                   (HCLK),         // input
      .rd_rst                   (1'b0)          // input
    );

//=========================8 to 16======================//
cmos_8_16bit cmos_8_16bit_m0(
	.rst                        (~SYSRESETn                    ),
	.pclk                       (cmos_pclk_g              ),
	.pdata_i                    (cmos_db                  ),
	.de_i                       (cmos_href                ),
	.pdata_o                    (cmos_16bit_data          ),
	.hblank                     (                         ),
	.de_o                       (cmos_16bit_wr            )
);
//=========================cmos======================//
cmos_write_req_gen #(
    .VS_DIV						( 10)
)cmos_write_req_gen_m0(
	.rst                        (~SYSRESETn                   ),
	.pclk                       (cmos_pclk_g              ),
	.cmos_vsync                 (cmos_vsync               ),
	.write_req                  (write_req                ),
	.write_addr_index           (write_addr_index         ),
	.read_addr_index            (read_addr_index          ),
	.write_req_ack              (write_req_ack            )
);


//=========================lcd======================//
video_timing_data#(
	.DATA_WIDTH (16)                       // Video data one clock data width
)video_timing_data_m0 (
	.video_clk                  (video_clk                ),
	.rst                        (~SYSRESETn                ),
	.read_req                   (read_req                 ),
	.read_req_ack               (read_req_ack             ),
	.read_en                    (read_en                  ),
	.read_data                  (read_data                ),
	.hs                         (hs                       ),
	.vs                         (vs                       ),
	.de                         (de                       ),
	.vout_data                  (vout_data                )
);


//存储原始图像数据-抽帧
down_sample_data #(
	.X_AXIS_I					(480),
	.Y_AXIS_I					(272),
	.X_AXIS_O					(120),
	.Y_AXIS_O					(68),
	.FIFO_BITS					(13),
	.DATA_BITS					(16)
)u_down_sample_data(
	.rst						(write_req_ack | ~SYSRESETn),

	.wr_clk						(cmos_pclk),
	.wr_en						(write_en),
	.wr_data					(write_data),

	.rd_clk						(cmos_pclk),
	.rd_addr					(down_rd_addr),
	.rd_data					(down_rd_data)
);


assign write_en = cmos_16bit_wr;
//DDR------------------------------------------------------------------
    always@(posedge HCLK or negedge rst_key)
    begin
       if (!rst_key)
          cnt <= 27'd0;
       else if ( cnt >= TH_1S )
          cnt <= 27'd0;
       else
          cnt <= cnt + 27'd1;
    end
    
    always @(posedge HCLK or negedge rst_key)
    begin
       if (!rst_key)
          heart_beat_led <= 1'd1;
       else if ( cnt >= TH_1S )
          heart_beat_led <= ~heart_beat_led;
    end

aq_axi_master u_aq_axi_master
	(
      .ARESETN                     (SYSRESETn                                 ),
	 // .ARESETN                     (~ui_clk_sync_rst                          ),
	  .ACLK                        (aclk_1                                    ),
	  .M_AXI_AWID                  (s00_axi_awid                              ),
	  .M_AXI_AWADDR                (s00_axi_awaddr                            ),
	  .M_AXI_AWLEN                 (s00_axi_awlen                             ),
	  .M_AXI_AWSIZE                (s00_axi_awsize                            ),
	  .M_AXI_AWBURST               (s00_axi_awburst                           ),
	  .M_AXI_AWLOCK                (s00_axi_awlock                            ),
	  .M_AXI_AWCACHE               (s00_axi_awcache                           ),
	  .M_AXI_AWPROT                (s00_axi_awprot                            ),
	  .M_AXI_AWQOS                 (s00_axi_awqos                             ),
	  .M_AXI_AWUSER                (s00_axi_awuser                            ),
	  .M_AXI_AWVALID               (s00_axi_awvalid                           ),
	  .M_AXI_AWREADY               (s00_axi_awready                           ),
	  .M_AXI_WDATA                 (s00_axi_wdata                             ),
	  .M_AXI_WSTRB                 (s00_axi_wstrb                             ),
	  .M_AXI_WLAST                 (s00_axi_wlast                             ),
	  .M_AXI_WUSER                 (s00_axi_wuser                             ),
	  .M_AXI_WVALID                (s00_axi_wvalid                            ),
	  .M_AXI_WREADY                (s00_axi_wready                            ),
	  .M_AXI_BID                   (s00_axi_bid                               ),
	  .M_AXI_BRESP                 (s00_axi_bresp                             ),
	  .M_AXI_BUSER                 (s00_axi_buser                             ),
	  .M_AXI_BVALID                (s00_axi_bvalid                            ),
	  .M_AXI_BREADY                (s00_axi_bready                            ),
	  .M_AXI_ARID                  (s00_axi_arid                              ),
	  .M_AXI_ARADDR                (s00_axi_araddr                            ),
	  .M_AXI_ARLEN                 (s00_axi_arlen                             ),
	  .M_AXI_ARSIZE                (s00_axi_arsize                            ),
	  .M_AXI_ARBURST               (s00_axi_arburst                           ),
	  .M_AXI_ARLOCK                (s00_axi_arlock                            ),
	  .M_AXI_ARCACHE               (s00_axi_arcache                           ),
	  .M_AXI_ARPROT                (s00_axi_arprot                            ),
	  .M_AXI_ARQOS                 (s00_axi_arqos                             ),
	  .M_AXI_ARUSER                (s00_axi_aruser                            ),
	  .M_AXI_ARVALID               (s00_axi_arvalid                           ),
	  .M_AXI_ARREADY               (s00_axi_arready                           ),
	  .M_AXI_RID                   (s00_axi_rid                               ),
	  .M_AXI_RDATA                 (s00_axi_rdata                             ),
	  .M_AXI_RRESP                 (s00_axi_rresp                             ),
	  .M_AXI_RLAST                 (s00_axi_rlast                             ),
	  .M_AXI_RUSER                 (s00_axi_ruser                             ),
	  .M_AXI_RVALID                (s00_axi_rvalid                            ),
	  .M_AXI_RREADY                (s00_axi_rready                            ),

	  .MASTER_RST                  (1'b0                                     ),
	  .WR_START                    (wr_burst_req                             ),  //�����д����
	  .WR_ADRS                     ({4'b0,{wr_burst_addr,3'd0}}              ),   //�����25λд��ַ һ��д64�����ݣ�ʵ�����ݵ�ַ*8 
	  .WR_LEN                      ({19'b0,{wr_burst_len,3'd0}}              ),      //�����10λд���ݳ��� ʵ�ʵ����ݳ��� *8
	  .WR_READY                    (wr_burst_ready                             ),
	  .WR_FIFO_RE                  (wr_burst_data_req                        ),      //������������
	  .WR_FIFO_EMPTY               (1'b0                                     ),
	  .WR_FIFO_AEMPTY              (1'b0                                     ),
	  .WR_FIFO_DATA                (wr_burst_data                            ),      //��������� 64λ
	  .WR_DONE                     (wr_burst_finish                          ),      //д���

	  .RD_START                    (rd_burst_req                             ),
	  .RD_ADRS                     ({4'b0,{rd_burst_addr,3'd0}}              ),
	  .RD_LEN                      ({19'b0,{rd_burst_len,3'd0}}              ),
	  .RD_READY                    (                                         ),
	  .RD_FIFO_WE                  (rd_burst_data_valid                      ),
	  .RD_FIFO_FULL                (1'b0                                     ),
	  .RD_FIFO_AFULL               (1'b0                                     ),
	  .RD_FIFO_DATA                (rd_burst_data                            ),
	  .RD_DONE                     (rd_burst_finish                          ),
	  .DEBUG                       (                                         )
);

//assign finish_flag = wr_burst_finish;            //T11
//assign req_flag = wr_burst_req;                  //R11
//assign addr_test = s00_axi_awaddr[27:20];

frame_read_write frame_read_write_m0
(
	.rst                        (~rst_key                   ),
	.mem_clk                    (aclk_1                   ),
//=========================read wires===========================//
	.rd_burst_req               (rd_burst_req             ),
	.rd_burst_len               (rd_burst_len             ),
	.rd_burst_addr              (rd_burst_addr            ),
	.rd_burst_data_valid        (rd_burst_data_valid      ),
	.rd_burst_data              (rd_burst_data            ),
	.rd_burst_finish            (rd_burst_finish          ),
	.read_clk                   (video_clk                ),
	.read_req                   (read_req                 ),
	.read_req_ack               (read_req_ack             ),
	.read_finish                (                         ),

    .read_addr_0                (25'h400000                ), //The first frame address is 0
	.read_addr_1                (25'h440000                ), //The second frame address is 24'd2073600 ,large enough address space for one frame of video
	.read_addr_2                (25'h480000              ),
	.read_addr_3                (25'h4C0000                ),
	.read_addr_index            (read_addr_index          ),
	.read_len                   (25'd32640                    ),//frame size �ֱ���*16/64
	.read_en                    (read_en                  ),
	.read_data                  (read_data                ),
//=========================write wires===========================//
	.wr_burst_req               (wr_burst_req             ),
	.wr_burst_len               (wr_burst_len             ),      //����Ĵ��䳤��  10λ 
	.wr_burst_addr              (wr_burst_addr            ),      //����Ĵ����ַ  25λ
	.wr_burst_data_req          (wr_burst_data_req        ),
	.wr_burst_data              (wr_burst_data      ),
	.wr_burst_finish            (wr_burst_finish          ),
	.write_clk                  (cmos_pclk_g              ),
	.write_req                  (write_req                ),
	.write_req_ack              (write_req_ack            ),
	.write_finish               (                         ),

    .write_addr_0               (25'h400000               ),    //���˵�ַ0x30000000 ����3��
	.write_addr_1               (25'h440000               ),     //���˵�ַ0x30200000
	.write_addr_2               (25'h480000               ),      //���˵�ַ0x30400000
	.write_addr_3               (25'h4C0000               ),      //���˵�ַ0x30600000
	.write_addr_index           (write_addr_index         ),       //����ĵ�ַѡ��
	.write_len                  (25'd32640                  ), //frame size �ֱ���*16/64 =/4
	.write_en                   (write_en                 ),
	.write_data                 (write_data               )
);

    DDR3 u_DDR3 (
      .pll_refclk_in          (ex_clk_50m),     // input
      .top_rst_n              (rst_key),        // input
      .ddrc_rst               (1'b0),           // input
      .csysreq_ddrc           (1'b1),           // input
      .csysack_ddrc           (),               // output
      .cactive_ddrc           (),               // output
      .pll_lock               (ddrc_pll_lock),  // output
      .pll_aclk_0             (aclk_mux),       // output
      .pll_aclk_1             (aclk_1),         // output
      .pll_aclk_2             (),               // output
      .ddrphy_rst_done        (ddrphy_rst_done),// output
      .ddrc_init_done         (ddrc_init_done), // output

      .pad_loop_in            (pad_loop_in),    // input
      .pad_loop_in_h          (pad_loop_in_h),  // input
      .pad_rstn_ch0           (pad_rstn_ch0),   // output
      .pad_ddr_clk_w          (pad_ddr_clk_w),  // output
      .pad_ddr_clkn_w         (pad_ddr_clkn_w), // output
      .pad_csn_ch0            (pad_csn_ch0),    // output
      .pad_addr_ch0           (pad_addr_ch0),   // output [15:0]
      .pad_dq_ch0             (pad_dq_ch0),     // inout [15:0]
      .pad_dqs_ch0            (pad_dqs_ch0),    // inout [1:0]
      .pad_dqsn_ch0           (pad_dqsn_ch0),   // inout [1:0]
      .pad_dm_rdqs_ch0        (pad_dm_rdqs_ch0),// output [1:0]
      .pad_cke_ch0            (pad_cke_ch0),    // output
      .pad_odt_ch0            (pad_odt_ch0),    // output
      .pad_rasn_ch0           (pad_rasn_ch0),   // output
      .pad_casn_ch0           (pad_casn_ch0),   // output
      .pad_wen_ch0            (pad_wen_ch0),    // output
      .pad_ba_ch0             (pad_ba_ch0),     // output [2:0]
      .pad_loop_out           (pad_loop_out),   // output
      .pad_loop_out_h         (pad_loop_out_h), // output
//=========================m1 wires===========================//     
      .areset_0               (1'b0),           // input
      .aclk_0                 (HCLK),           // input
      .awid_0                 (8'h00),          // input [7:0]
      .awaddr_0               (awaddr_mux),     // input [31:0]
      .awlen_0                (awlen_mux),      // input [7:0]
      .awsize_0               (3'b100),         // input [2:0]
      .awburst_0              (2'b01),          // input [1:0]
      .awlock_0               (1'b0),           // input
      .awvalid_0              (awvalid_mux),    // input
      .awready_0              (awready_mux),    // output
      .awurgent_0             (1'b0),           // input
      .awpoison_0             (1'b0),           // input
      .wdata_0                (wdata_mux),      // input [127:0]
      .wstrb_0                (wstrb_mux),      // input [15:0]
      .wlast_0                (wlast_mux),      // input
      .wvalid_0               (wvalid_mux),     // input
      .wready_0               (wready_mux),     // output
      .bid_0                  (),               // output [7:0]
      .bresp_0                (),               // output [1:0]
      .bvalid_0               (),               // output
      .bready_0               (1'b1),           // input

      .arid_0                 (8'h00),          // input [7:0]
      .araddr_0               (araddr_mux),     // input [31:0]
      .arlen_0                (arlen_mux),      // input [7:0]
      .arsize_0               (3'b100),         // input [2:0]
      .arburst_0              (2'b01),          // input [1:0]
      .arlock_0               (1'b0),           // input
      .arvalid_0              (arvalid_mux),    // input
      .arready_0              (arready_mux),    // output
      .arpoison_0             (1'b0),           // input
      .rid_0                  (),               // output [7:0]
      .rdata_0                (rdata_mux),      // output [127:0]
      .rresp_0                (),               // output [1:0]
      .rlast_0                (rlast_mux),      // output
      .rvalid_0               (rvalid_mux),     // output
      .rready_0               (rready_mux),     // input
      .arurgent_0             (1'b0),           // input
      .csysreq_0              (1'b1),           // input
      .csysack_0              (),               // output
      .cactive_0              (),               // output
//=========================frame wires===========================//
      .areset_1               (1'b0),           // input
      .aclk_1                 (aclk_1),         // input
      .awid_1                 (s00_axi_awid),          // input [7:0]
      .awaddr_1               (s00_axi_awaddr),       // input [31:0]
      .awlen_1                (s00_axi_awlen),          // input [7:0]
      .awsize_1               (s00_axi_awsize),         // input [2:0]
      .awburst_1              (s00_axi_awburst),          // input [1:0]
      .awlock_1               (s00_axi_awlock),           // input
      .awvalid_1              (s00_axi_awvalid),      // input
      .awready_1              (s00_axi_awready),      // output
      .awurgent_1             (1'b0),           // input
      .awpoison_1             (1'b0),           // input
      .wdata_1                (s00_axi_wdata),        // input [63:0]
      .wstrb_1                (s00_axi_wstrb),          // input [7:0]
      .wlast_1                (s00_axi_wlast),        // input
      .wvalid_1               (s00_axi_wvalid),       // input
      .wready_1               (s00_axi_wready),       // output
      .bid_1                  (s00_axi_bid),               // output [7:0]
      .bresp_1                (s00_axi_bresp),               // output [1:0]
      .bvalid_1               (s00_axi_bvalid),               // output
      .bready_1               (s00_axi_bready),           // input

      .arid_1                 (s00_axi_arid),          // input [7:0]
      .araddr_1               (s00_axi_araddr),       // input [31:0]
      .arlen_1                (s00_axi_arlen),          // input [7:0]
      .arsize_1               (s00_axi_arsize),         // input [2:0]
      .arburst_1              (s00_axi_arburst),          // input [1:0]
      .arlock_1               (s00_axi_arlock),           // input
      .arvalid_1              (s00_axi_arvalid),      // input
      .arready_1              (s00_axi_arready),      // output
      .arpoison_1             (1'b0),           // input
      .rid_1                  (s00_axi_rid),               // output [7:0]
      .rdata_1                (s00_axi_rdata),        // output [63:0]
      .rresp_1                (s00_axi_rresp),               // output [1:0]
      .rlast_1                (s00_axi_rlast),        // output
      .rvalid_1               (s00_axi_rvalid),       // output
      .rready_1               (s00_axi_rready),       // input
      .arurgent_1             (1'b0),           // input
      .csysreq_1              (1'b1),           // input
      .csysack_1              (),               // output
      .cactive_1              ()                // output
    );

 

//TSMAC-------------------------------------------------------------------------------------------------  
    assign {phy_txd3,phy_txd2,phy_txd1,phy_txd0} = phy_txd;
    assign phy_rxd = {phy_rxd3,phy_rxd2,phy_rxd1,phy_rxd0};
    //assign phy_rst_n = rst_key;

    assign tsmac_phy_mdi  = tsmac_phy_mdio;
    assign tsmac_phy_mdio = tsmac_phy_mdoen ? tsmac_phy_mdo : 1'bz;

    assign tsmac_fifo_wr_data = {10{tsmac_rvalid}} & {tsmac_rlast,tsmac_rvalid,tsmac_rdata};

    //tx_clk
    wire pado;
    wire padt;
    
    GTP_OSERDES #(
     .OSERDES_MODE            ("ODDR"),         //"ODDR","OMDDR","OGSER4","OMSER4","OGSER7","OGSER8",OMSER8"
     .WL_EXTEND               ("FALSE"),        //"TRUE"; "FALSE"
     .GRS_EN                  ("TRUE"),         //"TRUE"; "FALSE"
     .LRS_EN                  ("TRUE"),         //"TRUE"; "FALSE"
     .TSDDR_INIT              (1'b0)            //1'b0;1'b1
    ) u_GTP_OGDDR(
       .DO                    (pado),
       .TQ                    (padt),
       .DI                    (8'b00000001),
       .TI                    (4'd0),
       .RCLK                  (HCLK),
       .SERCLK                (HCLK),
       .OCLK                  (1'd0),
       .RST                   (~SYSRESETn)
    ); 
    
    GTP_OUTBUFT u_GTP_OUTBUFT
    (
        .O                    (l0_sgmii_clk_shft),
        .I                    (pado),
        .T                    (padt)
    );
    
    //rx_clk
    wire rx_clki_shft_bufg;   
    GTP_IOCLKDELAY
    #(
        .DELAY_STEP_VALUE     (8'd70), 
        .SIM_DEVICE           ("LOGOS"),
        .DELAY_STEP_SEL       ("PARAMETER") //"PARAMETER"/ "PORT"    ,pgh "PARAMETER"-->1'b1-->delay_step_value    pgl  "PARAMETER"-->1'b0-->delay_step_value
    )u_GTP_IOCLKDELAY(        
        .CLKOUT               (rx_clki_shft_bufg),
        .DELAY_OB             (),
        .CLKIN                (rx_clki),
        .DELAY_STEP           (),
        .DIRECTION            (1'b1),
        .LOAD                 (1'b1),
        .MOVE                 (1'b0)
    );      

    wire rx_clki_clkbufg;
    GTP_CLKBUFG u_GTP_CLKBUFG
    (
        .CLKOUT               (rx_clki_clkbufg),
        .CLKIN                (rx_clki_shft_bufg)
    );

    TSMAC_FIFO_RXCKLI u_TSMAC_FIFO_RXCKLI (
      .wr_clk                 (rx_clki_clkbufg),       // input
      .wr_rst                 (~SYSRESETn),            // input
      .wr_en                  (1'b1),                  // input
      .wr_data                (tsmac_fifo_wr_data),    // input [9:0]
      .wr_full                (),                      // output
      .almost_full            (),                      // output
      .rd_clk                 (HCLK),                  // input
      .rd_rst                 (~SYSRESETn),            // input
      .rd_en                  (1'b1),                  // input
      .rd_data                (ethernet_fifo_rd_data), // output [9:0]
      .rd_empty               (),      		           // output
      .almost_empty           ()                       // output
    );

    wire PSEL_temp;
    wire PENABLE_temp;
    assign PSEL_temp    = PSEL && (PADDR < 12'h300);
    assign PENABLE_temp = PENABLE && (PADDR < 12'h300);
    
    assign tsmac_tdata  = udp_cs ? udp_tdata  : m1_tdata;
    assign tsmac_tstart = udp_cs ? udp_tstart : m1_tstart;
    assign tsmac_tlast  = udp_cs ? udp_tlast  : m1_tlast;

    //tsmac
    tsmac_phy   u_tsmac_phy(
        .tx_clki              (HCLK),
        .rx_clki              (rx_clki_clkbufg),
        .tx_rst               (~SYSRESETn),
        .rx_rst               (~SYSRESETn),
                              
        .mdi                  (tsmac_phy_mdi),
        .mdc                  (tsmac_phy_mdc),
        .tsmac_tprt           (),
        .tsmac_tpar           (),
        .mdo                  (tsmac_phy_mdo),
        .mdoen                (tsmac_phy_mdoen),
                              
        .tsmac_tcrq           (1'b0),
        .tsmac_cfpt           (16'h0000),
        .tsmac_thdf           (1'b0),
        .tsmac_txcf           (),
        .tsmac_tcdr           (),
                              
        .rx_dv                (phy_rx_dv),
        .rxd                  (phy_rxd  ),
        .tx_en                (phy_tx_en),
        .txd                  (phy_txd  ),
                              
        .presetn              (~SYSRESETn),
        .pclk                 (HCLK),
        .pselx                (PSEL_temp),
        .pwrite               (PWRITE),
        .penable              (PENABLE_temp),
        .paddr                (PADDR[11:4]),
        .pwdata               (PWDATA),
        .prdata               (),  

        .tsmac_tsvp           (),
        .tsmac_tsv            (),
        .tsmac_rsvp           (),
        .tsmac_rsv            (),
                              
        .tsmac_tpnd           (tsmac_tpnd  ),
        .tsmac_tdata          (tsmac_tdata ),
        .tsmac_tstart         (tsmac_tstart),
        .tsmac_tlast          (tsmac_tlast ),
                              
        .tsmac_rdata          (tsmac_rdata),
        .tsmac_rvalid         (tsmac_rvalid),
        .tsmac_rlast          (tsmac_rlast),
                              
        .speed                ()
    );

endmodule

