// Generated by IP Generator (Version 2016.4B2 build 17944)



//************************************************************************
//Copyright (c) 2015, PANGO MICROSYSTEMS,INC
//All Rights Reserved
//************************************************************************
`timescale 1ns / 1ns

// ***********************************************************************
// --- Module Definition
module tsmac_phy               
(
// --- Clock Inputs
input         tx_clki,
input         rx_clki,
input         tx_rst,
input         rx_rst,

// --- MII Management Input
input         mdi,
output        mdc,     
output        mdo,     
output        mdoen,
// --- System Transmit Interface Inputs
input   [7:0] tsmac_tdata,
input         tsmac_tstart,    
input         tsmac_tlast,
// --- Transmit Flow Control Inputs
input         tsmac_tcrq,
input  [15:0] tsmac_cfpt,

input         tsmac_thdf,
// --- MII Receive Inputs
input         rx_dv,

input   [3:0] rxd,            //@IPC show RGMII_ENABLE

// --- System Transmit Interface Inputs ( Per-packet Control )
// --- APB Interface
input         presetn,  
input         pclk,  
input         pselx,  
input         pwrite,  
input         penable,   
input   [7:0] paddr,
input  [31:0] pwdata,
output [31:0] prdata,
// --- MII Transmit Outputs
output        tx_en,

output  [3:0] txd,            //@IPC show RGMII_ENABLE 
       
// --- System Transmit Interface Outputs
output        tsmac_tpnd,    
output        tsmac_tprt,    
output        tsmac_tpar,

output        tsmac_tsvp,     //@IPC show STATISTICS_ENABLE  
output [51:0] tsmac_tsv,      //@IPC show STATISTICS_ENABLE 

output        tsmac_txcf,    
output        tsmac_tcdr,
// --- System Receive Interface Outputs
output  [7:0] tsmac_rdata,
output        tsmac_rvalid,  
output        tsmac_rlast,

output        tsmac_rsvp,         //@IPC show STATISTICS_ENABLE 
output [32:0] tsmac_rsv,          //@IPC show STATISTICS_ENABLE 

// --- Host Interface Outputs
output  [7:0] rxd_gm,
output        speed

);



localparam WR_ADDR_WIDTH = 11 ; // @IPC int 11,16

localparam SPEED_TYPE = "10/100/1000M_MAC" ; // @IPC enum 10/100/1000M_MAC,1000M_MAC,10/100M_MAC

localparam INTERFACE = "RGMII" ; // @IPC enum MII/GMII,RGMII

localparam SERDES_MODE = "GE" ; // @IPC enum GE,SGMII

localparam SERDES_MODE_EN = 0 ; // @IPC bool


// ------------------------------------------------------------------------

wire[7:0]    rxd_gm1      ;
wire         rx_dv_gm1    ;
wire         rx_er_gm1    ;
wire         crs_gm1      ;
wire         col_gm1      ;

wire[7:0]    txd_gm       ;
wire         tx_en_gm     ;
wire         tx_er_gm     ;
wire[3:0]    txd_rgm      ;
wire         tx_ctr       ;
wire         rst_n        ;

//---------------------------------------------------
assign  rxd_gm    = (INTERFACE == "MII/GMII") ?  rxd   : rxd_gm1;
assign  rx_dv_gm  = (INTERFACE == "MII/GMII") ?  rx_dv : rx_dv_gm1;

assign  rx_er_gm  = (INTERFACE == "MII/GMII") ?  1'b0 : rx_er_gm1;       //@IPC show RGMII_ENABLE


assign  txd       = (INTERFACE == "MII/GMII") ?  txd_gm  : txd_rgm  ; 
assign  tx_en     = (INTERFACE == "MII/GMII") ?  tx_en_gm: tx_ctr   ;

assign  rst_n     = ~presetn;
//----------------------------------------------------------

pgs_tsmac_rgmii_gmii_convert_v1_0 U_pgs_tsmac_rgmii_gmii_convert_v1_0               //@IPC show RGMII_ENABLE
(                                                                         //@IPC show RGMII_ENABLE 
 .tx_rst     (~tx_rst    ),                                               //@IPC show RGMII_ENABLE 
 .rx_rst     (~rx_rst    ),                                               //@IPC show RGMII_ENABLE 
 .tx_clk     (tx_clki    ),                                               //@IPC show RGMII_ENABLE 
 .tx_en_gm   (tx_en_gm   ),                                               //@IPC show RGMII_ENABLE 
 .txd_gm     (txd_gm     ),                                               //@IPC show RGMII_ENABLE 
 .tx_er_gm   (tx_er_gm   ),                                               //@IPC show RGMII_ENABLE 
 
 .rx_clk     (rx_clki    ),                                               //@IPC show RGMII_ENABLE 
 .rx_ctr     (rx_dv      ),                                               //@IPC show RGMII_ENABLE 
 .col_rgm    (1'b0       ),                                               //@IPC show CRS_ENABLE 
 .crs_rgm    (1'b0       ),                                               //@IPC show CRS_ENABLE 
 .rxd_rgm    (rxd        ),                                               //@IPC show RGMII_ENABLE 
 
 .txd_rgm    (txd_rgm    ),                                               //@IPC show RGMII_ENABLE 
 .tx_ctr     (tx_ctr     ),                                               //@IPC show RGMII_ENABLE
 .col_gm     (col_gm1    ),                                               //@IPC show RGMII_ENABLE 
 .crs_gm     (crs_gm1    ),                                               //@IPC show RGMII_ENABLE
 .rx_dv_gm   (rx_dv_gm1  ),                                               //@IPC show RGMII_ENABLE
 .rx_er_gm   (rx_er_gm1  ),                                               //@IPC show RGMII_ENABLE
 .rxd_gm     (rxd_gm1    )                                                //@IPC show RGMII_ENABLE                                                                                                                                                   
);                                                                        //@IPC show RGMII_ENABLE 

pgs_tsmac_core_v1_1  #(
       .WR_ADDR_WIDTH   ( WR_ADDR_WIDTH   ),
       .SPEED_TYPE      ( SPEED_TYPE      )
       
)U_pgs_tsmac_core_v1_1
(
  .tx_clki(tx_clki),
  .rx_clki(rx_clki),
  .tx_rst (tx_rst),                                               //@IPC show RGMII_ENABLE 
  .rx_rst (rx_rst),                                               //@IPC show RGMII_ENABLE 

  .txceni(1'b1) ,        //@IPC show SERDES_MODE_EN
  .rxceni(1'b1) ,        //@IPC show SERDES_MODE_EN

	.crs(1'b0),            //@IPC show CRS_ENABLE
	.col(1'b0),            //@IPC show CRS_ENABLE

  .mdi(mdi),
  .tdata(tsmac_tdata), .tstart(tsmac_tstart),  .tlast(tsmac_tlast),  
  .tcrq(tsmac_tcrq),   .cfpt(tsmac_cfpt),   
  .thdf(tsmac_thdf),
  .rx_dv(rx_dv_gm),  .rxd(rxd_gm),   .rx_er(rx_er_gm),
  .presetn(presetn), .pclk(pclk),    .pselx(pselx),     .pwrite(pwrite), .penable(penable), 
  .paddr(paddr),     .pwdata(pwdata),.prdata(prdata),	 
  
  .tx_en(tx_en_gm),  .txd(txd_gm),     .tx_er(tx_er_gm),
  .mdc(mdc),         .mdo(mdo),        .mdoen(mdoen),
  .tpnd(tsmac_tpnd), .tprt(tsmac_tprt),.tpar(tsmac_tpar),

  .tsvp(tsmac_tsvp),      //@IPC show STATISTICS_ENABLE 
  .tsv(tsmac_tsv),          //@IPC show STATISTICS_ENABLE  

  .txcf(tsmac_txcf), 
  .tcdr(tsmac_tcdr), 
  .rdata(tsmac_rdata), 
  .rvalid(tsmac_rvalid),   
  .rlast(tsmac_rlast), 

  .rsvp(tsmac_rsvp),     //@IPC show STATISTICS_ENABLE 
  .rsv(tsmac_rsv),       //@IPC show STATISTICS_ENABLE 

  .speed(speed),
  .packet_cnt(packet_cnt),
  .crc_err_cnt(crc_err_cnt)
);
endmodule

