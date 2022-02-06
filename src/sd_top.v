/***************************************************/
/**module:SD read & write
/**version: v1.0
/**date:2021/4/21
/***************************************************/
module sd_top
(

input wire                        HCLK,                
input wire                        HRESETn,
input wire                        HSEL,            
input wire [1:0]                  HTRANS,
input wire [2:0]                  HSIZE,
input wire [2:0]	                HBURST,
input wire [3:0]	                HPROT,
input wire                        HWRITE,
input wire [31:0]                 HADDR,
input wire [31:0]                 HWDATA,    
input wire                        HREADY,
      
output wire                       HREADYOUT,
output wire                       HRESP,
output reg [31:0]                 HRDATA,
         
 
output reg                        WR_FINISH,
output                            SD_nCS,               
output                            SD_DCLK,                  
output                            SD_MOSI,                 
input                             SD_MISO     
   );

localparam S_IDLE               = 0;
localparam S_CMD0               = 1;
localparam S_CMD8               = 2;
localparam S_CMD55              = 3;
localparam S_CMD41              = 4;
localparam S_CMD17              = 5;
localparam S_READ_PRE 		  = 6;
localparam S_READ               = 7;
localparam S_CMD24              = 8;
localparam S_WRITE_PRE		  = 9;
localparam S_WRITE              = 10;
localparam S_ERR                = 11;
localparam S_WRITE_END          = 12;
localparam S_READ_END           = 13;
localparam S_WAIT_READ_WRITE    = 14;

localparam ADDR_DAT             = 32'h80000000;
localparam ADDR_ADD             = 32'h80000004;
localparam ADDR_STA             = 32'h80000008;
localparam ADDR_CTR             = 32'h8000000c;
localparam ADDR_DAT_O           = 32'h80000010; 
       
reg [31:0] DATA;			//数据输入寄存器
reg [31:0] ADDR;			//读写地址寄存器
reg [31:0] STATE;			//状态寄存器
reg [31:0] CTRL;			//控制寄存器  

reg 	  			        hsel_q0,hwrite;
reg [31:0]  		        reg_sel;
reg [1:0] 			        htrans;

reg                             cmd_req;                  
wire                            cmd_req_ack;             
wire                            cmd_req_error;           
reg[47:0]                       cmd;                      
reg[7:0]                        cmd_r1;                   
reg[15:0]                       cmd_data_len;             
      
reg                             block_read_req;           
wire                            block_read_req_ack;      
reg                             block_write_req;          
wire                            block_write_req_ack;     
            
wire                            nCS_ctrl;                
wire                            spi_wr_req;              
wire                            spi_wr_ack;              
wire[7:0]                       spi_data_in;             
wire[7:0]                       spi_data_out;            
wire[15:0]                      clk_div;		
  
reg [7:0]                       send_data;  
reg [31:0]                      dat_buf;
reg [3:0]                       buf_cnt;
reg [6:0]                       w_cnt;     
reg [6:0]                       r_cnt; 
reg [6:0]                       W_CNT;
reg [6:0]                       R_CNT;
			      
reg [4:0]         	        state;
reg [31:0] 		              BUF_W[0:127];
reg [31:0]		              BUF_R[0:127];

 /***********************************************************************************/
/*******************************AHB相关*********************************************/
/***********************************************************************************/
assign  			        HREADYOUT = 1'b1;
assign 			        HRESP     = 1'b0;


always @(posedge HCLK or negedge HRESETn) begin			//控制信号打拍
if(!HRESETn)begin
	hsel_q0 	            <= 1'h0;
	hwrite 		      <= 1'h0;
      reg_sel                 <= 32'h0;
      htrans                  <= 2'h0;
end
else begin
	hsel_q0 	            <= HSEL;
	hwrite 		      <= HWRITE;
      reg_sel                 <= HADDR;
      htrans                  <= HTRANS;
end
end

always @(posedge HCLK or negedge HRESETn) begin				//写寄存器
if(!HRESETn)begin
	DATA 	                  <= 32'h0;
	ADDR 	                  <= 32'h0;
	CTRL	                  <= 32'h0;
      W_CNT                   <= 7'h0;

	end
else if(  hwrite && hsel_q0  && htrans[1] ) begin
	case(reg_sel)
		(ADDR_DAT): begin                        //写数据寄存器时先写到BUF里
			BUF_W[W_CNT]        <= HWDATA;       
			W_CNT	 		  <= W_CNT+1'b1;
		end
		(ADDR_ADD):	ADDR[31:0]    <= HWDATA[31:0]; 
		(ADDR_CTR):	CTRL[31:0]    <= HWDATA[31:0];
	endcase
	end
else if(state == S_CMD24) begin
	CTRL[0] 	                     <= 1'b0;                         //写控制位清0
     
      end
else if(state == S_CMD17) begin
	CTRL[1] 	                     <= 1'b0;                        //读控制位清0
	
      end
end

always @(posedge HCLK or negedge HRESETn) begin                  //读寄存器
      if(!HRESETn) begin
      //      hrdata                            <= 32'h0;
             HRDATA                       <= 32'h0;
		R_CNT 	                      <= 7'h0;
      end
      else if( !HWRITE && HSEL  && HTRANS[1])begin
      	case(HADDR)
      		(ADDR_DAT):	HRDATA[31:0]    <= DATA[31:0];
      		(ADDR_ADD):	HRDATA[31:0]    <= ADDR[31:0];
      		(ADDR_STA):	HRDATA[31:0]    <= STATE[31:0];
      		(ADDR_CTR):	HRDATA[31:0]    <= CTRL[31:0];
      		(ADDR_DAT_O): begin
				  	HRDATA[31:0]    <= BUF_R[R_CNT];           //读数据时从BUF里读
				  	R_CNT		    <= R_CNT +1'b1; 
			            end
      	endcase
      end
//      else 
//	  	HRDATA                             <= HRDATA_udp;

end
/***********************************************************************************/
/*******************************SD卡相关*********************************************/
/***********************************************************************************/


always @(posedge HCLK or negedge HRESETn)					//读写状态机
begin
	if(!HRESETn)
	begin
		STATE 	                    <= 32'h0;

		state                           <= S_IDLE;
		cmd_req                         <= 1'b0;
		cmd_data_len                    <= 16'd0;
		cmd_r1                          <= 8'd0;
		cmd                             <= 48'd0;

		block_write_req                 <= 1'b0;
		block_read_req                  <= 1'b0;
            dat_buf                         <= 32'h0;
            buf_cnt                         <= 4'h0;

            w_cnt                           <= 7'h0;
		r_cnt                           <= 7'h0;
            WR_FINISH                       <= 1'b1;
	end
	else
	      case(state)
		      S_IDLE:
			begin
                        state               <= S_CMD0;
			end
			S_CMD0:                        //初始化指令
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state         <= S_CMD8;
					cmd_req       <= 1'b0;
				end
				else
				begin
					cmd_req       <= 1'b1;
					cmd_data_len  <= 16'd0;
					cmd_r1        <= 8'h01;
					cmd           <= {8'd0,8'h00,8'h00,8'h00,8'h00,8'h95};
                              
				end
			end
			S_CMD8:                        //初始化指令
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state         <= S_CMD55;
					cmd_req       <= 1'b0;
                              //STATE[12:8] <= S_CMD55;
				end
				else
				begin
					cmd_req       <= 1'b1;
					cmd_data_len  <= 16'd4;
					cmd_r1        <= 8'h01;
					cmd           <= {8'd8,8'h00,8'h00,8'h01,8'haa,8'h87};
				end
			end
			S_CMD55:                        //初始化指令
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state         <= S_CMD41;
					cmd_req       <= 1'b0;
                             // STATE[12:8] <= S_CMD41;
				end
				else
				begin
					cmd_req       <= 1'b1;
					cmd_data_len  <= 16'd0;
					cmd_r1        <= 8'h01;
					cmd           <= {8'd55,8'h00,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD41:                        //初始化指令
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state          <= S_WAIT_READ_WRITE;
					cmd_req        <= 1'b0;
                                 STATE[3]       <= 1'b1;
                              //STATE[12:8]  <= S_WAIT_READ_WRITE;
				end
				else if(cmd_req_ack )
				begin
					state           <= S_CMD55;
                              //STATE[12:8]   <= S_CMD41;
				end
				else
				begin
					cmd_req           <= 1'b1;
					cmd_data_len      <= 16'd0;
					cmd_r1            <= 8'h00;
					cmd               <= {8'd41,8'h40,8'h00,8'h00,8'h00,8'hff};        
				end
			end		
			S_WAIT_READ_WRITE:                        //等待读写指令
			begin
				if(CTRL[0] ==  1'b1)
				begin
					state             <= S_CMD24;
                              STATE[0]          <= 1'b0;
                             // STATE[12:8]     <= S_CMD24;
				end
				else if(CTRL[1] == 1'b1)
				begin
					state             <= S_CMD17;
                              STATE[1]          <= 1'b0;
                              //STATE[12:8]     <= S_CMD17;
				end

			end
			S_CMD24:                        //发送写指令
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state             <= S_WRITE;
					cmd_req           <= 1'b0;
					buf_cnt           <= 0;
                              dat_buf           <=BUF_W[0];
                              w_cnt             <= 7'h1;
                              //STATE[12:8]     <= S_WRITE;
				end
				else
				begin
					cmd_req           <= 1'b1;
					cmd_data_len      <= 16'd0;
					cmd_r1            <= 8'h00;
					cmd               <= {8'd24,ADDR,8'hff};
				end
			end
			S_WRITE:                        //进入写状态
			begin
				if(block_write_req_ack == 1'b1)
				begin
					block_write_req    <= 1'b0;
					state              <= S_WRITE_END;
                              //STATE[12:8]      <= S_WRITE_END;			
                              WR_FINISH          <= 1'b0;
                              //w_cnt             <= 0;
				end
				else if ( ( buf_cnt==4'h4)) begin 
					buf_cnt <= 0;
                              dat_buf            <= BUF_W[w_cnt];
                              w_cnt              <= w_cnt+1'b1;
                         //      STATE[22:16]    <= r_cnt;
                         
				end
				else if (  spi_wr_ack) begin
					send_data		<= dat_buf[31:24];
					dat_buf		<= {dat_buf[23:0],8'h0}; //数据拆分
					buf_cnt 	      <= buf_cnt + 1'b1;
				end
				else
					block_write_req   <= 1'b1;
			end
			S_CMD17:                        //发送读指令
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state             <= S_READ_PRE;
					cmd_req           <= 1'b0;
                              //STATE[12:8]     <= S_READ_PRE;
				end
				else
				begin
					cmd_req           <= 1'b1;
					cmd_data_len      <= 16'd0;
					cmd_r1            <= 8'h00;
					cmd               <= {8'd17,ADDR,8'hff};
				end
			end
			S_READ_PRE:                        //准备进入读状态，等从机的0xfe
			begin
			  if(spi_wr_ack && spi_data_out==8'hfe) begin
				state                   <= S_READ;
				buf_cnt                 <= 0;
                        //STATE[12:8]           <= S_READ;
			  end
			  else begin
				  block_read_req        <= 1'b1;
			  end
			end
			S_READ:                        //进入读状态
			begin
				if(block_read_req_ack)
				begin
					state             <= S_READ_END;
					block_read_req    <= 1'b0;
                              //STATE[12:8]     <= S_READ_END;
					WR_FINISH 	      <= 1'b0;
				end
                else if((buf_cnt==4'h4)) begin 
					buf_cnt	      <= 4'h0;
                              BUF_R[r_cnt]	<= dat_buf;
					r_cnt			<= r_cnt+1'b1;
                        //      STATE[22:16]    <= r_cnt;
				end
                else if ( spi_wr_ack) begin
					dat_buf	      <= {spi_data_out,dat_buf[31:8]}; //数据拼接为32位
					buf_cnt           <= buf_cnt +1'b1;
				end				
				else
				begin
					block_read_req    <= 1'b1;
				end
			end
			S_WRITE_END:                        //写完成
			begin
				state                   <= S_WAIT_READ_WRITE;
                        STATE[0]                <= 1'b1;
                    //  STATE[12:8]             <= S_WAIT_READ_WRITE;
                      WR_FINISH                 <= 1'b1;
			end
			S_READ_END:                        //读完成
			begin
				state                   <= S_WAIT_READ_WRITE;
                        STATE[1]                <= 1'b1;
                         //STATE[12:8]          <= S_WAIT_READ_WRITE;    
				WR_FINISH 	            <= 1'b1;    
			end
			default:
				state                   <= S_IDLE;
		endcase
end



sd_card_cmd u_sd_card_cmd(
	.sys_clk                        (HCLK                   ),
	.rst                            (HRESETn                ),
	.spi_clk_div                    (CTRL[31:16]            ),
	.cmd_req                        (cmd_req                ),		//in
	.cmd_req_ack                    (cmd_req_ack            ),
	.cmd_req_error                  (cmd_req_error          ),
	.cmd                            (cmd                    ),		//in
	.cmd_r1                         (cmd_r1                 ),		//in
	.cmd_data_len                   (cmd_data_len           ),		//in


	.block_read_req                 (block_read_req         ),
	.block_read_req_ack             (block_read_req_ack     ),

	.block_write_req                (block_write_req        ),		//in
	.block_write_data               (send_data              ),		//in
	.block_write_req_ack            (block_write_req_ack    ),
	/*********************************************************/
	.nCS_ctrl                       (nCS_ctrl               ),
	.clk_div                        (clk_div                ),
	.spi_wr_req                     (spi_wr_req             ),
	.spi_wr_ack                     (spi_wr_ack             ),
	.spi_data_in                    (spi_data_in            ),
	.spi_data_out                   (spi_data_out           )		

);

spi_master u_spi_master(
	.sys_clk                        (HCLK                   ),
	.rst                            (HRESETn                ),
	.nCS                            (SD_nCS                 ),
	.DCLK                           (SD_DCLK                ),
	.MOSI                           (SD_MOSI                ),
	.MISO                           (SD_MISO                ),
	.clk_div                        (clk_div             	  ),
	.CPOL                           (1'b1                   ),
	.CPHA                           (1'b1                   ),
	.nCS_ctrl                       (nCS_ctrl               ),
	.wr_req                         (spi_wr_req             ),
	.wr_ack                         (spi_wr_ack             ),
	.data_in                        (spi_data_in            ),
	.data_out                       (spi_data_out           )
);

endmodule

