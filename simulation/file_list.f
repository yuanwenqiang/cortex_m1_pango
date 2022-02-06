  ../bench/m1_test.v

  ../bench/src/M1_soc_top.v
  ../bench/src/rst_gen.v
  ../bench/src/axi1_wr_test.v
  
  ../bench/src/m1_core/cm1_adder.vp
  ../bench/src/m1_core/cm1_ahb.vp
  ../bench/src/m1_core/cm1_alu_dec.vp
  ../bench/src/m1_core/cm1_core.vp
  ../bench/src/m1_core/cm1_ctl.vp
  ../bench/src/m1_core/cm1_ctl_add3.vp
  ../bench/src/m1_core/cm1_dbg_ahb_dec.vp
  ../bench/src/m1_core/cm1_dbg_ahb_mux.vp
  ../bench/src/m1_core/cm1_dbg_ahb_ppb.vp
  ../bench/src/m1_core/cm1_dbg_bp.vp
  ../bench/src/m1_core/cm1_dbg_ctl.vp
  ../bench/src/m1_core/cm1_dbg_dw.vp
  ../bench/src/m1_core/cm1_dbg_mtx.vp
  ../bench/src/m1_core/cm1_dbg_mtx_dbg.vp
  ../bench/src/m1_core/cm1_dbg_mtx_sys.vp
  ../bench/src/m1_core/cm1_dbg_rom_tb.vp
  ../bench/src/m1_core/cm1_dbg_sys.vp
  ../bench/src/m1_core/cm1_dbg_tcm.vp
  ../bench/src/m1_core/cm1_decoder.vp
  ../bench/src/m1_core/cm1_dp.vp
  ../bench/src/m1_core/cm1_excpt.vp
  ../bench/src/m1_core/cm1_fetch.vp
  ../bench/src/m1_core/cm1_mem_ctl.vp
  ../bench/src/m1_core/cm1_multiplier.vp
  ../bench/src/m1_core/cm1_multiply_shift.vp
  ../bench/src/m1_core/cm1_mux4.vp
  ../bench/src/m1_core/cm1_nvic.vp
  ../bench/src/m1_core/cm1_nvic_ahb.vp
  ../bench/src/m1_core/cm1_nvic_ahb_os.vp
  ../bench/src/m1_core/cm1_nvic_main.vp
  ../bench/src/m1_core/cm1_nvic_pri_lvl.vp
  ../bench/src/m1_core/cm1_nvic_pri_num.vp
  ../bench/src/m1_core/cm1_nvic_tree.vp
  ../bench/src/m1_core/cm1_reg_bank.vp
  ../bench/src/m1_core/cm1_shifter.vp
  ../bench/src/m1_core/cm1_undef_check.vp
  ../bench/src/m1_core/CortexM1.vp
  ../bench/src/m1_core/CortexM1Dbg.vp
  ../bench/src/m1_core/CortexM1DbgIntegration.vp
  ../bench/src/m1_core/CortexM1Integration.vp
  ../bench/src/m1_core/DAPAHBAP.vp
  ../bench/src/m1_core/DAPAhbApAhbSync.vp
  ../bench/src/m1_core/DAPAhbApDapSync.vp
  ../bench/src/m1_core/DAPAhbApMst.vp
  ../bench/src/m1_core/DAPAhbApSlv.vp
  ../bench/src/m1_core/DAPAhbApSyn.vp
  ../bench/src/m1_core/DAPDecMux.vp
  ../bench/src/m1_core/DAPDpApbSync.vp
  ../bench/src/m1_core/DAPDpEnSync.vp
  ../bench/src/m1_core/DAPDpIMux.vp
  ../bench/src/m1_core/DAPDpSync.vp
  ../bench/src/m1_core/DAPJtagDpProtocol.vp
  ../bench/src/m1_core/DAPSwDpApbIf.vp
  ../bench/src/m1_core/DAPSwDpProtocol.vp
  ../bench/src/m1_core/DAPSwDpSync.vp
  ../bench/src/m1_core/DAPSWJDP.vp
  ../bench/src/m1_core/DAPSwjWatcher.vp
  ../bench/src/m1_core/DTCM.vp
  ../bench/src/m1_core/DTCMDBG.vp
  ../bench/src/m1_core/integration_kit_dbg.vp
  ../bench/src/m1_core/ITCM.vp
  ../bench/src/m1_core/ITCMDBG.vp
  
  ../bench/src/ahb/ahb_decoder.vp
  ../bench/src/ahb/ahb_def_slave.vp
  ../bench/src/ahb/ahb_mux.vp

  ../bench/src/logical/cmsdk_ahb_dcache/verilog/cmsdk_ahb_dcache.vp
  ../bench/src/logical/cmsdk_ahb_dcache/verilog/D_Cache.vp
  
  ../bench/src/logical/cmsdk_ahb_ethernet_dmac/verilog/cmsdk_ahb_ethernet_dmac.vp
  ../bench/src/logical/cmsdk_ahb_ethernet_dmac/verilog/Ethernet_DMAC.vp  

  ../bench/src/logical/cmsdk_ahb_gpio/verilog/cmsdk_ahb_gpio.vp
  ../bench/src/logical/cmsdk_ahb_gpio/verilog/cmsdk_ahb_to_iop.vp
  
  ../bench/src/logical/cmsdk_ahb_icache/verilog/cmsdk_ahb_icache.vp
  ../bench/src/logical/cmsdk_ahb_icache/verilog/I_Cache.vp
  
  ../bench/src/logical/cmsdk_ahb_to_apb/verilog/cmsdk_ahb_to_apb.vp
  
  ../bench/src/logical/cmsdk_apb_i2c/verilog/cmsdk_apb_i2c.vp  
  ../bench/src/logical/cmsdk_apb_i2c/verilog/i2c_master_bit_ctrl.vp
  ../bench/src/logical/cmsdk_apb_i2c/verilog/i2c_master_byte_ctrl.vp
  
  ../bench/src/logical/cmsdk_apb_mem/verilog/cmsdk_ahb_mem.v
  
  ../bench/src/logical/cmsdk_apb_slave_mux/verilog/cmsdk_apb_slave_mux.vp

  ../bench/src/logical/cmsdk_apb_spi/verilog/cmsdk_apb_spi.vp

  ../bench/src/logical/cmsdk_apb_subsystem/verilog/cmsdk_apb_subsystem.vp
  
  ../bench/src/logical/cmsdk_apb_timer/verilog/cmsdk_apb_timer.vp
  
  ../bench/src/logical/cmsdk_apb_uart/verilog/cmsdk_apb_uart.vp
  
  ../bench/src/logical/cmsdk_apb_watchdog/verilog/cmsdk_apb_watchdog.vp
  ../bench/src/logical/cmsdk_apb_watchdog/verilog/cmsdk_apb_watchdog_frc.vp
  
  ../bench/src/logical/cmsdk_iop_gpio/verilog/cmsdk_iop_gpio.vp

  ../pnr/ipcore/TEST_RAM/rtl/ipml_sdpram_v1_5_TEST_RAM.v  
  ../pnr/ipcore/TEST_RAM/TEST_RAM.v  
  
  ../pnr/ipcore/DCACHE_SRAM0/rtl/ipml_sdpram_v1_5_DCACHE_SRAM0.v
  ../pnr/ipcore/DCACHE_SRAM0/DCACHE_SRAM0.v        
  ../pnr/ipcore/DCACHE_SRAM1/rtl/ipml_sdpram_v1_5_DCACHE_SRAM1.v
  ../pnr/ipcore/DCACHE_SRAM1/DCACHE_SRAM1.v         
  ../pnr/ipcore/DCACHE_SRAM2/rtl/ipml_sdpram_v1_5_DCACHE_SRAM2.v
  ../pnr/ipcore/DCACHE_SRAM2/DCACHE_SRAM2.v         
  ../pnr/ipcore/DCACHE_SRAM3/rtl/ipml_sdpram_v1_5_DCACHE_SRAM3.v
  ../pnr/ipcore/DCACHE_SRAM3/DCACHE_SRAM3.v         
  ../pnr/ipcore/DCACHE_TAG/rtl/ipml_sdpram_v1_5_DCACHE_TAG.v
  ../pnr/ipcore/DCACHE_TAG/DCACHE_TAG.v             
  ../pnr/ipcore/ICACHE_INS0/rtl/ipml_sdpram_v1_5_ICACHE_INS0.v
  ../pnr/ipcore/ICACHE_INS0/ICACHE_INS0.v           
  ../pnr/ipcore/ICACHE_INS1/rtl/ipml_sdpram_v1_5_ICACHE_INS1.v 
  ../pnr/ipcore/ICACHE_INS1/ICACHE_INS1.v           
  ../pnr/ipcore/ICACHE_TAG0/rtl/ipml_sdpram_v1_5_ICACHE_TAG0.v 
  ../pnr/ipcore/ICACHE_TAG0/ICACHE_TAG0.v           
  ../pnr/ipcore/ICACHE_TAG1/rtl/ipml_sdpram_v1_5_ICACHE_TAG1.v 
  ../pnr/ipcore/ICACHE_TAG1/ICACHE_TAG1.v          
         
  ../pnr/ipcore/RX_RING/rtl/ipml_sdpram_v1_5_RX_RING.v 
  ../pnr/ipcore/RX_RING/RX_RING.v  
  ../pnr/ipcore/RX_FIFO/rtl/ipml_fifo_ctrl_v1_3.v 
  ../pnr/ipcore/RX_FIFO/rtl/ipml_fifo_v1_5_RX_FIFO.v 
  ../pnr/ipcore/RX_FIFO/rtl/ipml_prefetch_fifo_v1_5_RX_FIFO.v 
  ../pnr/ipcore/RX_FIFO/rtl/ipml_reg_fifo_v1_0.v 
  ../pnr/ipcore/RX_FIFO/rtl/ipml_sdpram_v1_5_RX_FIFO.v 
  ../pnr/ipcore/RX_FIFO/RX_FIFO.v 
  ../pnr/ipcore/TX_FIFO/rtl/ipml_fifo_ctrl_v1_3.v 
  ../pnr/ipcore/TX_FIFO/rtl/ipml_fifo_v1_5_TX_FIFO.v 
  ../pnr/ipcore/TX_FIFO/rtl/ipml_prefetch_fifo_v1_5_TX_FIFO.v 
  ../pnr/ipcore/TX_FIFO/rtl/ipml_reg_fifo_v1_0.v 
  ../pnr/ipcore/TX_FIFO/rtl/ipml_sdpram_v1_5_TX_FIFO.v 
  ../pnr/ipcore/TX_FIFO/TX_FIFO.v       
     
  ../pnr/ipcore/DDR3/example_design/bench/mem/ddr3.v                                                                                                                  
  ../pnr/ipcore/DDR3/rtl/pll/pll_50_400_v1_1.v                                                   
  ../pnr/ipcore/DDR3/rtl/ipsl_hmic_h_ddrc_apb_reset_v1_1.v                                                   
  ../pnr/ipcore/DDR3/rtl/ipsl_hmic_h_ddrc_reset_ctrl_v1_1.v                                              
  ../pnr/ipcore/DDR3/rtl/ipsl_hmic_h_ddrphy_dll_update_ctrl_v1_1.v                                       
  ../pnr/ipcore/DDR3/rtl/ipsl_hmic_h_ddrphy_reset_ctrl_v1_1.v                                            
  ../pnr/ipcore/DDR3/rtl/ipsl_hmic_h_ddrphy_training_ctrl_v1_1.v                                         
  ../pnr/ipcore/DDR3/rtl/ipsl_hmic_h_ddrphy_update_ctrl_v1_1.v                                           
  ../pnr/ipcore/DDR3/rtl/ipsl_hmic_h_ddrc_top_v1_1.v                                               
  ../pnr/ipcore/DDR3/rtl/ipsl_hmic_h_phy_top_v1_1.v                                                
  ../pnr/ipcore/DDR3/rtl/ipsl_hmic_h_phy_io_v1_1.v  
  ../pnr/ipcore/DDR3/DDR3.v  
  
  ../pnr/ipcore/TSMAC_FIFO_RXCKLI/rtl/ipml_fifo_ctrl_v1_3.v
  ../pnr/ipcore/TSMAC_FIFO_RXCKLI/rtl/ipml_fifo_v1_5_TSMAC_FIFO_RXCKLI.v
  ../pnr/ipcore/TSMAC_FIFO_RXCKLI/rtl/ipml_prefetch_fifo_v1_5_TSMAC_FIFO_RXCKLI.v
  ../pnr/ipcore/TSMAC_FIFO_RXCKLI/rtl/ipml_reg_fifo_v1_0.v
  ../pnr/ipcore/TSMAC_FIFO_RXCKLI/rtl/ipml_sdpram_v1_5_TSMAC_FIFO_RXCKLI.v
  ../pnr/ipcore/TSMAC_FIFO_RXCKLI/TSMAC_FIFO_RXCKLI.v
  
  ../pnr/ipcore/PGL_SDPRAM_11/PGL_SDPRAM_11.v
  ../pnr/ipcore/PGL_SDPRAM_11/rtl/ipml_sdpram_v1_5_PGL_SDPRAM_11.v
  
  ../bench/src/tsmac_phy/tsmac_phy.v
  ../bench/src/tsmac_phy/rtl/pgs_tsmac_gmii_to_rgmii_v1_0.v
  ../bench/src/tsmac_phy/rtl/pgs_tsmac_rgmii_gmii_convert_v1_0.v
  ../bench/src/tsmac_phy/rtl/pgs_tsmac_rgmii_to_gmii_v1_0.v
  ../bench/src/tsmac_phy/rtl/pgs_tsmac_core_v1_1.v
  ../bench/src/tsmac_phy/rtl/pgs_tsmac_rx_sm_v1_1.v
  ../bench/src/tsmac_phy/rtl/pgs_tsmac_apb_modify_v1_0.v
  ../bench/src/tsmac_phy/rtl/pgs_tsmac_apb_port_v1_0.v   
  ../bench/src/tsmac_phy/rtl/tsmac_data_ram_v1_1.v            
  ../bench/src/tsmac_phy/rtl/pgm_sdpram_v1_1.v
  ../bench/src/tsmac_phy/rtl/gtp_drm18k_wrapper.v
  ../bench/src/tsmac_phy/sim_lib/modelsim/pe_mcxmac_core_sim.vp
  ../bench/src/tsmac_phy/sim_lib/modelsim/pe_mcxmac_sim.vp
  ../bench/src/tsmac_phy/sim_lib/modelsim/pecar_sim.vp
  ../bench/src/tsmac_phy/sim_lib/modelsim/pecrc_sim.vp
  ../bench/src/tsmac_phy/sim_lib/modelsim/pehst_sim.vp
  ../bench/src/tsmac_phy/sim_lib/modelsim/pemgt_sim.vp
  ../bench/src/tsmac_phy/sim_lib/modelsim/perfn_top_sim.vp
  ../bench/src/tsmac_phy/sim_lib/modelsim/permc_top_sim.vp
  ../bench/src/tsmac_phy/sim_lib/modelsim/petfn_top_sim.vp
  ../bench/src/tsmac_phy/sim_lib/modelsim/petmc_top_sim.vp
  
  ../bench/src/udp_hw_speedup/udp_hw_speedup.vp
  ../bench/src/udp_hw_speedup/tx_speedup.vp

  ../pnr/ipcore/ex_data_fifo/rtl/ipml_fifo_ctrl_v1_3.v
  ../pnr/ipcore/ex_data_fifo/rtl/ipml_fifo_v1_5_ex_data_fifo.v
  ../pnr/ipcore/ex_data_fifo/rtl/ipml_prefetch_fifo_v1_5_ex_data_fifo.v
  ../pnr/ipcore/ex_data_fifo/rtl/ipml_reg_fifo_v1_0.v
  ../pnr/ipcore/ex_data_fifo/rtl/ipml_sdpram_v1_5_ex_data_fifo.v
  ../pnr/ipcore/ex_data_fifo/ex_data_fifo.v
 
  
+define+SIMULATION  
+define+den4096Mb   
+define+sg25E       
+define+x16         
                                                                                                                    