# bridge_top on SKY130 (LibreLane), X.2 GDS build: narrow PHY, small buffers.
# Both clocks 30 ns (33 MHz): relaxed on purpose so place-and-route needs no
# long timing repair (the interconnect run spent hours there at 15 ns).
# clk and aclk are unrelated (async CDC FIFO between them).
create_clock -name clk  -period 30.0 [get_ports clk]
create_clock -name aclk -period 30.0 [get_ports aclk]
set_clock_groups -asynchronous -group [get_clocks clk] -group [get_clocks aclk]
set_clock_uncertainty 0.25 [all_clocks]

# fabric side (aclk)
set_input_delay  3.0 -clock aclk [get_ports {s0_tdata* s0_tkeep* s0_tlast s0_tvalid s1_tdata* s1_tkeep* s1_tlast s1_tvalid}]
set_output_delay 3.0 -clock aclk [get_ports {s0_tready s1_tready session_active* session_overflow* session_end*}]

# host / link side (clk)
set_input_delay  3.0 -clock clk [get_ports {m_rx_axis_tready ts_byte_valid ts_byte_l0* ts_byte_l1* ts_byte_l2* ts_byte_l3* hot_reset_request flr_request updatefc_received}]
set_input_delay  3.0 -clock clk [get_ports {cfg_wr_en cfg_rd_en cfg_addr* cfg_wdata* bar0_wr_en bar0_rd_en bar0_addr* bar0_wdata* tbl_wr_en tbl_wr_vec* tbl_wr_addr* tbl_wr_data* tbl_wr_mask rx_level* rx_level_valid*}]
set_output_delay 3.0 -clock clk [get_ports {m_rx_axis_tvalid m_rx_axis_tdata* m_rx_axis_tkeep* m_rx_axis_tlast link_trained datapath_rst_n axi_rst_n}]
set_output_delay 3.0 -clock clk [get_ports {cfg_rdata* cfg_cpl_valid bar0_rdata* bar0_rd_valid tx_level* tx_level_valid* err_xlat_miss err_addr_ovf crc_err fec_err phy_framing_err phy_lane_err}]

# reset input: asynchronous
set_false_path -from [get_ports perst_n]
