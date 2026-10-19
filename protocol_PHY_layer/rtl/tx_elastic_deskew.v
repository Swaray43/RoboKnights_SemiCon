module tx_elastic_deskew #(
parameter DEPTH = 8,
parameter THRESHOLD = 4,
parameter ROOM_BYTES = 8        // room_ok asserts when every lane has >= this many free slots
)(
input  wire       clk,
input  wire       rst_n,
input  wire [3:0] wr_en,
input  wire [7:0] wr_data_0, wr_data_1, wr_data_2, wr_data_3,
input  wire       rd_en,
output wire [7:0] rd_data_0, rd_data_1, rd_data_2, rd_data_3,
output wire       lanes_valid_out,  // LIVE per-cycle availability, gated by initial priming
output wire       room_ok           // NEW: every lane has >= ROOM_BYTES free entries. Lets the
                                    // writer (pcie_phy_tx) avoid overflowing a lane FIFO, which
                                    // elastic_fifo would otherwise drop silently (wr_en && full).
);

// PTR_WIDTH used to be hard-coded to 3 below, which only worked for DEPTH=8.
localparam PW = (DEPTH <= 2) ? 1 : (DEPTH <= 4) ? 2 : (DEPTH <= 8) ? 3 :
                (DEPTH <= 16) ? 4 : (DEPTH <= 32) ? 5 : (DEPTH <= 64) ? 6 : 7;

wire [3:0] empty, full;
wire [3:0] fifo_rd_en;
wire [PW:0] fc0, fc1, fc2, fc3;

elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(PW)) fifo0 (
  .clk(clk), .rst_n(rst_n), .wr_en(wr_en[0]), .wr_data(wr_data_0),
  .rd_en(fifo_rd_en[0]), .rd_data(rd_data_0), .fill_count(fc0), .empty(empty[0]), .full(full[0]));
elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(PW)) fifo1 (
  .clk(clk), .rst_n(rst_n), .wr_en(wr_en[1]), .wr_data(wr_data_1),
  .rd_en(fifo_rd_en[1]), .rd_data(rd_data_1), .fill_count(fc1), .empty(empty[1]), .full(full[1]));
elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(PW)) fifo2 (
  .clk(clk), .rst_n(rst_n), .wr_en(wr_en[2]), .wr_data(wr_data_2),
  .rd_en(fifo_rd_en[2]), .rd_data(rd_data_2), .fill_count(fc2), .empty(empty[2]), .full(full[2]));
elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(PW)) fifo3 (
  .clk(clk), .rst_n(rst_n), .wr_en(wr_en[3]), .wr_data(wr_data_3),
  .rd_en(fifo_rd_en[3]), .rd_data(rd_data_3), .fill_count(fc3), .empty(empty[3]), .full(full[3]));

reg primed_latched;
wire all_primed_now = (fc0 >= THRESHOLD) && (fc1 >= THRESHOLD) &&
                       (fc2 >= THRESHOLD) && (fc3 >= THRESHOLD);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    primed_latched <= 1'b0;
  end else begin
    if (!primed_latched && all_primed_now)
      primed_latched <= 1'b1;
  end
end

// Fixed: previously this was latched off primed_latched alone, which stayed
// high even after a lane went empty (a real underrun bug caught in testing).
// Now it drops immediately if ANY lane runs dry, post-priming.
assign lanes_valid_out = primed_latched && !empty[0] && !empty[1] && !empty[2] && !empty[3];

assign room_ok = (fc0 <= DEPTH - ROOM_BYTES) && (fc1 <= DEPTH - ROOM_BYTES) &&
                 (fc2 <= DEPTH - ROOM_BYTES) && (fc3 <= DEPTH - ROOM_BYTES);

assign fifo_rd_en[0] = primed_latched && rd_en && !empty[0];
assign fifo_rd_en[1] = primed_latched && rd_en && !empty[1];
assign fifo_rd_en[2] = primed_latched && rd_en && !empty[2];
assign fifo_rd_en[3] = primed_latched && rd_en && !empty[3];
endmodule
