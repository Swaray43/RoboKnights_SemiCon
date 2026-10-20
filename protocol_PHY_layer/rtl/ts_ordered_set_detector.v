// -----------------------------------------------------------------------------
// Lane reversal support (TS1/TS2, scoped to lane-reversal detection only -
// full rate/equalization negotiation explicitly out of scope). Symbol-level
// input, reassembled to bytes upstream via symbol_to_byte before reaching here.
// -----------------------------------------------------------------------------
module ts_ordered_set_detector (
input  wire       clk,
input  wire       rst_n,
input  wire       byte_valid,
input  wire [7:0] byte_in,
output reg        lane_num_valid,
output reg  [7:0] lane_num_out,
output reg        os_valid,       // NEW: pulse when any well-formed TS1/TS2 ends (for pcie_ltssm)
output reg        os_ts2          // NEW: ...and it was a TS2
);

localparam COM     = 8'hBC;
localparam TS1_ID  = 8'h4A;
localparam TS2_ID  = 8'h45;
localparam IDLE    = 2'd0;
localparam CAPTURE = 2'd1;

reg [1:0] state;
reg [3:0] byte_idx;
reg [7:0] lane_candidate;
reg [7:0] id_byte0;
reg       id_match;
reg [7:0] prev_confirmed;
reg       prev_valid;

// Same-cycle check for THIS byte, folded into the byte-15 decision directly -
// fixes the one-cycle-lag bug where a corrupted byte 15 wasn't caught in time.
wire byte_ok_now = (byte_idx <= 4'd10) ||
                   (byte_in == id_byte0 && (id_byte0 == TS1_ID || id_byte0 == TS2_ID));

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    state          <= IDLE;
    byte_idx       <= 4'd0;
    lane_num_valid <= 1'b0;
    lane_num_out   <= 8'd0;
    prev_valid     <= 1'b0;
    prev_confirmed <= 8'd0;
    id_match       <= 1'b1;
    os_valid       <= 1'b0;
    os_ts2         <= 1'b0;
  end else begin
    lane_num_valid <= 1'b0;
    os_valid       <= 1'b0;

    if (byte_valid) begin
      case (state)
        IDLE: begin
          if (byte_in == COM) begin
            state    <= CAPTURE;
            byte_idx <= 4'd1;
            id_match <= 1'b1;
          end
        end

        CAPTURE: begin
          if (byte_idx == 4'd2)
            lane_candidate <= byte_in;

          if (byte_idx == 4'd10)
            id_byte0 <= byte_in;
          else if (byte_idx > 4'd10) begin
            if (byte_in != id_byte0 || !(id_byte0 == TS1_ID || id_byte0 == TS2_ID))
              id_match <= 1'b0;
          end

          if (byte_idx == 4'd15) begin
            if (id_match && byte_ok_now && (id_byte0 == TS1_ID || id_byte0 == TS2_ID)) begin
              os_valid <= 1'b1;
              os_ts2   <= (id_byte0 == TS2_ID);
              if (prev_valid && (prev_confirmed == lane_candidate)) begin
                lane_num_valid <= 1'b1;
                lane_num_out   <= lane_candidate;
              end
              prev_confirmed <= lane_candidate;
              prev_valid     <= 1'b1;
            end else begin
              prev_valid <= 1'b0;
            end
            state <= IDLE;
          end else begin
            byte_idx <= byte_idx + 4'd1;   // fix: this increment was previously dropped
          end
        end
      endcase
    end
  end
end
endmodule
