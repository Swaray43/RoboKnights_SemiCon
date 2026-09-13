// ------------------------------------------------------------------
// session_arm_ctrl.v
// Extracted standalone from pcie_tag_man.v for integration with the
// interconnect layer. pcie_tag_man.v also defines its own (unused,
// not instantiated by pcie_link_top.v or tb_full_system_integration.v)
// axi4s_ingest_port and session_overflow_ctrl modules that duplicate
// interconnect_layer/src/axi4s_ingest_port.v's territory -- do NOT
// `include pcie_tag_man.v` wholesale alongside the interconnect layer,
// or those name collide. Only session_arm_ctrl is needed here.
//
// Session-arm handshake: host writes base_addr + (optional TC), then
// pulses session_arm (W1S). FSM latches into shadow regs one cycle
// later and asserts session_valid for the fabric logic to gate on.
// IDLE -> LATCH -> ACTIVE -> IDLE (on session end or re-arm)
// ------------------------------------------------------------------
module session_arm_ctrl #(
    parameter ADDR_WIDTH = 64,
    parameter TC_WIDTH   = 3,
    parameter ARM_CNT_WIDTH = 8   // saturating debug counter width
)(
    input  wire                   clk,
    input  wire                   rst_n,

    input  wire [ADDR_WIDTH-1:0]  base_addr_in,
    input  wire [TC_WIDTH-1:0]    tc_in,
    input  wire                   session_arm,     // W1S pulse from host reg write
    input  wire                   session_end,     // from fabric: TLAST or overflow abort

    output reg  [ADDR_WIDTH-1:0]  base_addr_latched,
    output reg  [TC_WIDTH-1:0]    tc_latched,
    output reg                    session_valid,

    // debug/bring-up visibility, readable via CfgRd like other counters
    output reg [ARM_CNT_WIDTH-1:0] session_arm_while_active_cnt
);

    localparam IDLE   = 2'b00;
    localparam LATCH  = 2'b01;
    localparam ACTIVE = 2'b10;

    reg [1:0] state, next_state;

    always @(posedge clk) begin
        if (!rst_n) state <= IDLE;
        else        state <= next_state;
    end

    always @(*) begin
        next_state = state;
        case (state)
            IDLE:   if (session_arm)  next_state = LATCH;
            LATCH:  next_state = ACTIVE;
            ACTIVE: if (session_end)  next_state = IDLE;
                    // re-arm while ACTIVE is now IGNORED here on purpose --
                    // no transition to LATCH. session_end must retire first.
            default: next_state = IDLE;
        endcase
    end

    // Detect the unsafe condition combinationally for the counter
    wire arm_while_active = (state == ACTIVE) && session_arm;

    always @(posedge clk) begin
        if (!rst_n) begin
            base_addr_latched            <= {ADDR_WIDTH{1'b0}};
            tc_latched                    <= {TC_WIDTH{1'b0}};
            session_valid                 <= 1'b0;
            session_arm_while_active_cnt  <= {ARM_CNT_WIDTH{1'b0}};
        end else begin
            if (state == LATCH) begin
                base_addr_latched <= base_addr_in;
                tc_latched         <= tc_in;
            end
            session_valid <= (next_state == ACTIVE);

            // Saturating counter -- never wraps silently
            if (arm_while_active && !(&session_arm_while_active_cnt))
                session_arm_while_active_cnt <= session_arm_while_active_cnt + 1'b1;
        end
    end
endmodule
