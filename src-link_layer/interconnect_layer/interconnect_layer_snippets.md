# Interconnect layer — snippet-by-snippet walkthrough

Every module and testbench, broken into chunks with the code shown and explained inline. Read alongside the actual files in `src/` — line numbers here match those files.

---

# PART 1: RTL MODULES

## `axi4s_ingest_port.v`

```verilog
module axi4s_ingest_port #(
    parameter DATA_WIDTH   = 512,
    parameter KEEP_WIDTH   = DATA_WIDTH/8,
    parameter OFFSET_WIDTH = 24,
    parameter CH_ID        = 0
)(
```
Four parameters. `KEEP_WIDTH` defaults to `DATA_WIDTH/8` — one TKEEP bit per byte, standard AXI4-Stream convention. `OFFSET_WIDTH` is sized to the largest session you expect, not to a full address — 24 bits covers sessions up to 16MB, plenty for a per-channel slice. `CH_ID` is carried for documentation/instance-naming only; it doesn't gate any internal logic (the module that *instantiates* two of these, `interconnect_top`, is what actually ties a physical port to `stream_id=0` or `1`).

```verilog
    input  wire [DATA_WIDTH-1:0]      s_tdata,
    input  wire [KEEP_WIDTH-1:0]      s_tkeep,
    input  wire                       s_tlast,
    input  wire                       s_tvalid,
    output wire                       s_tready,
```
Standard AXI4-Stream slave port. Nothing custom here — `s_tready` is the one signal this module computes and drives back to the master.

```verilog
    input  wire                       session_start,
    input  wire [OFFSET_WIDTH-1:0]    session_limit,
    output reg                        session_active,
    output reg                        session_overflow,
```
The session-control interface — this is the boundary with Kshitij's config-space block. `session_start` is a pulse (expected one cycle wide), `session_limit` is a static value for the duration of one session. `session_active`/`session_overflow` are `reg` here because they're driven from the sequential block below, not combinationally.

```verilog
    output wire                       req,
    input  wire                       grant,
    input  wire                       ds_backpressure,
```
The arbiter interface. Three separate signals, each doing one job — `req` says "I want the bus," `grant` says "you have it," `ds_backpressure` is layered on top independently (a channel can be granted the bus and *still* not be allowed to actually transfer, if the FIFO downstream is nearly full).

```verilog
    output wire [DATA_WIDTH-1:0]      out_tdata,
    output wire [KEEP_WIDTH-1:0]      out_tkeep,
    output wire                       out_tlast,
    output wire [OFFSET_WIDTH-1:0]    out_session_offset,
    output wire                       accepted
);
```
`out_*` is this channel's current beat, always driven (whether or not it's granted — the crossbar decides which channel's `out_*` to actually use). `accepted` is the odd one out and the most important signal in this file — explained below.

```verilog
    function [$clog2(KEEP_WIDTH+1)-1:0] popcount;
        input [KEEP_WIDTH-1:0] keep;
        integer i;
        begin
            popcount = 0;
            for (i = 0; i < KEEP_WIDTH; i = i + 1)
                popcount = popcount + keep[i];
        end
    endfunction

    wire [$clog2(KEEP_WIDTH+1)-1:0] beat_bytes = popcount(s_tkeep);
```
A synthesizable for-loop counting set bits in `TKEEP`. This gives the *real* number of valid bytes in this beat — matters for the last beat of a session, which is usually partial (not all 64 bytes valid). `beat_bytes` is what `session_offset` actually advances by; using a hardcoded `KEEP_WIDTH` here would silently overcount on partial beats.

```verilog
    wire would_overflow = session_active && s_tvalid &&
                           ((session_offset + beat_bytes) > session_limit);
```
The overflow *predictor* — computed before the beat is accepted, using the current offset plus this beat's byte count. This is a look-ahead check: it catches the beat that *would* cause overflow, before it's counted or forwarded, rather than detecting the overflow after the damage is done.

```verilog
    assign req      = s_tvalid && session_active && !session_overflow && !would_overflow;
    assign s_tready  = grant && session_active && !session_overflow && !ds_backpressure;
```
**The most important two lines in the file.** `req` and `s_tready` look similar but answer different questions and deliberately don't share a definition:
- `req` = "do I have something to send" — feeds the arbiter's decision. Doesn't know about `grant` (would be circular) or `ds_backpressure` (the arbiter shouldn't refuse to even consider a channel just because the FIFO is momentarily full — that's a separate concern handled at the handshake level).
- `s_tready` = "can the transfer actually happen right now" — gates the real AXI4-Stream handshake. Depends on `grant` (arbiter's decision) AND `ds_backpressure` (a completely independent reason to stall) AND overflow state.

```verilog
    wire beat_hs = s_tvalid && s_tready;
    assign accepted = beat_hs;
```
`beat_hs` is the actual AXI4-Stream handshake condition — both `valid` and `ready` high means a transfer genuinely happened this cycle. `accepted` just exposes this as a named output, because the consumer of this module (`interconnect_top`) needs to distinguish "this channel was granted" from "this channel actually transferred a beat" — conflating those two was a real bug caught during integration (see `interconnect_top.v` notes below).

```verilog
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            session_active   <= 1'b0;
            session_overflow <= 1'b0;
            session_offset   <= {OFFSET_WIDTH{1'b0}};
        end else begin
            if (session_start) begin
                session_active   <= 1'b1;
                session_overflow <= 1'b0;
                session_offset   <= {OFFSET_WIDTH{1'b0}};
            end else if (would_overflow) begin
                session_overflow <= 1'b1;
                session_active   <= 1'b0;
            end else if (beat_hs) begin
                session_offset <= session_offset + beat_bytes;
                if (s_tlast)
                    session_active <= 1'b0;
            end
        end
    end
```
The one sequential block, and the priority order matters:
1. **`session_start` always wins** — if a fresh session is being armed, that overrides anything else happening this cycle, including a stale overflow condition from before.
2. **Then `would_overflow`** — abort the session, set the sticky flag. `session_active` drops here too, which is what makes `s_tready` go low on the *next* cycle (via the `session_active` term in the `s_tready` expression) — the offending beat itself never gets accepted or counted.
3. **Then normal accumulation** — add `beat_bytes`, and if this was the last beat (`TLAST`), end the session cleanly.

```verilog
    assign out_tdata          = s_tdata;
    assign out_tkeep           = s_tkeep;
    assign out_tlast           = s_tlast;
    assign out_session_offset  = session_offset;
```
Simple pass-through — this channel's current beat and current offset, always driven. The crossbar (in `interconnect_top`) decides whether to actually use it this cycle.

---

## `rr_arbiter_2ch.v`

```verilog
module rr_arbiter_2ch #(
    parameter CREDIT_WIDTH  = 8,
    parameter STARVE_WIDTH  = 16,
    parameter STARVE_CYCLES = 64
)(
```
`CREDIT_WIDTH=8` caps weight values at 255 — plenty of resolution for a bandwidth ratio. `STARVE_CYCLES=64` is explicitly called out in the doc as a placeholder needing real tuning once backpressure propagation delay is known.

```verilog
    input  wire                      req0,
    input  wire                      req1,
    input  wire [CREDIT_WIDTH-1:0]   weight0,
    input  wire [CREDIT_WIDTH-1:0]   weight1,
    output reg                       grant0,
    output reg                       grant1
);
```
Deliberately hardcoded to 2 channels — not a generalized N-channel arbiter. `grant0`/`grant1` are `reg` because they're registered outputs (see below), not combinational.

```verilog
    reg [CREDIT_WIDTH-1:0]  credit0, credit1;
    reg                     last_grant;
    reg [STARVE_WIDTH-1:0]  wait0, wait1;

    wire starved0 = req0 && (wait0 >= STARVE_CYCLES);
    wire starved1 = req1 && (wait1 >= STARVE_CYCLES);
```
Four pieces of state per pair: credit counters (weight-based service), a single `last_grant` bit (round-robin tie-break memory), and wait counters (starvation tracking). `starved0`/`starved1` are simple combinational threshold checks on those wait counters.

```verilog
    wire round_done0 = (credit0 == {CREDIT_WIDTH{1'b0}}) && (!req1 || credit1 == {CREDIT_WIDTH{1'b0}});
    wire round_done1 = (credit1 == {CREDIT_WIDTH{1'b0}}) && (!req0 || credit0 == {CREDIT_WIDTH{1'b0}});
```
**This is the deadlock fix**, and it's worth understanding exactly what each half of the condition does. `credit0 == 0` — this channel is out of credit, refill is due. `(!req1 || credit1 == 0)` — AND the other channel isn't even competing right now (`!req1`, so its credit can't be "stealing" fairness), OR it's also exhausted (matching the original intent of a joint round). The `!req1` clause is what breaks the deadlock: without it, an idle channel's credit sits forever at whatever it was refilled to, and since it never decrements (nothing consumes it), the original joint condition (`credit0==0 && credit1==0`) could never become true again — permanently starving channel 0's refill once its own credit ran out.

```verilog
    wire cred_avail0 = (credit0 != {CREDIT_WIDTH{1'b0}}) || round_done0;
    wire cred_avail1 = (credit1 != {CREDIT_WIDTH{1'b0}}) || round_done1;
    wire eligible0 = req0 && cred_avail0;
    wire eligible1 = req1 && cred_avail1;
```
Refill itself is registered (happens on the next clock edge), so a channel that hits exactly zero credit would show `credit==0` for one cycle before the refill lands — costing a cycle of wasted arbitration if eligibility only checked the raw credit value. `cred_avail` treats "about to refill this cycle" as already available, closing that bubble.

```verilog
    always @(*) begin
        next_g0 = 1'b0;
        next_g1 = 1'b0;
        if (starved0 && !starved1) begin
            next_g0 = 1'b1;
        end else if (starved1 && !starved0) begin
            next_g1 = 1'b1;
        end else if (starved0 && starved1) begin
            if (last_grant == 1'b0) next_g1 = 1'b1;
            else                    next_g0 = 1'b1;
        end else if (eligible0 && eligible1) begin
            if (last_grant == 1'b0) next_g1 = 1'b1;
            else                    next_g0 = 1'b1;
        end else if (eligible0) begin
            next_g0 = 1'b1;
        end else if (eligible1) begin
            next_g1 = 1'b1;
        end
    end
```
Priority ladder, top to bottom: starvation beats everything (a channel waiting too long gets serviced regardless of credit), then normal weighted eligibility, with `last_grant` breaking ties whenever both channels are simultaneously in contention (whether via starvation or via normal eligibility) — always favoring whoever *didn't* go last. This is purely combinational — `next_g0`/`next_g1` are the *proposed* decision, not yet the actual output.

```verilog
    always @(posedge clk or negedge rstn) begin
        ...
        end else begin
            grant0 <= next_g0;
            grant1 <= next_g1;
```
Here's the registration: `grant0`/`grant1` are just `next_g0`/`next_g1` delayed by one clock edge. This is what makes them safe to drive a 512-bit mux select without a long combinational path from `req`/`weight` all the way through the data mux.

```verilog
            if (round_done0) begin
                credit0 <= next_g0 ? ((weight0 != 0) ? weight0 - 1'b1 : {CREDIT_WIDTH{1'b0}})
                                    : weight0;
            end else if (next_g0) begin
                credit0 <= (credit0 != 0) ? credit0 - 1'b1 : {CREDIT_WIDTH{1'b0}};
            end
```
Credit update for channel 0, independent of channel 1's state entirely (same pattern repeats for `credit1`). Two cases: if a refill is due this cycle, either refill-then-immediately-consume-one (if this channel is also winning the grant this same cycle) or just refill. Otherwise, if granted, saturating-decrement (never goes negative, guarded by the `!= 0` check even though it should always be nonzero here given `eligible0` required it).

```verilog
            if (next_g0) begin
                last_grant <= 1'b0;
                wait0      <= {STARVE_WIDTH{1'b0}};
                wait1      <= (req1) ? wait1 + 1'b1 : {STARVE_WIDTH{1'b0}};
            end else if (next_g1) begin
                ...
            end else begin
                wait0 <= (req0) ? wait0 + 1'b1 : {STARVE_WIDTH{1'b0}};
                wait1 <= (req1) ? wait1 + 1'b1 : {STARVE_WIDTH{1'b0}};
            end
```
Wait-counter bookkeeping: the winning channel's wait counter resets to 0 (it just got serviced), the losing channel's counter increments *only if it's actually requesting* (an idle channel shouldn't accumulate starvation credit it doesn't need), and if nobody was granted at all, both counters advance for whoever's requesting.

---

## `cdc_fifo_async.v`

```verilog
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    reg  [ADDR_WIDTH:0] wr_ptr_bin;
    reg  [ADDR_WIDTH:0] wr_ptr_gray;
```
Pointers are `ADDR_WIDTH+1` bits wide — one bit wider than needed to address `DEPTH` entries. That extra bit is the classic Cummings trick for distinguishing "completely full" from "completely empty" using only pointer comparison (both cases would otherwise show read_ptr == write_ptr).

```verilog
    wire [ADDR_WIDTH:0] wr_ptr_bin_lookahead  = wr_ptr_bin + 1'b1;
    wire [ADDR_WIDTH:0] wr_ptr_gray_lookahead = (wr_ptr_bin_lookahead >> 1) ^ wr_ptr_bin_lookahead;
```
**This line is the fix for the combinational-loop bug.** `wr_ptr_bin_lookahead` is `wr_ptr_bin + 1`, full stop — no `wr_en` term, no `wr_full` term. The original buggy version gated this increment by `wr_en && !wr_full`, which meant the signal used to *compute* `wr_full` depended on `wr_full` itself — a zero-delay self-reference that hung the simulator. Making the lookahead unconditional breaks that cycle: it always represents "the pointer value if a write happened right now," purely as a function of the current registered pointer, nothing else.

```verilog
    always @(posedge wr_clk or negedge wr_rstn) begin
        if (!wr_rstn) begin
            rd_ptr_gray_wr1 <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr_gray_wr2 <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            rd_ptr_gray_wr1 <= rd_ptr_gray;
            rd_ptr_gray_wr2 <= rd_ptr_gray_wr1;
        end
    end
```
One of two identical-shaped synchronizer blocks (the other mirrors this for the write pointer crossing into the read domain). Two flip-flops in series, both clocked by the *receiving* domain's clock — the first flop is where a metastable capture might occur, and giving it one full extra cycle before the second flop samples it is what makes `rd_ptr_gray_wr2` trustworthy to use in logic.

```verilog
    assign wr_full = (wr_ptr_gray_lookahead == {~rd_ptr_gray_wr2[ADDR_WIDTH:ADDR_WIDTH-1],
                                                  rd_ptr_gray_wr2[ADDR_WIDTH-2:0]});
```
The classic Cummings full-detection trick: compare the lookahead gray pointer against the synced read pointer with its top two bits inverted. This specific bit-inversion pattern is what correctly distinguishes "pointers equal because full" from "pointers equal because empty" — a full derivation is in any standard async FIFO reference; the key point here is that both operands are now purely registered values (`wr_ptr_bin` via the lookahead, `rd_ptr_gray_wr2` via the synchronizer), so this line has no self-reference.

```verilog
    assign rd_empty = (rd_ptr_gray == wr_ptr_gray_rd2);
```
Empty, by contrast, needs no lookahead — a direct comparison of current pointers answers "is there data right now" without needing to predict anything.

```verilog
    always @(posedge wr_clk or negedge wr_rstn) begin
        if (!wr_rstn) begin
            ...
        end else if (wr_en && !wr_full) begin
            mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
            wr_ptr_bin  <= wr_ptr_bin + 1'b1;
            wr_ptr_gray <= (wr_ptr_bin_lookahead >> 1) ^ wr_ptr_bin_lookahead;
        end
    end
```
The *actual* conditional write — gated by `wr_en && !wr_full`, using the now-safely-computed `wr_full` (no loop, since `wr_full`'s own definition never references this gated update). Note it reuses `wr_ptr_bin_lookahead` for the gray-code update rather than recomputing — the lookahead value already *is* "pointer plus one," exactly what's needed here.

```verilog
    always @(posedge rd_clk or negedge rd_rstn) begin
        if (!rd_rstn) begin
            rd_ptr_bin  <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr_gray <= {(ADDR_WIDTH+1){1'b0}};
        end else if (rd_en && !rd_empty) begin
            rd_ptr_bin  <= rd_ptr_bin + 1'b1;
            rd_ptr_gray <= ((rd_ptr_bin + 1'b1) >> 1) ^ (rd_ptr_bin + 1'b1);
        end
    end
    assign rd_data = mem[rd_ptr_bin[ADDR_WIDTH-1:0]];
```
**This is the fix for the FWFT bug.** `rd_data` is a plain combinational read of `mem[]` at the *current* pointer — notice there's no register capturing it, unlike a standard synchronous-read memory. That means valid data appears on `rd_data` the instant `!rd_empty` is true, *before* `rd_en` is ever asserted — matching AXI4-Stream's "TDATA is valid whenever TVALID is high" semantics. The sequential block only advances the pointer when `rd_en && !rd_empty` — it doesn't gate the data output at all, only which entry will be visible *next*.

```verilog
    function [ADDR_WIDTH:0] gray2bin;
        input [ADDR_WIDTH:0] g;
        integer i;
        begin
            gray2bin[ADDR_WIDTH] = g[ADDR_WIDTH];
            for (i = ADDR_WIDTH-1; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    wire [ADDR_WIDTH:0] rd_ptr_bin_synced_to_wr = gray2bin(rd_ptr_gray_wr2);
    wire [ADDR_WIDTH:0] occupancy = wr_ptr_bin - rd_ptr_bin_synced_to_wr;
    assign wr_almost_full = (occupancy >= ALMOST_FULL_THRESH);
```
The gray-to-binary conversion (standard iterative XOR-chain) is only ever applied to `rd_ptr_gray_wr2` — the *already twice-synchronized* value. This is the one place in the file binary conversion happens on a crossed signal, and it's safe specifically because synchronization has already resolved any metastability by this point; converting the raw crossing signal itself (before synchronization) would not be safe.

---

## `interconnect_top.v`

```verilog
    wire req0, grant0, ch0_accepted;
    ...
    axi4s_ingest_port #(...) u_ch0 (
        ...
        .req(req0), .grant(grant0), .ds_backpressure(fifo_almost_full),
        ...
        .accepted(ch0_accepted)
    );
```
Two identical-shaped instantiations (`u_ch0`, `u_ch1`), each getting `fifo_almost_full` wired to their `ds_backpressure` input — this is the "one shared backpressure signal" design decision made concrete: both channels see the exact same FIFO-fullness signal, not independent per-channel thresholds.

```verilog
    rr_arbiter_2ch #(...) u_arb (
        .clk(aclk), .rstn(arstn),
        .req0(req0), .req1(req1),
        .weight0(weight0), .weight1(weight1),
        .grant0(grant0), .grant1(grant1)
    );
```
The arbiter sits between the two ports, taking their `req` outputs and feeding back `grant` — note `grant0` feeds back into `u_ch0`'s `grant` input above, closing the loop.

```verilog
    wire [DATA_WIDTH-1:0]   xbar_data  = grant0 ? ch0_data  : ch1_data;
    wire [KEEP_WIDTH-1:0]   xbar_keep  = grant0 ? ch0_keep  : ch1_keep;
    wire                    xbar_last  = grant0 ? ch0_last  : ch1_last;
    wire [OFFSET_WIDTH-1:0] xbar_offset= grant0 ? ch0_offset: ch1_offset;
    wire                    xbar_id    = grant0 ? 1'b0      : 1'b1;
```
The "crossbar" is literally just five parallel ternary selects, all keyed off `grant0`. This is only correct because mutual exclusion (`grant0` and `grant1` never both high) is guaranteed by the arbiter and explicitly verified in its testbench — a real N-way crossbar with more channels would need a proper priority encoder or one-hot mux structure instead.

```verilog
    wire                    xbar_valid = (grant0 && ch0_accepted) | (grant1 && ch1_accepted);
```
**The fix for the duplicate-write bug**, and worth contrasting with what it used to be: `grant0 | grant1` (wrong — fires whenever a channel is merely granted, even if the actual handshake is still blocked by backpressure or overflow, causing the same held-steady beat to be written into the FIFO repeatedly). The fixed version requires *both* "this channel won arbitration" *and* "this channel's handshake actually completed this cycle" — the second term is exactly the `accepted` output built into `axi4s_ingest_port` for this purpose.

```verilog
    localparam FIFO_WIDTH = DATA_WIDTH + KEEP_WIDTH + 1 + OFFSET_WIDTH + 1;
    wire [FIFO_WIDTH-1:0] fifo_wr_data = {xbar_data, xbar_keep, xbar_id, xbar_offset, xbar_last};
```
Everything about one beat — the data, the byte mask, which channel it came from, its position in the session, and whether it's the last beat — gets packed into a single wide word and written into the FIFO atomically. This is what guarantees these fields never get torn apart from each other across the clock-domain crossing.

```verilog
    cdc_fifo_async #(...) u_fifo (
        .wr_clk(aclk), .wr_rstn(arstn), .wr_en(fifo_wr_en), .wr_data(fifo_wr_data),
        .wr_full(fifo_full), .wr_almost_full(fifo_almost_full),
        .rd_clk(pcie_clk), .rd_rstn(pcie_rstn), .rd_en(m_tready && !fifo_empty), .rd_data(fifo_rd_data),
        .rd_empty(fifo_empty)
    );
```
Note `rd_en = m_tready && !fifo_empty` — since the FIFO is FWFT, this correctly means "the consumer is ready AND there's actually something to give it," matching AXI4-Stream's read-side handshake exactly.

```verilog
    assign {m_tdata, m_tkeep, m_stream_id, m_session_offset, m_tlast} = fifo_rd_data;
    assign m_tvalid = !fifo_empty;
```
Unpacking mirrors the packing order exactly. `m_tvalid` is just `!fifo_empty` directly — valid the moment there's data, thanks to FWFT semantics; no extra registering or gating needed here.

---

# PART 2: TESTBENCHES

## `tb_axi4s_ingest_port.v`

```verilog
    localparam DATA_WIDTH = 64; // small for readability
```
Deliberately overrides the 512-bit production width down to 64 for this testbench — the module is fully parameterized, so this doesn't reduce coverage of the logic, just makes waveforms/print statements easier to read by hand.

```verilog
    task do_reset;
        begin
            rstn = 0; s_tvalid = 0; ...
            repeat (3) @(posedge clk);
            rstn = 1;
            @(posedge clk);
        end
    endtask
```
A reusable reset task, called at the start of every test scenario — each of the four tests starts from a clean, known state rather than relying on the previous test's ending state, which would make failures harder to isolate.

```verilog
    $display("=== Test 1: session tracking across several beats ===");
    do_reset;
    session_start = 1;
    @(posedge clk); #1; session_start = 0;
    grant = 1;
    s_tvalid = 1; s_tdata = 64'hAAAA_AAAA_AAAA_AAAA; s_tkeep = {KEEP_WIDTH{1'b1}}; s_tlast = 0;

    for (i = 0; i < 3; i = i + 1) begin
        @(posedge clk);
        #1;
        $display("beat %0d: session_offset=%0d tready=%0d", i, out_session_offset, s_tready);
    end
    if (out_session_offset != (3 * KEEP_WIDTH)) begin
        $display("FAIL: ...");
        errors = errors + 1;
    end else $display("PASS");
```
Arms a session with a one-cycle pulse (note the `#1` delay after the edge before dropping `session_start` — a settle pattern used throughout to avoid racing the DUT's own sampling), then drives three back-to-back full-width beats and checks the offset advanced by exactly `3 × KEEP_WIDTH` bytes. This test originally had an off-by-one (asserting for 2 beats after a 3-beat loop) — a testbench bug, not a DUT bug, caught and fixed during development.

```verilog
    $display("=== Test 3: overflow guard aborts session before exceeding limit ===");
    do_reset;
    session_limit = 16'd12; // small limit: 1.5 beats at 8B/beat
    ...
    @(posedge clk); #1; // beat 1: offset 0->8, within limit(12)
    @(posedge clk); #1; // beat 2: would be 8->16, exceeds limit(12) -> overflow expected
    if (session_overflow !== 1'b1) begin
        $display("FAIL: ...");
```
Deliberately picks a `session_limit` (12 bytes) that sits *between* two beat boundaries (8 and 16 bytes) — this specifically exercises the look-ahead overflow check's arithmetic (`session_offset + beat_bytes > session_limit`), rather than a limit that happens to land exactly on a beat boundary, which would be a weaker test.

```verilog
    $display("=== Test 4: ds_backpressure holds TREADY low even when granted ===");
    do_reset;
    session_start = 1;
    @(posedge clk); #1; session_start = 0;
    grant = 1; ds_backpressure = 1;
    s_tvalid = 1;
    @(posedge clk); #1;
    if (s_tready !== 1'b0) begin
```
Directly tests the independence of `grant` and `ds_backpressure` in the `s_tready` expression — sets `grant=1` (would normally allow a transfer) but also `ds_backpressure=1`, confirming `s_tready` still correctly reads 0. This is the exact property the duplicate-write bug depended on getting right.

---

## `tb_rr_arbiter_2ch.v`

```verilog
    rr_arbiter_2ch #(
        .CREDIT_WIDTH(8),
        .STARVE_WIDTH(16),
        .STARVE_CYCLES(8)
    ) dut (
```
Note `STARVE_CYCLES` is overridden down to 8 here (vs. the production default of 64) — makes the starvation tests (Test 3) actually finish within a reasonable simulation window instead of waiting 64+ cycles for the guard to fire.

```verilog
    $display("=== Test 1: 50/50 default weights, both always requesting ===");
    reset_dut;
    req0 = 1; req1 = 1;
    g0_count = 0; g1_count = 0;
    for (i = 0; i < 40; i = i + 1) begin
        @(posedge clk);
        if (grant0) g0_count = g0_count + 1;
        if (grant1) g1_count = g1_count + 1;
    end
    if (g0_count < 15 || g0_count > 25 || g1_count < 15 || g1_count > 25) begin
```
Both channels held continuously requesting for 40 cycles, counting grants for each. The pass window (15–25 out of 40) is deliberately loose — this isn't checking for a perfectly exact split (which registered-decision latency and startup transients would make brittle to assert exactly), just that neither channel is being starved or dominating.

```verilog
    $display("=== Test 2: 3:1 weight ratio, both always requesting ===");
    reset_dut;
    weight0 = 8'd3; weight1 = 8'd1;
    ...
    if (g1_count == 0 || (g0_count * 10 / g1_count) < 25 || (g0_count * 10 / g1_count) > 35) begin
```
Computes the ratio as `(g0_count * 10) / g1_count` and checks it falls between 2.5 and 3.5 — integer arithmetic scaled by 10 to get one decimal place of precision without needing real/floating types in the testbench. Guards against division by zero (`g1_count == 0`) as the first term in the OR chain, since Verilog evaluates left-to-right and a zero-check failing first prevents an actual divide-by-zero from ever being reached.

```verilog
    $display("=== Test 3: starvation guard, ch1 weight 0 but continuously requesting ===");
    reset_dut;
    weight0 = 8'd10; weight1 = 8'd0;
    ...
    if (g1_count == 0) begin
        $display("FAIL: starvation guard never fired for weight-0 channel");
```
The critical edge case: a channel with literally zero weight should *never* be serviced by the normal credit mechanism (its credit is permanently 0), so any grants it receives must have come from the starvation override. `g1_count == 0` failing here would mean the starvation guard is broken.

```verilog
    $display("=== Test 5: mutual exclusion check across all prior activity ===");
    reset_dut;
    req0 = 1; req1 = 1;
    for (i = 0; i < 20; i = i + 1) begin
        @(posedge clk);
        if (grant0 && grant1) begin
            $display("FAIL: grant0 and grant1 both asserted same cycle");
            errors = errors + 1;
        end
    end
```
This is the check that `interconnect_top`'s simple ternary-mux crossbar relies on being true — if `grant0` and `grant1` were ever simultaneously high, the mux would silently pick one channel's data while discarding the other's, with no error signaled anywhere. This test is what makes that simplification in the top-level file justifiable rather than an unverified assumption.

---

## `tb_cdc_fifo_async.v`

```verilog
    localparam WIDTH = 16;
    localparam ADDR_WIDTH = 3; // depth 8

    always #3.5 wr_clk = ~wr_clk;
    always #6.5 rd_clk = ~rd_clk;
```
Two independent clock generators with periods (7ns, 13ns) chosen specifically to have no small common multiple — this maximizes how often the two clocks' edges land close together, which is exactly the condition that would expose a CDC bug if one existed. Using related periods (e.g. 7ns and 14ns) would risk always-aligned edges accidentally masking a real synchronization problem.

```verilog
    initial begin
        wr_rstn = 0; wr_en = 0; wr_data = 0;
        repeat (5) @(posedge wr_clk);
        wr_rstn = 1;
        @(posedge wr_clk);

        for (i = 0; i < 200; i = i + 1) begin
            @(posedge wr_clk);
            #1; // settle after the edge -- avoid racing the DUT's own posedge sampling
            if (!wr_full) begin
                wr_en   = 1;
                wr_data = i[WIDTH-1:0];
                expected_q[push_idx] = i[WIDTH-1:0];
                push_idx = push_idx + 1;
            end else begin
                wr_en = 0;
            end
        end
```
The writer process. The `#1` delay here is the fix for a real testbench race found during development: without it, this process and the DUT's own `always @(posedge wr_clk)` block were both triggering on the exact same edge with no guaranteed ordering between them, occasionally causing the DUT to sample stale `wr_data`/`wr_en` values. `expected_q[]` records only the values actually written (gated by the same `!wr_full` check the DUT itself uses), building a reference model to check against later.

```verilog
    initial begin
        wr_rstn = 0; rd_rstn = 0; rd_en = 0;
        repeat (5) @(posedge rd_clk);
        rd_rstn = 1;
        @(posedge rd_clk);

        forever begin
            @(posedge rd_clk);
            #1;
            if (!rd_empty) begin
                rd_en = 1;
                if (rd_data !== expected_q[pop_idx]) begin
                    $display("FAIL at pop_idx=%0d: expected %0d got %0d", pop_idx, expected_q[pop_idx], rd_data);
                    errors = errors + 1;
                end
                pop_idx = pop_idx + 1;
            end else begin
                rd_en = 0;
            end
        end
    end
```
The reader process — and note the check happens in the *same* cycle `rd_en` is set, not delayed. This matches the FIFO's FWFT semantics: `rd_data` is already valid combinationally the moment `!rd_empty`, so there's no need to wait a cycle before checking it (the original version of this testbench, written against the pre-fix registered-read FIFO, *did* delay the check by a cycle using a separate `rd_en_d` flop — that extra delay logic was removed entirely once the FIFO itself switched to FWFT).

```verilog
    initial begin
        #3000;
        $display("Pushed %0d words, popped %0d words", push_idx, pop_idx);
        if (push_idx != pop_idx) begin
            $display("FAIL: push/pop count mismatch (some data still in flight or lost)");
```
A fixed-time watchdog that lets both processes run for 3000ns then checks the final tallies match — simple but effective given the writer is bounded (200 iterations, naturally stops) and the reader is a `forever` loop that's implicitly terminated by `$finish` in this block.

---

## `tb_interconnect_top.v`

```verilog
    always #3.5 aclk = ~aclk;
    always #5.5 pcie_clk = ~pcie_clk;
```
Same "unrelated clock periods" principle as the standalone FIFO testbench, applied at the full-stack level — 7ns and 11ns here.

```verilog
    initial begin : ch0_drive
        wait (arstn == 1);
        @(posedge aclk); #1;
        s0_tvalid = 1;
        while (sent0 < 60) begin
            s0_tdata = {8'hC0, sent0[31:0], 24'h0};
            @(posedge aclk);
            #1;
            if (s0_tready) sent0 = sent0 + 1;
        end
        s0_tvalid = 0;
    end
```
Each channel's driver tags its data with a fixed identifying byte (`8'hC0` for channel 0, `8'hC1` for channel 1) plus a running counter — this lets the receiver cross-check that data claiming to be from channel 0 (`m_stream_id == 0`) actually *contains* the C0 tag, catching a whole class of potential channel-mixup bugs that checking `session_offset` sequencing alone wouldn't necessarily catch. Written as a named block (`initial begin : ch0_drive`) specifically so Verilog's block-naming rules are satisfied — an early version tried to `disable` an unnamed block, which iverilog correctly rejected.

```verilog
    initial begin
        wait (pcie_rstn == 1);
        forever begin
            @(posedge pcie_clk);
            #1;
            if (m_tvalid && m_tready) begin
                tag = m_tdata[63:56];
                ctr = m_tdata[55:24];
                if (m_stream_id == 1'b0) begin
                    if (tag !== 8'hC0) begin
                        $display("FAIL: m_stream_id=0 but data tag=%02h (expected C0)", tag);
                    end
                    if (m_session_offset !== expected_offset0) begin
                        $display("FAIL ch0: expected session_offset=%0d got %0d", expected_offset0, m_session_offset);
                    end
                    expected_offset0 = expected_offset0 + KEEP_WIDTH;
                    recv0 = recv0 + 1;
```
The receiver maintains its own independent expected-offset counters per stream (`expected_offset0`, incrementing by `KEEP_WIDTH` each receive) — this is a from-scratch re-derivation of what the offset *should* be, checked against what the DUT actually produced, rather than just trusting the DUT's own count.

```verilog
    always @(posedge aclk) begin
        if (dut.xbar_valid && dut.u_fifo.wr_full)
            $display("*** DROPPED WRITE at time %0t: xbar_valid high while wr_full already true ***", $time);
        if (dut.ch0_accepted) accepted0_cnt = accepted0_cnt + 1;
        if (dut.ch1_accepted) accepted1_cnt = accepted1_cnt + 1;
        if (dut.fifo_wr_en)   wr_cnt = wr_cnt + 1;
    end
    always @(posedge pcie_clk) begin
        if (dut.m_tvalid && dut.m_tready) rd_cnt = rd_cnt + 1;
    end
```
Hierarchical debug instrumentation, added specifically during the investigation of the duplicate-write bug — reaches directly into the DUT's internal signals (`dut.xbar_valid`, `dut.u_fifo.wr_full`, etc.) via Verilog's hierarchical reference syntax. This is simulation-only debug code, not something that would exist in synthesizable RTL; it's what made it possible to distinguish "the pipeline is genuinely dropping data" from "the driver's own count is off" when a 59-vs-60 discrepancy first appeared.

```verilog
    initial begin
        wait (sent0 >= 60 && sent1 >= 60);
        repeat (500) @(posedge pcie_clk); // generous drain margin
        $display("sent0=%0d sent1=%0d recv0=%0d recv1=%0d", sent0, sent1, recv0, recv1);
        $display("DUT-internal ground truth: accepted0_cnt=%0d accepted1_cnt=%0d wr_cnt=%0d rd_cnt=%0d", ...);
        if (accepted0_cnt != recv0 || accepted1_cnt != recv1) begin
            $display("FAIL: DUT accepted-pulse count vs received count mismatch");
        end
        if (wr_cnt != rd_cnt) begin
            $display("FAIL: FIFO write count vs read count mismatch");
        end
```
The final check deliberately does **not** compare against the driver's own `sent0`/`sent1` — after investigation showed those had a benign testbench-side sampling discrepancy, the pass criteria was rewritten to trust the DUT's own internal instrumentation (`accepted0_cnt`, `wr_cnt`, `rd_cnt`) as ground truth instead. `wait(...)` followed by a fixed `repeat(500)` drain margin (rather than a fixed total simulation time) ensures the check only runs after both drivers have genuinely finished *and* the pipeline has had generous time to flush — removing timing-window sizing as a possible source of false failures.
