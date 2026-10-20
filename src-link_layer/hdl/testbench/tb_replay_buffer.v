`timescale 1ns/1ps
// ============================================================
// tb_replay_buffer -- S2 (rank 2)
//
// Revived against the current interface: PAYLOAD_W (was
// FLIT_WIDTH), plus the new m_seq/outstanding/replay_event
// outputs. Scaled down (PAYLOAD_W=64, SEQ_WIDTH=4, BUF_DEPTH=8,
// REPLAY_TIMEOUT=20) for a fast, easy-to-read sim.
//
// Checks:
//   1. ACK frees exactly the acknowledged Flits (outstanding drops)
//   2. NAK rewinds to the NAKed sequence and resends in order
//      (replay_event pulses, data replayed matches original)
//   3. No ACK within REPLAY_TIMEOUT cycles -> timeout replay
//   4. Buffer full -> s_ready low, nothing dropped; draining one
//      slot (ACK) lets the next push through intact
// ============================================================
module tb_replay_buffer;

    localparam PW   = 64;
    localparam SW   = 4;
    localparam BD   = 8;
    localparam RTO  = 20;

    reg            clk, rst_n;
    reg            s_valid;
    reg  [PW-1:0]  s_data;
    wire           s_ready;

    wire           m_valid;
    wire [PW-1:0]  m_data;
    wire [SW-1:0]  m_seq;
    reg            m_ready;

    reg            rx_ack_valid, rx_nak_valid;
    reg  [SW-1:0]  rx_ack_seq, rx_nak_seq;

    wire [SW-1:0]  outstanding;
    wire           replay_event;

    integer errors;

    replay_buffer #(
        .PAYLOAD_W(PW), .SEQ_WIDTH(SW), .BUF_DEPTH(BD), .REPLAY_TIMEOUT(RTO)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .s_valid(s_valid), .s_data(s_data), .s_ready(s_ready),
        .m_valid(m_valid), .m_data(m_data), .m_seq(m_seq), .m_ready(m_ready),
        .rx_ack_valid(rx_ack_valid), .rx_ack_seq(rx_ack_seq),
        .rx_nak_valid(rx_nak_valid), .rx_nak_seq(rx_nak_seq),
        .outstanding(outstanding), .replay_event(replay_event)
    );

    always #5 clk = ~clk;

    // -------- capture every transmitted (seq,data) word --------
    reg [PW-1:0] cap_data [0:255];
    reg [SW-1:0] cap_seq  [0:255];
    integer      cap_cnt;
    integer      replay_pulses;

    always @(posedge clk) begin
        if (m_valid && m_ready) begin
            cap_data[cap_cnt] = m_data;
            cap_seq [cap_cnt] = m_seq;
            cap_cnt = cap_cnt + 1;
        end
        if (replay_event) replay_pulses = replay_pulses + 1;
    end

    task reset_capture; begin cap_cnt = 0; replay_pulses = 0; end endtask

    task check_eq(input [63:0] got, input [63:0] exp, input [255:0] label);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=%0d expected=%0d", label, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    task do_reset;
        begin
            rst_n = 0; s_valid = 0; m_ready = 0;
            rx_ack_valid = 0; rx_nak_valid = 0;
            rx_ack_seq = 0; rx_nak_seq = 0;
            @(negedge clk); @(negedge clk);
            rst_n = 1;
            @(negedge clk);
        end
    endtask

    // Push one Flit; race-free (reads s_ready only at posedge).
    task push_flit(input [PW-1:0] data);
        begin
            @(negedge clk);
            s_data = data; s_valid = 1;
            @(posedge clk);
            while (!s_ready) @(posedge clk);
            @(negedge clk);
            s_valid = 0;
        end
    endtask

    integer k;

    initial begin
        clk = 0;
        errors = 0; cap_cnt = 0; replay_pulses = 0;

        // ============================================================
        // Test 1: ACK frees exactly the acknowledged Flits
        // ============================================================
        do_reset;
        m_ready = 1;
        reset_capture;
        push_flit(64'hAAAA_AAAA_0000_0000);
        push_flit(64'hBBBB_BBBB_0000_0001);
        push_flit(64'hCCCC_CCCC_0000_0002);
        repeat (6) @(negedge clk); // let tx_seq catch up to head (all 3 "sent")
        check_eq(outstanding, 3, "T1: outstanding before ACK");
        @(negedge clk);
        rx_ack_valid = 1; rx_ack_seq = 1; // ACK seq 0 and 1
        @(negedge clk);
        rx_ack_valid = 0;
        @(negedge clk);
        check_eq(outstanding, 1, "T1: outstanding after ACK(seq=1)");

        // ============================================================
        // Test 2: NAK rewinds and resends in order
        // ============================================================
        do_reset;
        m_ready = 1;
        reset_capture;
        push_flit(64'h1111_1111_0000_0000); // seq0
        push_flit(64'h2222_2222_0000_0001); // seq1
        push_flit(64'h3333_3333_0000_0002); // seq2
        repeat (6) @(negedge clk); // all 3 already sent once, outstanding=3
        check_eq(outstanding, 3, "T2: outstanding before NAK");
        cap_cnt = 0; // forget the first pass, we only care about the replay
        @(negedge clk);
        rx_nak_valid = 1; rx_nak_seq = 0; // NAK seq0 -> ack 0, replay from 1
        @(negedge clk);
        rx_nak_valid = 0;
        repeat (6) @(negedge clk); // let the replay drain
        check_eq(replay_pulses >= 1, 1, "T2: replay_event pulsed on NAK");
        check_eq(outstanding, 2, "T2: outstanding after NAK(seq=0)");
        check_eq(cap_cnt, 2, "T2: exactly 2 Flits replayed (seq1,seq2)");
        check_eq(cap_seq[0], 1, "T2: replay order seq1 first");
        check_eq(cap_data[0], 64'h2222_2222_0000_0001, "T2: replay data seq1 matches original");
        check_eq(cap_seq[1], 2, "T2: replay order seq2 second");
        check_eq(cap_data[1], 64'h3333_3333_0000_0002, "T2: replay data seq2 matches original");

        // ============================================================
        // Test 3: no ACK within REPLAY_TIMEOUT -> timeout replay
        // ============================================================
        do_reset;
        m_ready = 1;
        reset_capture;
        push_flit(64'h4444_4444_0000_0000); // seq0
        push_flit(64'h5555_5555_0000_0001); // seq1
        repeat (4) @(negedge clk); // both sent once, outstanding=2, no ack/nak
        cap_cnt = 0;
        check_eq(outstanding, 2, "T3: outstanding before timeout");
        check_eq(replay_pulses, 0, "T3: no replay yet");
        repeat (RTO + 4) @(negedge clk); // wait past REPLAY_TIMEOUT
        check_eq(replay_pulses >= 1, 1, "T3: replay_event pulsed on timeout");
        check_eq(outstanding, 2, "T3: outstanding unchanged (nothing ACKed)");
        check_eq(cap_cnt, 2, "T3: both Flits replayed after timeout");
        check_eq(cap_seq[0], 0, "T3: replay order seq0 first");
        check_eq(cap_seq[1], 1, "T3: replay order seq1 second");

        // ============================================================
        // Test 4: buffer full -> s_ready low, nothing dropped
        // ============================================================
        do_reset;
        m_ready = 0; // never drain -- force the buffer to fill up
        reset_capture;
        for (k = 0; k < BD; k = k + 1)
            push_flit(64'hF000_0000_0000_0000 + k);
        @(negedge clk);
        check_eq(s_ready, 0, "T4: s_ready low once BUF_DEPTH Flits are outstanding");
        check_eq(outstanding, BD, "T4: outstanding == BUF_DEPTH");

        // Attempt a 9th push while full: s_valid asserted but must not
        // be accepted (s_ready stays low) -- nothing should be dropped.
        s_data  = 64'hDEAD_0000_0000_0009;
        s_valid = 1;
        repeat (3) @(negedge clk);
        check_eq(s_ready, 0, "T4: still not ready while full (9th push stalled)");

        // Free one slot via ACK, then the stalled push should go through.
        rx_ack_valid = 1; rx_ack_seq = 0; // ACK seq0 only
        @(negedge clk);
        rx_ack_valid = 0;
        @(posedge clk);
        while (!s_ready) @(posedge clk);
        @(negedge clk);
        s_valid = 0;
        check_eq(outstanding, BD, "T4: 9th Flit accepted, outstanding back at BUF_DEPTH");

        // Drain everything and confirm the original 8 Flits (seq1..7)
        // plus the 9th (seq8) come out intact and in order.
        m_ready = 1;
        reset_capture;
        repeat (BD + 5) @(negedge clk);
        check_eq(cap_cnt, BD, "T4: BUF_DEPTH Flits drained (seq1..8)");
        for (k = 0; k < BD - 1; k = k + 1)
            check_eq(cap_data[k], 64'hF000_0000_0000_0000 + k + 1, "T4: drained data matches original push, no corruption");
        check_eq(cap_data[BD-1], 64'hDEAD_0000_0000_0009, "T4: 9th Flit drained last, intact");

        if (errors == 0) $display("RESULT: PASS");
        else              $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #50000;
        $display("[ERROR] Watchdog timeout");
        $display("RESULT: FAIL (watchdog)");
        $finish;
    end

endmodule