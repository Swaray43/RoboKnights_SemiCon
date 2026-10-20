`timescale 1ns/1ps
// ============================================================
// tb_link_layer_loopback -- S5 (rank 7)
//
// The old bench drove AXI-Stream beats into link_layer_tx/rx and
// is incompatible with the current interface: both modules now
// move whole Flit payloads (from/to the replay buffer) over the
// SYS_BUS_W system bus, with no AXI-Stream side at all. Rewritten
// at the Flit level, as the ticket's second option: one Flit
// link_layer_tx -> (optional error injection) -> link_layer_rx,
// checking fec_corrected / fec_err / crc_err and the ACK/NAK
// piggyback path.
// ============================================================
module tb_link_layer_loopback;

    localparam SYS_BUS_W = 256;
    localparam SEQ_W     = 12;
    localparam PW        = 1888; // FL_TLP_W

    reg  clk, rst_n;

    // TX side (from "replay buffer")
    reg             s_valid;
    reg  [PW-1:0]   s_payload;
    reg  [SEQ_W-1:0] s_seq;
    wire            s_ready;

    reg             ack_req, nak_req;
    reg  [SEQ_W-1:0] ackn_seq;
    wire            ackn_taken;

    wire            tx_valid;
    wire [SYS_BUS_W-1:0] tx_data;
    wire            tx_sof, tx_eof;
    reg             tx_ready;

    // Error injection: one 256-bit XOR mask per bus word (0..7)
    reg [SYS_BUS_W-1:0] err_mask [0:7];
    reg [2:0]           word_ctr;
    wire [SYS_BUS_W-1:0] rx_data_in = tx_data ^ err_mask[word_ctr];

    // RX side
    wire            rx_flit_valid, rx_flit_ok;
    wire [1:0]      rx_kind;
    wire [SEQ_W-1:0] rx_seq;
    wire            rx_ack_valid, rx_nak_valid;
    wire [SEQ_W-1:0] rx_ackn_seq;
    wire [PW-1:0]   rx_payload;
    wire            crc_err, fec_err, fec_corrected;

    integer errors;

    link_layer_tx #(.SYS_BUS_W(SYS_BUS_W), .SEQ_W(SEQ_W)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .s_valid(s_valid), .s_payload(s_payload), .s_seq(s_seq), .s_ready(s_ready),
        .ack_req(ack_req), .nak_req(nak_req), .ackn_seq(ackn_seq), .ackn_taken(ackn_taken),
        .dllp_req(1'b0), .dllp_data(96'd0), .dllp_taken(),
        .m_sys_bus_valid(tx_valid), .m_sys_bus_data(tx_data),
        .m_sys_bus_sof(tx_sof), .m_sys_bus_eof(tx_eof), .m_sys_bus_ready(tx_ready)
    );

    link_layer_rx #(.SYS_BUS_W(SYS_BUS_W), .SEQ_W(SEQ_W)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .s_sys_bus_valid(tx_valid), .s_sys_bus_data(rx_data_in),
        .s_sys_bus_sof(tx_sof), .s_sys_bus_eof(tx_eof),
        .rx_flit_valid(rx_flit_valid), .rx_flit_ok(rx_flit_ok),
        .rx_kind(rx_kind), .rx_seq(rx_seq),
        .rx_ack_valid(rx_ack_valid), .rx_nak_valid(rx_nak_valid), .rx_ackn_seq(rx_ackn_seq),
        .rx_payload(rx_payload),
        .crc_err(crc_err), .fec_err(fec_err), .fec_corrected(fec_corrected)
    );

    always #5 clk = ~clk;

    // Track which bus word (0..7) is currently being driven, so the
    // error mask can target a specific word of the Flit.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) word_ctr <= 3'd0;
        else if (tx_valid && tx_ready) begin
            if (tx_eof) word_ctr <= 3'd0;
            else        word_ctr <= word_ctr + 3'd1;
        end
    end

    integer k;
    task clear_err_mask; begin
        for (k = 0; k < 8; k = k + 1) err_mask[k] = {SYS_BUS_W{1'b0}};
    end endtask

    task check_eq(input [63:0] got, input [63:0] exp, input [255:0] label);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=0x%0h expected=0x%0h", label, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    // -------- capture decoded Flits --------
    reg rx_ev_valid; reg rx_ev_ok; reg [SEQ_W-1:0] rx_ev_seq;
    reg [PW-1:0] rx_ev_payload;
    reg rx_ev_crc_err, rx_ev_fec_err, rx_ev_fec_corrected;
    reg rx_ev_ack_valid; reg [SEQ_W-1:0] rx_ev_ackn_seq; reg [1:0] rx_ev_kind;

    // Sticky latches: a decoded Flit and its status flags are each a
    // single-cycle pulse, so hold them until clear_rx_ev() resets them
    // before the next test phase (otherwise the next cycle's capture
    // would silently overwrite the pulse back to 0).
    always @(posedge clk) begin
        if (rx_flit_valid) begin
            rx_ev_valid          = 1'b1;
            rx_ev_ok             = rx_flit_ok;
            rx_ev_seq            = rx_seq;
            rx_ev_payload        = rx_payload;
            rx_ev_kind           = rx_kind;
            rx_ev_ack_valid      = rx_ack_valid;
            rx_ev_ackn_seq       = rx_ackn_seq;
        end
        if (crc_err)       rx_ev_crc_err       = 1'b1;
        if (fec_err)       rx_ev_fec_err       = 1'b1;
        if (fec_corrected) rx_ev_fec_corrected = 1'b1;
    end

    task clear_rx_ev;
        begin
            rx_ev_valid = 0; rx_ev_ok = 0; rx_ev_seq = 0; rx_ev_payload = 0;
            rx_ev_kind = 0; rx_ev_ack_valid = 0; rx_ev_ackn_seq = 0;
            rx_ev_crc_err = 0; rx_ev_fec_err = 0; rx_ev_fec_corrected = 0;
        end
    endtask

    // Push one payload Flit; race-free (reads s_ready only at posedge).
    task push_flit(input [PW-1:0] payload, input [SEQ_W-1:0] seq);
        begin
            @(negedge clk);
            s_payload = payload; s_seq = seq; s_valid = 1;
            @(posedge clk);
            while (!s_ready) @(posedge clk);
            @(negedge clk);
            s_valid = 0;
        end
    endtask

    // Request an ACK/NAK piggyback on the next (control, if no payload
    // is pending) Flit.
    task request_ackn(input ack, input nak, input [SEQ_W-1:0] seq);
        begin
            @(negedge clk);
            ack_req = ack; nak_req = nak; ackn_seq = seq;
            @(posedge clk);
            while (!ackn_taken) @(posedge clk);
            @(negedge clk);
            ack_req = 0; nak_req = 0;
        end
    endtask

    // Wait for a full Flit to clear the pipeline (CALC + 8 SEND beats).
    task wait_flit_drain; begin repeat (12) @(negedge clk); end endtask

    initial begin
        clk = 0; rst_n = 0;
        s_valid = 0; s_payload = 0; s_seq = 0;
        ack_req = 0; nak_req = 0; ackn_seq = 0;
        tx_ready = 1;
        errors = 0;
        clear_err_mask;

        @(negedge clk); @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ============================================================
        // Test 1: clean Flit, no errors
        // ============================================================
        clear_rx_ev;
        push_flit({PW{1'b1}} ^ 1888'h1357_9BDF, 12'd5);
        wait_flit_drain;
        check_eq(rx_ev_valid, 1, "T1: a Flit was decoded");
        check_eq(rx_ev_ok, 1, "T1: clean Flit decodes OK");
        check_eq(rx_ev_crc_err, 0, "T1: no CRC error");
        check_eq(rx_ev_fec_err, 0, "T1: no FEC error");
        check_eq(rx_ev_fec_corrected, 0, "T1: nothing to correct");
        check_eq(rx_ev_seq, 5, "T1: sequence number preserved");
        check_eq(rx_ev_payload, {PW{1'b1}} ^ 1888'h1357_9BDF, "T1: payload bit-exact");

        // ============================================================
        // Test 2: single-bit error -> corrected by FEC
        // ============================================================
        clear_err_mask;
        clear_rx_ev;
        err_mask[0] = 256'd1; // flip flit bit 0 (byte0 bit0, interleave j=0)
        push_flit({PW{1'b0}} ^ 1888'hDEAD_BEEF, 12'd6);
        wait_flit_drain;
        check_eq(rx_ev_valid, 1, "T2: a Flit was decoded");
        check_eq(rx_ev_fec_corrected, 1, "T2: single-bit error corrected by FEC");
        check_eq(rx_ev_fec_err, 0, "T2: not flagged uncorrectable");
        check_eq(rx_ev_crc_err, 0, "T2: CRC passes once FEC has fixed the bit");
        check_eq(rx_ev_ok, 1, "T2: Flit still reported OK (corrected)");
        check_eq(rx_ev_payload, {PW{1'b0}} ^ 1888'hDEAD_BEEF, "T2: payload recovered bit-exact");

        // ============================================================
        // Test 3: two-bit error in the same FEC interleave -> uncorrectable
        // ============================================================
        clear_err_mask;
        clear_rx_ev;
        err_mask[0] = (256'd1 << 0) | (256'd1 << 24); // byte0 bit0 + byte3 bit0, both interleave j=0
        push_flit({PW{1'b1}} ^ 1888'hFEED_FACE, 12'd7);
        wait_flit_drain;
        check_eq(rx_ev_valid, 1, "T3: a Flit was decoded");
        check_eq(rx_ev_fec_err, 1, "T3: double-bit error (same interleave) flagged uncorrectable");
        check_eq(rx_ev_ok, 0, "T3: Flit reported not-OK");

        // ============================================================
        // Test 4: ACK/NAK piggyback on a control-only Flit
        // ============================================================
        clear_err_mask;
        clear_rx_ev;
        request_ackn(1'b1, 1'b0, 12'd42); // ACK seq 42, no payload pending
        wait_flit_drain;
        check_eq(rx_ev_valid, 1, "T4: a control Flit was decoded");
        check_eq(rx_ev_kind, 2'd1, "T4: Flit kind == CONTROL");
        check_eq(rx_ev_ack_valid, 1, "T4: ACK flag carried through");
        check_eq(rx_ev_ackn_seq, 42, "T4: ACK sequence number carried through");

        if (errors == 0) $display("RESULT: PASS");
        else              $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #20000;
        $display("[ERROR] Watchdog timeout");
        $display("RESULT: FAIL (watchdog)");
        $finish;
    end

endmodule