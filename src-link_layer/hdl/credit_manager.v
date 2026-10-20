`timescale 1ns/1ps

module credit_manager (
    input  wire        clk,
    input  wire        rst_n,

    // UpdateFC DLLP Interface (from RX Engine / DLLP)
    input  wire        update_fc_valid,
    input  wire [1:0]  update_fc_type,       // 2'b00: Posted, 2'b01: Non-Posted, 2'b10: Completion
    input  wire [7:0]  update_fc_hdr_limit,
    input  wire [11:0] update_fc_data_limit,

    // Credit Allocation Interface (from TLP Packetizer)
    input  wire        alloc_req_valid,
    input  wire [1:0]  alloc_req_type,       // 2'b00: Posted (MWr), 2'b01: Non-Posted (MRd), 2'b10: Cpl
    input  wire [9:0]  alloc_req_dw_len,     // Segment payload length in DW (0 for MRd)
    output wire        alloc_req_grant,
    output wire        stall
);

    // Credit Registers (5 Pools)
    reg [7:0]  ph_limit,   ph_consumed;
    reg [11:0] pd_limit,   pd_consumed;
    reg [7:0]  nph_limit,  nph_consumed;
    reg [11:0] npd_limit,  npd_consumed;
    reg [7:0]  cplh_limit, cplh_consumed;
    reg [11:0] cpld_limit, cpld_consumed;

    // ---- G2 (P2-2): PCIe FC semantics ----------------------------------
    // * The FIRST update for a pool after reset is its InitFC value. An
    //   advertised value of 0 there means INFINITE credits for that field
    //   (header and data independently). 0x7F / 0xFFF are ordinary finite
    //   limits -- the old "0x7F/0xFFF = infinite" sentinel is gone.
    // * Later updates (UpdateFC) move a finite limit; for an infinite field
    //   they are ignored (the spec requires them to be 0 anyway).
    // * A pool that has not been initialised yet grants nothing: no TLP of
    //   that type may go out before FC init (data_link_layer_top's
    //   FC_INIT1 delivers the InitFC values here).
    reg p_init, np_init, cpl_init;
    reg ph_inf_r, pd_inf_r, nph_inf_r, npd_inf_r, cplh_inf_r, cpld_inf_r;

    wire is_ph_inf   = ph_inf_r;
    wire is_pd_inf   = pd_inf_r;
    wire is_nph_inf  = nph_inf_r;
    wire is_npd_inf  = npd_inf_r;
    wire is_cplh_inf = cplh_inf_r;
    wire is_cpld_inf = cpld_inf_r;

    wire upd_hdr_inf  = (update_fc_hdr_limit  == 8'd0);
    wire upd_data_inf = (update_fc_data_limit == 12'd0);

    // Modulo 2's Complement Credit Available Logic
    wire [7:0]  ph_avail   = (ph_limit   - ph_consumed)   & 8'hFF;
    wire [11:0] pd_avail   = (pd_limit   - pd_consumed)   & 12'hFFF;
    wire [7:0]  nph_avail  = (nph_limit  - nph_consumed)  & 8'hFF;
    wire [11:0] npd_avail  = (npd_limit  - npd_consumed)  & 12'hFFF;
    wire [7:0]  cplh_avail = (cplh_limit - cplh_consumed) & 8'hFF;
    wire [11:0] cpld_avail = (cpld_limit - cpld_consumed) & 12'hFFF;

    // Convert DW length to 4-DW Data Credits (rounded up)
    wire [11:0] req_data_credits = (alloc_req_dw_len == 10'd0) ? 12'd0 :
                                   ((alloc_req_dw_len + 10'd3) >> 2);

    // Credit Availability Checks
    wire ph_ok   = is_ph_inf   || (ph_avail >= 8'd1);
    wire pd_ok   = is_pd_inf   || (pd_avail >= req_data_credits);
    wire nph_ok  = is_nph_inf  || (nph_avail >= 8'd1);
    wire npd_ok  = is_npd_inf  || (npd_avail >= req_data_credits);
    wire cplh_ok = is_cplh_inf || (cplh_avail >= 8'd1);
    wire cpld_ok = is_cpld_inf || (cpld_avail >= req_data_credits);

    reg has_credits;
    always @(*) begin
        case (alloc_req_type)
            2'b00:   has_credits = p_init   && ph_ok   && pd_ok;   // MWr
            2'b01:   has_credits = np_init  && nph_ok  && npd_ok;  // MRd
            2'b10:   has_credits = cpl_init && cplh_ok && cpld_ok; // CplD
            default: has_credits = 1'b0;
        endcase
    end

    assign alloc_req_grant = alloc_req_valid && has_credits;
    assign stall           = alloc_req_valid && !has_credits;

    // Sequential Tracking Block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ph_limit      <= 8'd0;   ph_consumed   <= 8'd0;
            pd_limit      <= 12'd0;  pd_consumed   <= 12'd0;
            nph_limit     <= 8'd0;   nph_consumed  <= 8'd0;
            npd_limit     <= 12'd0;  npd_consumed  <= 12'd0;
            cplh_limit    <= 8'd0;   cplh_consumed <= 8'd0;
            cpld_limit    <= 12'd0;  cpld_consumed <= 12'd0;
            p_init        <= 1'b0;   np_init       <= 1'b0;   cpl_init   <= 1'b0;
            ph_inf_r      <= 1'b0;   pd_inf_r      <= 1'b0;
            nph_inf_r     <= 1'b0;   npd_inf_r     <= 1'b0;
            cplh_inf_r    <= 1'b0;   cpld_inf_r    <= 1'b0;
        end else begin
            // InitFC (first update per pool) / UpdateFC (later updates)
            if (update_fc_valid) begin
                case (update_fc_type)
                    2'b00: begin
                        if (!p_init) begin
                            p_init   <= 1'b1;
                            ph_inf_r <= upd_hdr_inf;
                            pd_inf_r <= upd_data_inf;
                            ph_limit <= update_fc_hdr_limit;
                            pd_limit <= update_fc_data_limit;
                        end else begin
                            if (!ph_inf_r) ph_limit <= update_fc_hdr_limit;
                            if (!pd_inf_r) pd_limit <= update_fc_data_limit;
                        end
                    end
                    2'b01: begin
                        if (!np_init) begin
                            np_init   <= 1'b1;
                            nph_inf_r <= upd_hdr_inf;
                            npd_inf_r <= upd_data_inf;
                            nph_limit <= update_fc_hdr_limit;
                            npd_limit <= update_fc_data_limit;
                        end else begin
                            if (!nph_inf_r) nph_limit <= update_fc_hdr_limit;
                            if (!npd_inf_r) npd_limit <= update_fc_data_limit;
                        end
                    end
                    2'b10: begin
                        if (!cpl_init) begin
                            cpl_init   <= 1'b1;
                            cplh_inf_r <= upd_hdr_inf;
                            cpld_inf_r <= upd_data_inf;
                            cplh_limit <= update_fc_hdr_limit;
                            cpld_limit <= update_fc_data_limit;
                        end else begin
                            if (!cplh_inf_r) cplh_limit <= update_fc_hdr_limit;
                            if (!cpld_inf_r) cpld_limit <= update_fc_data_limit;
                        end
                    end
                    default: ;
                endcase
            end

            // Deduct Credits on Grant
            if (alloc_req_grant) begin
                case (alloc_req_type)
                    2'b00: begin
                        ph_consumed <= ph_consumed + 8'd1;
                        pd_consumed <= pd_consumed + req_data_credits;
                    end
                    2'b01: begin
                        nph_consumed <= nph_consumed + 8'd1;
                        npd_consumed <= npd_consumed + req_data_credits;
                    end
                    2'b10: begin
                        cplh_consumed <= cplh_consumed + 8'd1;
                        cpld_consumed <= cpld_consumed + req_data_credits;
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule