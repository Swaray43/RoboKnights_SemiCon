`timescale 1ns/1ps
module tb_ingest_overflow_leak;
  reg clk=0, rstn=0; always #5 clk=~clk;
  reg [511:0] d=0; reg [63:0] k={64{1'b1}}; reg l=0, v=0, start=0;
  wire rdy, act, ovf, req0, req1, g0, g1, acc; wire [511:0] od; wire [63:0] ok; wire ol; wire [23:0] off;
  axi4s_ingest_port #(.CH_ID(0)) u (.clk(clk),.rstn(rstn),.s_tdata(d),.s_tkeep(k),.s_tlast(l),.s_tvalid(v),.s_tready(rdy),
    .session_start(start),.session_limit(24'd200),.session_active(act),.session_overflow(ovf),
    .req(req0),.grant(g0),.ds_backpressure(1'b0),.out_tdata(od),.out_tkeep(ok),.out_tlast(ol),.out_session_offset(off),.accepted(acc));
  rr_arbiter_2ch #(.CREDIT_WIDTH(8),.STARVE_WIDTH(16),.STARVE_CYCLES(64)) a (.clk(clk),.rstn(rstn),.req0(req0),.req1(1'b0),
    .weight0(8'd1),.weight1(8'd1),.grant0(g0),.grant1(g1));
  integer n=0, lim_hits=0;
  always @(posedge clk) if (acc) begin n=n+1; $display("[%0t] #%0d %s offset %0d tkeep=%h tlast=%b%s", $time, n, (v&&rdy)?"DATA beat handshaked at":"marker (no data) at", off, ok, ol, ((v&&rdy) && off+64>200)?"  <-- DATA BEYOND LIMIT":""); end
  initial begin
    repeat(3) @(posedge clk); rstn=1; @(negedge clk); start=1; @(negedge clk); start=0;
    v=1;                               // continuous valid, 64 B beats, limit 200 B -> only 3 beats fit
    repeat(20) @(posedge clk);
    $display("accepted=%0d (3 fit in 200 B), overflow=%b", n, ovf); $finish;
  end
endmodule
