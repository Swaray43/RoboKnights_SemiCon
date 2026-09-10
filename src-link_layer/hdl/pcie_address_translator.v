`timescale 1ns/1ps

module pcie_address_translator #(
    parameter PCIE_ADDR_WIDTH = 64,
    parameter AXI_ADDR_WIDTH  = 32
)(
    input  wire                       clk,
    input  wire                       rst_n,

    input  wire [PCIE_ADDR_WIDTH-1:0] trans_window_base,
    input  wire [PCIE_ADDR_WIDTH-1:0] trans_window_mask,
    input  wire [AXI_ADDR_WIDTH-1:0]  trans_axi_base,

    input  wire                       req_valid,
    input  wire [PCIE_ADDR_WIDTH-1:0] req_pcie_addr,

    input  wire                       axi_addr_ready, // AXI4 Handshake addition
    output reg                        axi_addr_valid,
    output reg  [AXI_ADDR_WIDTH-1:0]  axi_addr,
    output reg                        translation_err
);

    wire [PCIE_ADDR_WIDTH-1:0] masked_req_addr = req_pcie_addr & trans_window_mask;
    wire hit = (masked_req_addr == trans_window_base);
    wire [AXI_ADDR_WIDTH-1:0] offset = req_pcie_addr[AXI_ADDR_WIDTH-1:0] & ~trans_window_mask[AXI_ADDR_WIDTH-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_addr_valid  <= 1'b0;
            axi_addr        <= 0;
            translation_err <= 1'b0;
        end else begin
            // Hold valid and data if AXI slave is not ready
            if (axi_addr_valid && !axi_addr_ready) begin
                axi_addr_valid  <= 1'b1;
            end else if (req_valid) begin
                if (hit) begin
                    axi_addr_valid  <= 1'b1;
                    axi_addr        <= trans_axi_base + offset;
                    translation_err <= 1'b0;
                end else begin
                    axi_addr_valid  <= 1'b0;
                    translation_err <= 1'b1;
                end
            end else begin
                axi_addr_valid  <= 1'b0;
                translation_err <= 1'b0;
            end
        end
    end

endmodule