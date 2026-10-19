// =============================================================================
// pcie_config_space + msix_controller
//
// Scope: exactly the "Config space block" and "MSI-X controller" bullets from
// work_distribution.md. Built with the DMA addressing document's base_addr_ch0/1
// question still OPEN, so BARs are generic (size+base, standard PCI BAR-sizing
// convention) - NOT wired to any DMA-channel-specific meaning. Revisit BAR
// count/semantics once that document settles.
//
// Address map (byte addresses, DWORD-aligned, reduced/documented vs full PCIe
// spec - same "architecture honesty" approach as the rest of this project):
//   0x00        Vendor/Device ID              (RO)
//   0x04        Status/Command                (Command[1:0] RW: mem space en,
//                                               bus master en; Status RO)
//   0x08        Class Code/Revision ID        (RO)
//   0x0C        BIST/HeaderType/LatTmr/CLS    (RO, HeaderType=0 fixed)
//   0x10-0x24   BAR0-BAR5                     (RW, size-masked per BAR)
//   0x28        Cardbus CIS ptr               (RO 0, unused)
//   0x2C        Subsystem Vendor/Device ID    (RO)
//   0x30        Expansion ROM base            (RO 0, not implemented)
//   0x34        Capabilities pointer -> 0x40  (RO)
//   0x38        Reserved                      (RO 0)
//   0x3C        Int Pin/Line/Max Lat/Min Gnt  (Int Line RW, rest RO 0)
//   0x40        PCIe Cap header (id=0x10, next=0x50) + PCIe Capabilities (RO)
//   0x44        Device Capabilities           (RO 0, not modeled)
//   0x48        Device Status/Device Control  (Device Control RW: MPS[7:5],
//                                               MRRS[14:12], enable bits[4:0])
//   0x4C        Link Capabilities             (RO 0, not modeled)
//   0x50        MSI-X Cap header (id=0x11, next=0x00) + Message Control
//                                              (MSI-X Enable[31] RW,
//                                               Func Mask[30] RW, TblSize RO)
//   0x54        MSI-X Table Offset/BIR        (RO)
//   0x58        MSI-X PBA Offset/BIR          (RO)
//   0x5C        Link Status (speed/width/training bit)  (RO, from live inputs
//                                              - doc names this as its own
//                                              bullet, given a dedicated
//                                              offset here rather than folding
//                                              it into 0x4C since we're not
//                                              modeling full Link Capabilities)
// =============================================================================

module pcie_config_space #(
    parameter [15:0] VENDOR_ID  = 16'hFACE,
    parameter [15:0] DEVICE_ID  = 16'hC0DE,
    parameter [23:0] CLASS_CODE = 24'h020000,   // network controller, per problem statement's SmartNIC use case
    parameter [7:0]  REVISION_ID = 8'h01,

    // BAR sizes: each BAR's writable/settable bits are determined by its size
    // (standard PCI BAR-sizing: bits below the size boundary read back as 0
    // regardless of what the host writes, revealing size on the size-probe
    // read). SIZE must be a power of two; 0 means "BAR not implemented / hardwired 0".
    parameter [31:0] BAR0_SIZE = 32'h0001_0000,  // 64KB
    parameter [31:0] BAR1_SIZE = 32'h0,
    parameter [31:0] BAR2_SIZE = 32'h0,
    parameter [31:0] BAR3_SIZE = 32'h0,
    parameter [31:0] BAR4_SIZE = 32'h0,
    parameter [31:0] BAR5_SIZE = 32'h0,

    parameter        MSIX_TABLE_SIZE_M1 = 11'd7  // "table size" field is N-1; 8 vectors default for sim
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- CfgRd/CfgWr TLP-decoded access (from Swaray's RX engine) ----
    input  wire        cfg_wr_en,
    input  wire        cfg_rd_en,
    input  wire [11:0] cfg_addr,      // byte address, DWORD-aligned (cfg_addr[1:0] ignored)
    input  wire [31:0] cfg_wdata,
    output reg  [31:0] cfg_rdata,
    output reg         cfg_cpl_valid, // one pulse per CfgRd or CfgWr: "generate a Cpl (no data) response" for writes, Cpl+data for reads

    // ---- Live status feeding Link Status (from your reset/training chain) ----
    input  wire        link_training_bit,   // e.g. link_trained
    input  wire [5:0]  link_width,          // e.g. 6'd4 for x4
    input  wire [3:0]  link_speed,          // encoded speed, e.g. 4'd6 for PCIe 6.0 (Gen6 encoding TBD by spec; passed through as-is)

    // ---- Outputs consumed elsewhere in the design ----
    output wire [31:0] bar_addr_0, bar_addr_1, bar_addr_2, bar_addr_3, bar_addr_4, bar_addr_5,
    output wire [2:0]  max_payload_size,     // Device Control MPS field
    output wire [2:0]  max_read_req_size,    // Device Control MRRS field
    output wire        mem_space_enable,     // Command register bit1
    output wire        bus_master_enable,    // Command register bit2

    // ---- MSI-X capability fields, exposed for msix_controller to read ----
    output wire        msix_enable,
    output wire        msix_func_mask
);

localparam PCIE_CAP_ID = 8'h10;
localparam MSIX_CAP_ID = 8'h11;

// ---- Type 0 header registers ----
reg [15:0] command_reg;      // only bits [1:0] implemented: mem space en, bus master en
reg [31:0] bar [0:5];
reg [7:0]  interrupt_line;

// ---- PCIe capability: Device Control ----
reg [2:0]  mps_reg, mrrs_reg;
reg [4:0]  dev_ctrl_enable_bits;

// ---- MSI-X capability: Message Control ----
reg msix_enable_reg, msix_func_mask_reg;

// BAR write mask: writing to a BAR only updates bits at/above the size
// boundary; the low bits are hardwired 0 so a size-probe (write all-1s,
// read back) reveals size, per standard PCI BAR sizing.
function [31:0] bar_wmask;
    input [31:0] size;
    begin
        bar_wmask = (size == 0) ? 32'h0 : ~(size - 32'h1);
    end
endfunction

wire [31:0] bar_mask [0:5];
assign bar_mask[0] = bar_wmask(BAR0_SIZE);
assign bar_mask[1] = bar_wmask(BAR1_SIZE);
assign bar_mask[2] = bar_wmask(BAR2_SIZE);
assign bar_mask[3] = bar_wmask(BAR3_SIZE);
assign bar_mask[4] = bar_wmask(BAR4_SIZE);
assign bar_mask[5] = bar_wmask(BAR5_SIZE);

integer i;
always @(posedge clk) begin
    if (!rst_n) begin
        command_reg          <= 16'h0;
        interrupt_line        <= 8'h0;
        mps_reg               <= 3'b000;
        mrrs_reg               <= 3'b000;
        dev_ctrl_enable_bits  <= 5'b0;
        msix_enable_reg       <= 1'b0;
        msix_func_mask_reg    <= 1'b0;
        cfg_cpl_valid         <= 1'b0;
        cfg_rdata             <= 32'h0;
        for (i = 0; i < 6; i = i + 1) bar[i] <= 32'h0;
    end else begin
        cfg_cpl_valid <= 1'b0; // default: pulse only

        // ---- CfgWr: update the addressed register, generate Cpl (no data) ----
        if (cfg_wr_en) begin
            cfg_cpl_valid <= 1'b1;
            case (cfg_addr[11:2])
                10'h001: command_reg <= {14'b0, cfg_wdata[1:0]};
                10'h004: bar[0] <= (cfg_wdata & bar_mask[0]) | (bar[0] & ~bar_mask[0]);
                10'h005: bar[1] <= (cfg_wdata & bar_mask[1]) | (bar[1] & ~bar_mask[1]);
                10'h006: bar[2] <= (cfg_wdata & bar_mask[2]) | (bar[2] & ~bar_mask[2]);
                10'h007: bar[3] <= (cfg_wdata & bar_mask[3]) | (bar[3] & ~bar_mask[3]);
                10'h008: bar[4] <= (cfg_wdata & bar_mask[4]) | (bar[4] & ~bar_mask[4]);
                10'h009: bar[5] <= (cfg_wdata & bar_mask[5]) | (bar[5] & ~bar_mask[5]);
                10'h00F: interrupt_line <= cfg_wdata[7:0];
                10'h012: begin // 0x48: Device Status(RO)/Device Control(RW)
                    mps_reg              <= cfg_wdata[7:5];
                    mrrs_reg             <= cfg_wdata[14:12];
                    dev_ctrl_enable_bits <= cfg_wdata[4:0];
                end
                10'h014: begin // 0x50: MSI-X Message Control RW bits only
                    msix_func_mask_reg <= cfg_wdata[30];
                    msix_enable_reg    <= cfg_wdata[31];
                end
                default: ; // reserved/RO addresses: write has no effect
            endcase
        end

        // ---- CfgRd: return the addressed register, generate Cpl+data ----
        if (cfg_rd_en) begin
            cfg_cpl_valid <= 1'b1;
            case (cfg_addr[11:2])
                10'h000: cfg_rdata <= {DEVICE_ID, VENDOR_ID};
                10'h001: cfg_rdata <= {16'h0, 14'b0, command_reg[1:0]};      // Status RO=0 (not modeled)
                10'h002: cfg_rdata <= {CLASS_CODE, REVISION_ID};
                10'h003: cfg_rdata <= {8'h0, 8'h00, 8'h00, 8'h00};           // BIST/HeaderType(Type0)/LatTmr/CLS, all RO 0
                10'h004: cfg_rdata <= bar[0];
                10'h005: cfg_rdata <= bar[1];
                10'h006: cfg_rdata <= bar[2];
                10'h007: cfg_rdata <= bar[3];
                10'h008: cfg_rdata <= bar[4];
                10'h009: cfg_rdata <= bar[5];
                10'h00A: cfg_rdata <= 32'h0;                                  // Cardbus CIS
                10'h00B: cfg_rdata <= {DEVICE_ID, VENDOR_ID};                 // Subsystem ID (reuse for sim)
                10'h00C: cfg_rdata <= 32'h0;                                  // Expansion ROM, not implemented
                10'h00D: cfg_rdata <= {24'h0, 8'h40};                         // Capabilities ptr -> 0x40
                10'h00E: cfg_rdata <= 32'h0;                                  // reserved
                10'h00F: cfg_rdata <= {16'h0, 8'h0, interrupt_line};          // Max Lat/Min Gnt=0, Int Pin=0(not modeled), Int Line
                10'h010: cfg_rdata <= {16'h0, 8'h50, PCIE_CAP_ID};            // 0x40: next=0x50, id=0x10; PCIe Capabilities RO=0
                10'h011: cfg_rdata <= 32'h0;                                  // 0x44: Device Capabilities, not modeled
                10'h012: cfg_rdata <= {16'h0, 1'b0, mrrs_reg, dev_ctrl_enable_bits[4], mps_reg, dev_ctrl_enable_bits[3:0]}; // 0x48: {DevStatus=0, DevControl}
                10'h013: cfg_rdata <= 32'h0;                                  // 0x4C: Link Capabilities, not modeled
                10'h014: cfg_rdata <= {msix_enable_reg, msix_func_mask_reg, MSIX_TABLE_SIZE_M1[10:0], 3'b0, 8'h00, MSIX_CAP_ID}; // 0x50
                10'h015: cfg_rdata <= 32'h0;                                  // 0x54: Table Offset/BIR - RO, not exposing a real BAR-relative offset yet (design pending BAR layout finalization)
                10'h016: cfg_rdata <= 32'h0;                                  // 0x58: PBA Offset/BIR - same caveat
                10'h017: cfg_rdata <= {20'h0, link_speed, link_width, 1'b0, link_training_bit}; // 0x5C: Link Status
                default: cfg_rdata <= 32'h0;
            endcase
        end
    end
end

assign bar_addr_0 = bar[0]; assign bar_addr_1 = bar[1]; assign bar_addr_2 = bar[2];
assign bar_addr_3 = bar[3]; assign bar_addr_4 = bar[4]; assign bar_addr_5 = bar[5];
assign max_payload_size  = mps_reg;
assign max_read_req_size = mrrs_reg;
assign mem_space_enable  = command_reg[0];
assign bus_master_enable = command_reg[1];
assign msix_enable       = msix_enable_reg;
assign msix_func_mask    = msix_func_mask_reg;

endmodule
