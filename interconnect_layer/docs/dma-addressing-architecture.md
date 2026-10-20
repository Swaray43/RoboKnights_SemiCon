# DMA addressing architecture — interconnect layer

**Author:** Aditya (interconnect layer) — for review by Swaray (link layer) and Kshitij (protocol-to-PHY / config space)
**Scope:** How outbound AXI4-Stream writes (SmartNIC path, ML path) get a real PCIe destination address, and how the host tells the device where to write.

---

## 1. Background

Both use cases the interconnect layer serves are **one-directional device-to-host writes** — the FPGA never reads host memory:

- **SmartNIC path** — incoming network packets written into a host server buffer.
- **ML path** — inference output (weights/pixels) written into a host buffer.

Because of this, the interconnect layer only implements two AXI4-Stream ingest ports (no AXI4 memory-mapped AW/W/B, no AR/R, no read-completion path). This doc covers how those two streams get a valid PCIe host address to write to.

---

## 2. What each layer owns

| Concern | Owner | Why |
|---|---|---|
| Two AXI4-Stream ports, crossbar, round-robin arbiter, per-channel offset tracking, CDC FIFO | **Aditya** (interconnect) | Fabric-facing, clock-domain boundary |
| BAR sizing/decode, config-space register file (`base_addr_ch0/1`), MSI-X vector generation | **Kshitij** (protocol-to-PHY / sideband) | Owns config space per project split |
| FLIT framing (256B PCIe 6.0 Flits), zero-padding on partial-length termination, TLP header generation, final address arithmetic, IOMMU pass-through if used | **Swaray** (link layer) | Owns TLP formation/FLIT framing per project split |

**Key principle:** the interconnect layer never handles a full 64-bit host address. It only ever produces a small per-session running offset. Base-address lookup and final address arithmetic happen downstream, in Swaray's block, reading a value Kshitij's block stored.

---

## 3. How the host tells the device where to write

There is no PCIe-wide address discovery involved — the host driver is the only party that ever knows the real destination address, and it hands it to the device the same way it configures any other device register.

1. **PCIe enumeration (boot, once)** — BIOS/OS sizes the device's BARs and assigns each a base address in host space. Standard PCIe Hard IP behavior; nothing custom here.
2. **Host driver allocates a DMA buffer** in its own memory (e.g. `dma_alloc_coherent` or equivalent) and now knows that buffer's real physical address.
3. **Host writes that address into a device register**, via an ordinary memory write landing inside the device's BAR range. The Hard IP decodes it to (BAR hit, offset X) and hands Kshitij's config-space logic the offset + data.
4. **Kshitij's decode logic** maps the fixed offset to a specific register and latches the data there.

The offset (e.g. `0x20`) is fixed at design time and only used to select *which register* — it is never part of any outgoing PCIe address and is not seen again after step 4.

### Register map (per channel)

| Register | BAR0 offset | Width | Written by | Purpose |
|---|---|---|---|---|
| `base_addr_ch0` | `0x20` | 64-bit | Host driver | SmartNIC session's destination base address in host memory |
| `base_addr_ch1` | `0x28` | 64-bit | Host driver | ML session's destination base address in host memory |

Two independent registers, one per stream, since the two use cases write to different host buffers.

---

## 4. Address computation

```
real_addr_ch[N] = base_addr_ch[N] + session_offset_ch[N]
```

- `base_addr_ch[N]` — the 64-bit value the host wrote into the channel's register (step 3/4 above). Static for the duration of one session.
- `session_offset_ch[N]` — a running counter, local to the interconnect layer, tracking position within the current session. Starts at 0 when a session begins, increments by the beat size (64 bytes) on every accepted AXI4-Stream beat for that channel.

**Only `real_addr_ch[N]` is a full 64-bit host address, and it is never computed inside the interconnect layer.** The addition is done downstream by Swaray's TLP-generation logic, which reads `base_addr_ch[N]` from Kshitij's register file and combines it with the offset the interconnect layer supplies per beat.

### Why not compute the full address here

- Keeps the crossbar/arbiter/CDC FIFO datapath narrow (small offset field, not 64 bits).
- Matches how AW/AWADDR works in memory-mapped AXI4 — one base value per burst, not a recomputed address every beat.
- Avoids duplicating base-address storage in two places (interconnect layer and config-space block).

---

## 5. TUSER field definition (per beat)

| Field | Width (indicative) | Meaning |
|---|---|---|
| `stream_id` | 1 bit | Which channel (0 = SmartNIC, 1 = ML). May be implicit from which physical port a beat arrived on, depending on where the two streams merge onto one internal bus. |
| `session_offset` | ~16–24 bits (sized to max session/slice size — not 64 bits) | Running byte position within the current session, per channel. Increments by 64 (beat size) per accepted beat. |
| `TLAST` | 1 bit | Marks the final beat of the current session/packet. **Not in the original bullet list — confirmed necessary** so downstream FLIT logic knows when to flush and zero-pad a partial Flit. |

**Bit layout on the wire (Phase 2, fixed):** `m_tuser[25:0] = {tlast_flag[25], session_offset[24:1], stream_id[0]}`,
which is the link layer's native TUSER, so `interconnect_top.m_tuser` connects straight to
`link_phy_bridge_top.s_axis_tuser` with no remap. `tlast_flag` is a copy of `m_tlast` (which is still driven).
The FIFO stays 602 bits wide; the extra bit is a wire, not a register.

`addr64` as originally scoped (a raw 64-bit address carried per beat) **is dropped** — replaced by the base-plus-offset split above.

Open: whether `attr`, `TC`, `reqtype` (from the original interface contract) are still needed per-beat, or can be fixed per channel / omitted since `reqtype` is always MWr for both streams. Worth confirming with Swaray rather than assuming.

---

## 6. Session lifecycle

```
[Host writes new base_addr_ch[N]]
        │
        ▼
[Kshitij's block latches base_addr_ch[N]]
        │
        ▼
[??? — some signal tells channel N's fabric logic
       it's safe to start a new session]
        │
        ▼
[session_offset_ch[N] resets to 0]
        │
        ▼
[Streaming begins — session_offset increments per beat]
        │
        ▼
[TLAST asserted — session ends]
        │
        ▼
[Swaray's block flushes/pads partial Flit if needed,
 fires MSI-X on the channel's dedicated vector]
        │
        ▼
[Host driver services the interrupt, reads data,
 optionally writes a new base_addr_ch[N] to start the next session]
```

### MSI-X

- Device → host only. Not used to carry any address information.
- Each channel fires on its **own dedicated MSI-X vector**. The host's interrupt handler identifies the channel by which vector fired — not by any data content.

---

## 7. Buffer partitioning assumption (flag to problem owner)

Both streams currently write into **separate host buffers** (separate `base_addr` per channel — see §3), not a single shared buffer with a fixed 50/50 split as earlier discussed. This is simpler and avoids the equal-partition sizing question entirely, since each channel's host buffer is independently sized and addressed by the driver. **Confirm this is the intended model** before finalizing register count/layout — if a single shared buffer with fixed slices is actually wanted instead, the register map in §3 collapses to one `base_addr` register plus compile-time slice offsets.

---

## 8. Open questions — need confirmation before implementation

1. **Session-armed handshake.** Nothing currently tells the interconnect layer's fabric logic "a fresh `base_addr` has landed, you're clear to start a new session." Without this there's a race: streaming could start against a stale/previous address. Needs either a dedicated "session armed" bit (host-writable, cleared automatically or by fabric logic once consumed) or an equivalent signal — owner TBD (likely Kshitij, since it's adjacent to the register file).
2. **Session/slice size bound.** `session_offset` needs an upper bound so one channel's session can never run into another channel's buffer or off the end of its own. Is `SLICE_SIZE`/max session length a compile-time constant, or does the host communicate it at runtime alongside `base_addr`?
3. **Overflow policy.** If a session would exceed its bound: wrap (ring buffer), stall (backpressure until host drains), or drop-and-flag? Each has different host-driver implications (see draft options in prior discussion).
4. **TUSER field set.** Confirm final field list — is `stream_id`/`session_offset`/`TLAST` sufficient, or do `attr`/`TC`/`reqtype` still need to travel per-beat?
5. **FLIT-alignment vs beat-alignment of `session_offset`.** Currently spec'd to increment per 64B beat. An alternative (from a candidate design) increments only per completed 256B Flit, with a separate intra-Flit counter. Needs to be agreed with Swaray since it affects exactly what value his block reads for address computation.
6. **Backpressure signal.** Per earlier discussion: AWREADY-equivalent backpressure (now TREADY on both stream ports) should likely be gated by one **shared** downstream-full signal covering both channels' contribution to the same downstream path, not two independent ones — confirm this framing before building.

---

## 9. Summary of what changed from the original scope

- Dropped: AXI4 memory-mapped slave write (AW/W/B), AXI4 read (AR/R), read-completion CDC FIFO, per-beat 64-bit `addr64` in TUSER.
- Added: per-channel `base_addr` register (owned by Kshitij, written by host), per-channel `session_offset` tracked in the interconnect layer, `TLAST` in TUSER, per-channel dedicated MSI-X vector.
- Unchanged: two AXI4-Stream ingest ports, crossbar + round-robin arbiter with starvation guard and weights, single write-path CDC FIFO, registered (not combinational) arbitration for timing closure.
