// ECC Frame Packer - RTL
// Front-end glue between the UART RX byte stream and the SEC-DED ECC
// accelerator. The accelerator consumes 13-bit frames through a
// first-word fall-through FIFO interface; the UART delivers bytes.
//
// Frame transport convention (two UART bytes per ECC frame):
//   byte 0  = frame[7:0]   the 8-bit payload field
//   byte 1  = frame[12:8]  in bits [4:0]: {P, C3, C2, C1, C0}
//             bits [7:5] are ignored (don't care on the wire)
//
// A UART framing error aborts a half-assembled frame (the pending
// payload byte is dropped and the packer re-synchronizes on the next
// byte). If the internal FIFO is full, a completed frame is dropped -
// at UART rates versus the 100 MHz fabric this cannot happen unless
// software stops draining the accelerator entirely.
//
// FIFO: 8 deep, register-based, first-word fall-through (head word is
// combinationally visible while fifo_empty = 0; read_en pops).

module ecc_frame_packer (
  input  logic        clk,
  input  logic        resetn,

  // UART RX stream tap
  input  logic [7:0]  byte_data,
  input  logic        byte_valid,    // Single-cycle pulse per byte
  input  logic        byte_ferr,     // Single-cycle pulse on framing error

  // FWFT FIFO read interface (consumed by ecc_accel)
  output logic        fifo_empty,
  input  logic        fifo_read_en,
  output logic [12:0] fifo_data
);

  localparam int unsigned DEPTH   = 8;
  localparam int unsigned PTR_W   = $clog2(DEPTH);

  // ----------------------------------------------------------------
  // Byte pairing: phase 0 collects the payload byte, phase 1 completes
  // the frame with the check/parity byte.
  // ----------------------------------------------------------------
  logic       phase;        // 0 = expect payload byte, 1 = expect ECC byte
  logic [7:0] payload_hold;

  logic        push;
  logic [12:0] push_frame;

  assign push_frame = {byte_data[4:0], payload_hold};

  // ----------------------------------------------------------------
  // FWFT FIFO storage and pointers (extra MSB distinguishes full/empty)
  // ----------------------------------------------------------------
  logic [12:0]      mem [0:DEPTH-1];
  logic [PTR_W:0]   wr_ptr, rd_ptr;

  logic fifo_full;
  assign fifo_empty = (wr_ptr == rd_ptr);
  assign fifo_full  = (wr_ptr[PTR_W-1:0] == rd_ptr[PTR_W-1:0]) &&
                      (wr_ptr[PTR_W]     != rd_ptr[PTR_W]);

  assign fifo_data = mem[rd_ptr[PTR_W-1:0]];

  assign push = byte_valid && (phase == 1'b1) && !fifo_full;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      phase        <= 1'b0;
      payload_hold <= '0;
      wr_ptr       <= '0;
      rd_ptr       <= '0;
    end
    else begin
      // Framing error: drop any half-assembled frame and re-sync
      if (byte_ferr)
        phase <= 1'b0;
      else if (byte_valid) begin
        if (phase == 1'b0) begin
          payload_hold <= byte_data;
          phase        <= 1'b1;
        end
        else
          phase <= 1'b0;     // Frame completes (pushed below if room)
      end

      if (push)
        wr_ptr <= wr_ptr + 1'b1;

      if (fifo_read_en && !fifo_empty)
        rd_ptr <= rd_ptr + 1'b1;
    end
  end

  // FIFO memory write (no reset: plain register array, LUTRAM-friendly)
  always_ff @(posedge clk) begin
    if (push)
      mem[wr_ptr[PTR_W-1:0]] <= push_frame;
  end

  // Sink for the intentionally ignored upper bits of the ECC byte
  wire _unused_bits;
  assign _unused_bits = &{1'b0, byte_data[7:5]};

endmodule
