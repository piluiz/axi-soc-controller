/*
 * controller -- SoC controller with a dependency-aware instruction queue
 *
 * Accepts 30-bit instructions from the testbench (tb_NewInstruction, valid when
 * tb_IsNewInstruction is high) into a 16-entry circular buffer, and dispatches
 * them to three units (ALU, Memory, and I/O), each through a unit_scheduler
 * that drives its own master_axi.
 *
 * Scheduling:
 *   - When a unit is free, it takes the oldest unfinished instruction for that
 *     unit...
 *   - ...unless an older unfinished instruction, for any unit, has the same
 *     transaction ID.
 *   - Instructions with different IDs may run out of order and in parallel;
 *     instructions with the same ID always run in arrival order.
 *
 * Structure (synthesizable):
 *   - This module is the only writer of the instruction buffer. Schedulers
 *     report completion with a done pulse and slot number.
 *   - The "ready" check is a fixed 16 x 16 comparison of transaction IDs,
 *     computed once and shared by all three schedulers.
 *   - The head of the queue retires one completed entry per cycle.
 *
 * The FIFO_Q_* outputs tell the testbench when it can send another instruction.
 */

`timescale 1 ns / 100 ps

// Instruction format (tb_NewInstruction)
//   [29:22] Value (writes)
//   [21:14] Address
//   [13:12] Unit: ALU (0), Memory (1), I/O (2)
//   [11]    Read (0), Write (1)
//   [10:1]  Transaction ID
//   [0]     Done flag: ignored on input; completion is tracked in done_bits

module controller (
    input          clk,
    input          reset,

    // Instruction interface (from the testbench)
    input          tb_IsNewInstruction,  // tb_NewInstruction is valid
    input  [29:0]  tb_NewInstruction,    // See the instruction format above
    input          dependency_test,      // Print task start/complete messages (simulation)

    // Queue status (one-hot state) and empty flag
    output         FIFO_Q_NOTFULL,       // Ready to accept an instruction
    output         FIFO_Q_GOTVALUE,      // Instruction accepted at the last clock edge
    output         FIFO_Q_FULL,          // Queue full: hold the instruction
    output         FIFO_IsEmpty,         // Every instruction has completed and retired

    // Scheduler state per unit (one-hot)
    output         ALU_Q_FINDTASK,    ALU_Q_PROCESSTASK,
    output         MEMORY_Q_FINDTASK, MEMORY_Q_PROCESSTASK,
    output         IO_Q_FINDTASK,     IO_Q_PROCESSTASK,

    // ALU: AXI-style master port
    output         ALU_ARVALID,
    input          ALU_ARREADY,
    output [21:0]  ALU_ARADDR,           // {address[7:0], length[3:0], ID[9:0]}
    input          ALU_RVALID,
    output         ALU_RREADY,
    input  [7:0]   ALU_RDATA,
    input          ALU_RLAST,
    output         ALU_AWVALID,
    input          ALU_AWREADY,
    output [17:0]  ALU_AWADDR,           // {address[7:0], ID[9:0]}
    output         ALU_WVALID,
    input          ALU_WREADY,
    output [7:0]   ALU_WDATA,
    output         ALU_WLAST,
    input          ALU_BVALID,
    output         ALU_BREADY,
    input  [9:0]   ALU_BRESP,            // {status[1:0], overflow count[7:0]}; 01 = OK, 10 = WARN
    output [7:0]   ALU_ReadValue,        // Most recent value read

    // Memory: AXI-style master port
    output         Memory_ARVALID,
    input          Memory_ARREADY,
    output [21:0]  Memory_ARADDR,        // {address[7:0], length[3:0], ID[9:0]}
    input          Memory_RVALID,
    output         Memory_RREADY,
    input  [7:0]   Memory_RDATA,
    input          Memory_RLAST,
    output         Memory_AWVALID,
    input          Memory_AWREADY,
    output [17:0]  Memory_AWADDR,        // {address[7:0], ID[9:0]}
    output         Memory_WVALID,
    input          Memory_WREADY,
    output [7:0]   Memory_WDATA,
    output         Memory_WLAST,
    input          Memory_BVALID,
    output         Memory_BREADY,
    input  [9:0]   Memory_BRESP,         // {status[1:0], overflow count[7:0]}; 01 = OK, 10 = WARN
    output [7:0]   Memory_ReadValue,     // Most recent value read

    // I/O: AXI-style master port
    output         IO_ARVALID,
    input          IO_ARREADY,
    output [21:0]  IO_ARADDR,            // {address[7:0], length[3:0], ID[9:0]}
    input          IO_RVALID,
    output         IO_RREADY,
    input  [7:0]   IO_RDATA,
    input          IO_RLAST,
    output         IO_AWVALID,
    input          IO_AWREADY,
    output [17:0]  IO_AWADDR,            // {address[7:0], ID[9:0]}
    output         IO_WVALID,
    input          IO_WREADY,
    output [7:0]   IO_WDATA,
    output         IO_WLAST,
    input          IO_BVALID,
    output         IO_BREADY,
    input  [9:0]   IO_BRESP,             // {status[1:0], overflow count[7:0]}; 01 = OK, 10 = WARN
    output [7:0]   IO_ReadValue          // Most recent value read
);


// ---------------------------------------------------------------------------
// Instruction buffer: 16 entries, written only by this module
// ---------------------------------------------------------------------------

reg [29:0] buffer [0:15];
reg [15:0] done_bits;        // done_bits[s]: the entry in slot s has completed
reg [4:0]  read_pointer;     // (n+1)-bit pointers: the extra bit tells full from empty
reg [4:0]  write_pointer;

wire [4:0] count = write_pointer - read_pointer;   // Entries in the queue (0..16)
wire [3:0] head  = read_pointer[3:0];

assign FIFO_IsEmpty = (count == 5'd0);

localparam FIFO_NOTFULL = 3'b001, FIFO_GOTVALUE = 3'b010, FIFO_FULL = 3'b100;
reg [2:0] FIFO_State;
assign {FIFO_Q_FULL, FIFO_Q_GOTVALUE, FIFO_Q_NOTFULL} = FIFO_State;

// Completion reports from the schedulers
wire       alu_done, mem_done, io_done;
wire [3:0] alu_done_slot, mem_done_slot, io_done_slot;

always @ (posedge clk, posedge reset)
begin
    if (reset)
    begin
        read_pointer  <= 5'd0;
        write_pointer <= 5'd0;
        done_bits     <= 16'd0;
        FIFO_State    <= FIFO_NOTFULL;
    end
    else
    begin
        // Mark finished tasks
        if (alu_done) done_bits[alu_done_slot] <= 1'b1;
        if (mem_done) done_bits[mem_done_slot] <= 1'b1;
        if (io_done)  done_bits[io_done_slot]  <= 1'b1;

        // Retire the oldest entry once it has completed
        if (count != 5'd0 && done_bits[head])
            read_pointer <= read_pointer + 5'd1;

        // Accept new instructions from the testbench
        case (FIFO_State)
            FIFO_NOTFULL:
                if (tb_IsNewInstruction && count < 5'd16)
                begin
                    buffer[write_pointer[3:0]]    <= {tb_NewInstruction[29:1], 1'b0};
                    done_bits[write_pointer[3:0]] <= 1'b0;
                    write_pointer                 <= write_pointer + 5'd1;
                    FIFO_State                    <= FIFO_GOTVALUE;
                end

            FIFO_GOTVALUE:
                FIFO_State <= (count < 5'd16) ? FIFO_NOTFULL : FIFO_FULL;

            FIFO_FULL:
                if (count < 5'd16)
                    FIFO_State <= FIFO_NOTFULL;

            default:
                FIFO_State <= FIFO_NOTFULL;
        endcase
    end
end


// ---------------------------------------------------------------------------
// Ready check, shared by all schedulers
//
// Positions are numbered by age: position 0 is the oldest entry (slot `head`).
// Position k is ready if it holds an unfinished instruction and no older
// unfinished instruction has the same transaction ID. Both loops have fixed
// bounds, so this synthesizes to a 16 x 16 grid of ID comparators.
// ---------------------------------------------------------------------------

reg [16*30-1:0] entries;      // Instruction at each position
reg [15:0]      unfinished;   // Position holds an instruction that hasn't completed
reg [15:0]      ready;        // Position may issue now
integer p, q;

always @*
begin
    for (p = 0; p < 16; p = p + 1)
    begin
        entries[p*30 +: 30] = buffer[(head + p) & 4'hF];
        unfinished[p]       = (p < count) && !done_bits[(head + p) & 4'hF];
    end

    for (p = 0; p < 16; p = p + 1)
    begin
        ready[p] = unfinished[p];
        for (q = 0; q < 16; q = q + 1)
            if (q < p && unfinished[q] && entries[q*30 + 1 +: 10] == entries[p*30 + 1 +: 10])
                ready[p] = 1'b0;
    end
end


// ---------------------------------------------------------------------------
// One scheduler per unit
// ---------------------------------------------------------------------------

unit_scheduler #(.UNIT(2'd0), .NAME("ALU")) ALU_Scheduler (
    .clk(clk), .reset(reset), .dependency_test(dependency_test),
    .entries(entries), .ready(ready), .read_ptr(head),
    .done(alu_done), .done_slot(alu_done_slot),
    .q_findtask(ALU_Q_FINDTASK), .q_processtask(ALU_Q_PROCESSTASK),
    .ARREADY(ALU_ARREADY), .RVALID(ALU_RVALID), .RLAST(ALU_RLAST), .AWREADY(ALU_AWREADY),
    .WREADY(ALU_WREADY), .BVALID(ALU_BVALID), .RDATA(ALU_RDATA), .BRESP(ALU_BRESP),
    .AWVALID(ALU_AWVALID), .WLAST(ALU_WLAST), .WVALID(ALU_WVALID), .BREADY(ALU_BREADY),
    .ARVALID(ALU_ARVALID), .RREADY(ALU_RREADY), .AWADDR(ALU_AWADDR), .ARADDR(ALU_ARADDR),
    .WDATA(ALU_WDATA), .ReadValue(ALU_ReadValue)
);

unit_scheduler #(.UNIT(2'd1), .NAME("Memory")) Memory_Scheduler (
    .clk(clk), .reset(reset), .dependency_test(dependency_test),
    .entries(entries), .ready(ready), .read_ptr(head),
    .done(mem_done), .done_slot(mem_done_slot),
    .q_findtask(MEMORY_Q_FINDTASK), .q_processtask(MEMORY_Q_PROCESSTASK),
    .ARREADY(Memory_ARREADY), .RVALID(Memory_RVALID), .RLAST(Memory_RLAST),
    .AWREADY(Memory_AWREADY), .WREADY(Memory_WREADY), .BVALID(Memory_BVALID),
    .RDATA(Memory_RDATA), .BRESP(Memory_BRESP),
    .AWVALID(Memory_AWVALID), .WLAST(Memory_WLAST), .WVALID(Memory_WVALID),
    .BREADY(Memory_BREADY), .ARVALID(Memory_ARVALID), .RREADY(Memory_RREADY),
    .AWADDR(Memory_AWADDR), .ARADDR(Memory_ARADDR), .WDATA(Memory_WDATA),
    .ReadValue(Memory_ReadValue)
);

unit_scheduler #(.UNIT(2'd2), .NAME("IO")) IO_Scheduler (
    .clk(clk), .reset(reset), .dependency_test(dependency_test),
    .entries(entries), .ready(ready), .read_ptr(head),
    .done(io_done), .done_slot(io_done_slot),
    .q_findtask(IO_Q_FINDTASK), .q_processtask(IO_Q_PROCESSTASK),
    .ARREADY(IO_ARREADY), .RVALID(IO_RVALID), .RLAST(IO_RLAST), .AWREADY(IO_AWREADY),
    .WREADY(IO_WREADY), .BVALID(IO_BVALID), .RDATA(IO_RDATA), .BRESP(IO_BRESP),
    .AWVALID(IO_AWVALID), .WLAST(IO_WLAST), .WVALID(IO_WVALID), .BREADY(IO_BREADY),
    .ARVALID(IO_ARVALID), .RREADY(IO_RREADY), .AWADDR(IO_AWADDR), .ARADDR(IO_ARADDR),
    .WDATA(IO_WDATA), .ReadValue(IO_ReadValue)
);

endmodule
