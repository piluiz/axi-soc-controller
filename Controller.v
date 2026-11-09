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

module controller(
    clk, reset, tb_IsNewInstruction, tb_NewInstruction, ALU_AWVALID, ALU_WLAST, ALU_WVALID,
    ALU_BREADY, ALU_ARVALID, ALU_RREADY, dependency_test, ALU_qr_Idle, ALU_qr_ARValid,
    ALU_qr_ReadReady, ALU_qr_Read, ALU_qw_Idle, ALU_qw_AWValid, ALU_qw_GetDataToWrite, ALU_qw_Write,
    ALU_qw_WaitForResponse, ALU_AWADDR, ALU_ARADDR, ALU_WDATA, ALU_tb_ReadValue, Memory_AWVALID,
    Memory_WLAST, Memory_WVALID, Memory_BREADY, Memory_ARVALID, Memory_RREADY, Memory_qr_Idle,
    Memory_qr_ARValid, Memory_qr_ReadReady, Memory_qr_Read, Memory_qw_Idle, Memory_qw_AWValid,
    Memory_qw_GetDataToWrite, Memory_qw_Write, Memory_qw_WaitForResponse, Memory_AWADDR,
    Memory_ARADDR, Memory_WDATA, Memory_tb_ReadValue, IO_AWVALID, IO_WLAST, IO_WVALID, IO_BREADY,
    IO_ARVALID, IO_RREADY, IO_qr_Idle, IO_qr_ARValid, IO_qr_ReadReady, IO_qr_Read, IO_qw_Idle,
    IO_qw_AWValid, IO_qw_GetDataToWrite, IO_qw_Write, IO_qw_WaitForResponse, IO_AWADDR, IO_ARADDR,
    IO_WDATA, IO_tb_ReadValue, ALU_Q_FINDTASK, ALU_Q_PROCESSTASK, FIFO_Q_FULL, FIFO_Q_GOTVALUE,
    FIFO_Q_NOTFULL, MEMORY_Q_FINDTASK, MEMORY_Q_PROCESSTASK, IO_Q_FINDTASK, IO_Q_PROCESSTASK,
    ALU_ARREADY, ALU_RVALID, ALU_AWREADY, ALU_WREADY, ALU_BRESP, ALU_BVALID, ALU_tb_LastWrite,
    ALU_RLAST, ALU_RDATA, ALU_tb_WriteValue, ALU_tb_WriteAddress, ALU_tb_WriteTransactionID,
    Memory_ARREADY, Memory_RVALID, Memory_AWREADY, Memory_WREADY, Memory_BRESP, Memory_BVALID,
    Memory_tb_LastWrite, Memory_RLAST, Memory_RDATA, Memory_tb_WriteValue, Memory_tb_WriteAddress,
    Memory_tb_WriteTransactionID, IO_ARREADY, IO_RVALID, IO_AWREADY, IO_WREADY, IO_BRESP, IO_BVALID,
    IO_tb_LastWrite, IO_RLAST, IO_RDATA, IO_tb_WriteValue, IO_tb_WriteAddress,
    IO_tb_WriteTransactionID, FIFO_IsEmpty
);

input clk, reset, tb_IsNewInstruction, dependency_test;

// Instruction format
// 8-bits[29:22]: Value
// 8-bits[21:14]: Address
// 2-bits[13:12]: ALU (0), Memory (1), I/O (2)
// 1-bit[11]: Read (0), Write (1)
// 10-bits[10:1]: Transaction ID
// 1-bit[0]: Done? (ignored on input; completion is tracked in done_bits)
input [29:0] tb_NewInstruction;

// Slave-side monitor signals. These are part of the testbench wiring but are
// not used by the controller.
input       ALU_tb_LastWrite, Memory_tb_LastWrite, IO_tb_LastWrite;
input [7:0] ALU_tb_WriteValue, Memory_tb_WriteValue, IO_tb_WriteValue;
input [7:0] ALU_tb_WriteAddress, Memory_tb_WriteAddress, IO_tb_WriteAddress;
input [9:0] ALU_tb_WriteTransactionID, Memory_tb_WriteTransactionID, IO_tb_WriteTransactionID;

// ALU master interface
input        ALU_ARREADY, ALU_RVALID, ALU_AWREADY, ALU_WREADY, ALU_BVALID, ALU_RLAST;
input  [7:0] ALU_RDATA;
input  [9:0] ALU_BRESP;   // {status[1:0], overflow count[7:0]}; status 01 = OK, 10 = WARN
output       ALU_AWVALID, ALU_WLAST, ALU_WVALID, ALU_BREADY, ALU_ARVALID, ALU_RREADY;
output       ALU_qr_Idle, ALU_qr_ARValid, ALU_qr_ReadReady, ALU_qr_Read, ALU_qw_Idle,
             ALU_qw_AWValid, ALU_qw_GetDataToWrite, ALU_qw_Write, ALU_qw_WaitForResponse;
output [17:0] ALU_AWADDR;   // 8-bit address, 10-bit transaction ID
output [21:0] ALU_ARADDR;   // 8-bit address, 4-bit length, 10-bit transaction ID
output [7:0]  ALU_WDATA;
output [7:0]  ALU_tb_ReadValue;

// Memory master interface
input        Memory_ARREADY, Memory_RVALID, Memory_AWREADY, Memory_WREADY, Memory_BVALID,
             Memory_RLAST;
input  [7:0] Memory_RDATA;
input  [9:0] Memory_BRESP;
output       Memory_AWVALID, Memory_WLAST, Memory_WVALID, Memory_BREADY, Memory_ARVALID,
             Memory_RREADY;
output       Memory_qr_Idle, Memory_qr_ARValid, Memory_qr_ReadReady, Memory_qr_Read,
             Memory_qw_Idle, Memory_qw_AWValid, Memory_qw_GetDataToWrite, Memory_qw_Write,
             Memory_qw_WaitForResponse;
output [17:0] Memory_AWADDR;
output [21:0] Memory_ARADDR;
output [7:0]  Memory_WDATA;
output [7:0]  Memory_tb_ReadValue;

// I/O master interface
input        IO_ARREADY, IO_RVALID, IO_AWREADY, IO_WREADY, IO_BVALID, IO_RLAST;
input  [7:0] IO_RDATA;
input  [9:0] IO_BRESP;
output       IO_AWVALID, IO_WLAST, IO_WVALID, IO_BREADY, IO_ARVALID, IO_RREADY;
output       IO_qr_Idle, IO_qr_ARValid, IO_qr_ReadReady, IO_qr_Read, IO_qw_Idle,
             IO_qw_AWValid, IO_qw_GetDataToWrite, IO_qw_Write, IO_qw_WaitForResponse;
output [17:0] IO_AWADDR;
output [21:0] IO_ARADDR;
output [7:0]  IO_WDATA;
output [7:0]  IO_tb_ReadValue;

// Queue and scheduler state (one-hot)
output FIFO_Q_NOTFULL, FIFO_Q_GOTVALUE, FIFO_Q_FULL;
output ALU_Q_FINDTASK, ALU_Q_PROCESSTASK;
output MEMORY_Q_FINDTASK, MEMORY_Q_PROCESSTASK;
output IO_Q_FINDTASK, IO_Q_PROCESSTASK;
output FIFO_IsEmpty;


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

unit_scheduler #(.UNIT(2'd0), .LATENCY(9), .NAME("ALU")) ALU_Scheduler (
    .clk(clk), .reset(reset), .dependency_test(dependency_test),
    .entries(entries), .ready(ready), .read_ptr(head),
    .done(alu_done), .done_slot(alu_done_slot),
    .q_findtask(ALU_Q_FINDTASK), .q_processtask(ALU_Q_PROCESSTASK),
    .ARREADY(ALU_ARREADY), .RVALID(ALU_RVALID), .RLAST(ALU_RLAST), .AWREADY(ALU_AWREADY),
    .WREADY(ALU_WREADY), .BVALID(ALU_BVALID), .RDATA(ALU_RDATA), .BRESP(ALU_BRESP),
    .AWVALID(ALU_AWVALID), .WLAST(ALU_WLAST), .WVALID(ALU_WVALID), .BREADY(ALU_BREADY),
    .ARVALID(ALU_ARVALID), .RREADY(ALU_RREADY), .AWADDR(ALU_AWADDR), .ARADDR(ALU_ARADDR),
    .WDATA(ALU_WDATA), .tb_ReadValue(ALU_tb_ReadValue),
    .qr_Idle(ALU_qr_Idle), .qr_ARValid(ALU_qr_ARValid), .qr_ReadReady(ALU_qr_ReadReady),
    .qr_Read(ALU_qr_Read), .qw_Idle(ALU_qw_Idle), .qw_AWValid(ALU_qw_AWValid),
    .qw_GetDataToWrite(ALU_qw_GetDataToWrite), .qw_Write(ALU_qw_Write),
    .qw_WaitForResponse(ALU_qw_WaitForResponse)
);

unit_scheduler #(.UNIT(2'd1), .LATENCY(19), .NAME("Memory")) Memory_Scheduler (
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
    .tb_ReadValue(Memory_tb_ReadValue),
    .qr_Idle(Memory_qr_Idle), .qr_ARValid(Memory_qr_ARValid),
    .qr_ReadReady(Memory_qr_ReadReady), .qr_Read(Memory_qr_Read),
    .qw_Idle(Memory_qw_Idle), .qw_AWValid(Memory_qw_AWValid),
    .qw_GetDataToWrite(Memory_qw_GetDataToWrite), .qw_Write(Memory_qw_Write),
    .qw_WaitForResponse(Memory_qw_WaitForResponse)
);

unit_scheduler #(.UNIT(2'd2), .LATENCY(29), .NAME("IO")) IO_Scheduler (
    .clk(clk), .reset(reset), .dependency_test(dependency_test),
    .entries(entries), .ready(ready), .read_ptr(head),
    .done(io_done), .done_slot(io_done_slot),
    .q_findtask(IO_Q_FINDTASK), .q_processtask(IO_Q_PROCESSTASK),
    .ARREADY(IO_ARREADY), .RVALID(IO_RVALID), .RLAST(IO_RLAST), .AWREADY(IO_AWREADY),
    .WREADY(IO_WREADY), .BVALID(IO_BVALID), .RDATA(IO_RDATA), .BRESP(IO_BRESP),
    .AWVALID(IO_AWVALID), .WLAST(IO_WLAST), .WVALID(IO_WVALID), .BREADY(IO_BREADY),
    .ARVALID(IO_ARVALID), .RREADY(IO_RREADY), .AWADDR(IO_AWADDR), .ARADDR(IO_ARADDR),
    .WDATA(IO_WDATA), .tb_ReadValue(IO_tb_ReadValue),
    .qr_Idle(IO_qr_Idle), .qr_ARValid(IO_qr_ARValid), .qr_ReadReady(IO_qr_ReadReady),
    .qr_Read(IO_qr_Read), .qw_Idle(IO_qw_Idle), .qw_AWValid(IO_qw_AWValid),
    .qw_GetDataToWrite(IO_qw_GetDataToWrite), .qw_Write(IO_qw_Write),
    .qw_WaitForResponse(IO_qw_WaitForResponse)
);

endmodule
