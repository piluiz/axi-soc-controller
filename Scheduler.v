/*
 * unit_scheduler -- issues queued instructions to one subordinate unit
 *
 * One instance per unit (ALU, Memory, I/O). Each instance owns a master_axi
 * and runs a two-state machine:
 *
 *   FINDTASK     Pick the oldest queue entry that is ready (see controller)
 *                and addressed to this unit, latch it, and start the AXI read
 *                or write on the master.
 *   PROCESSTASK  Wait for the AXI transfer to finish: the last read beat
 *                (RVALID && RREADY && RLAST) or the write response
 *                (BVALID && BREADY). In that cycle, raise `done` with the
 *                entry's buffer slot so the controller can mark it complete,
 *                and return to FINDTASK.
 *
 * Unit latency is modeled in the slave (slave_axi's LATENCY parameter), so a
 * task is complete exactly when its data or write response has arrived.
 *
 * The scheduler never writes the instruction buffer itself. It only reports
 * completion, which keeps the buffer to a single driver (required for
 * synthesis).
 */
`timescale 1 ns / 100 ps

module unit_scheduler #(
    parameter [1:0] UNIT = 2'd0,   // Unit code in instruction bits [13:12]
    parameter       NAME = "ALU"   // Label for simulation log messages
) (
    input clk,
    input reset,
    input dependency_test,            // Print task start/complete messages

    // View of the queue in age order: position 0 is the oldest entry
    input [16*30-1:0] entries,        // Instruction at each position
    input [15:0]      ready,          // Position may issue now
    input [3:0]       read_ptr,       // Buffer slot of position 0

    // Completion report to the controller
    output            done,
    output [3:0]      done_slot,

    // Scheduler state (one-hot)
    output q_findtask,
    output q_processtask,

    // AXI master interface (to this unit's slave)
    input        ARREADY, RVALID, RLAST, AWREADY, WREADY, BVALID,
    input  [7:0] RDATA,
    input  [9:0] BRESP,
    output       AWVALID, WLAST, WVALID, BREADY, ARVALID, RREADY,
    output [17:0] AWADDR,
    output [21:0] ARADDR,
    output [7:0]  WDATA,
    output [7:0]  ReadValue          // Most recent value read
);

// Master state (one-hot). Only the address-phase bits are used here.
wire qr_Idle, qr_ARValid, qr_ReadReady, qr_Read;
wire qw_Idle, qw_AWValid, qw_GetDataToWrite, qw_Write, qw_WaitForResponse;

localparam FINDTASK = 2'b01, PROCESSTASK = 2'b10;

reg [1:0]  state;
reg [3:0]  slot;         // Buffer slot of the task in progress
reg [29:0] task_entry;   // Copy of the instruction in progress

// The transfer for the task in progress has finished this cycle
wire read_complete  = RVALID && RREADY && RLAST;   // Last read beat accepted
wire write_complete = BVALID && BREADY;            // Write response accepted
wire transfer_done  = task_entry[11] ? write_complete : read_complete;

assign {q_processtask, q_findtask} = state;
assign done      = (state == PROCESSTASK) && transfer_done;
assign done_slot = slot;

// Requests to the master
reg       ReadRequest, WriteRequest, WriteDataValid, WriteLast;
reg [7:0] ReadAddress, WriteAddress, WriteValue;
reg [3:0] NumAddressesToRead;
reg [9:0] ReadTransactionID, WriteTransactionID;

// Priority encoder: the lowest (oldest) ready position addressed to this unit.
// Scanning from 15 down to 0 lets the oldest match win. The loop has a fixed
// bound, so it synthesizes to 16 copies of the comparison.
reg       found;
reg [3:0] pick;
integer k;

always @* begin
    found = 1'b0;
    pick  = 4'd0;
    for (k = 15; k >= 0; k = k - 1)
        if (ready[k] && entries[k*30 + 12 +: 2] == UNIT) begin
            found = 1'b1;
            pick  = k;
        end
end

wire [29:0] pick_entry = entries[pick*30 +: 30];

always @(posedge clk, posedge reset)
begin
    if (reset)
    begin
        state              <= FINDTASK;
        slot               <= 4'd0;
        task_entry         <= 30'd0;
        ReadRequest        <= 1'b0;
        WriteRequest       <= 1'b0;
        WriteDataValid     <= 1'b0;
        WriteLast          <= 1'b0;
        ReadAddress        <= 8'd0;
        NumAddressesToRead <= 4'd0;
        ReadTransactionID  <= 10'd0;
        WriteAddress       <= 8'd0;
        WriteTransactionID <= 10'd0;
        WriteValue         <= 8'd0;
    end
    else
        case (state)
            FINDTASK:
                if (found)
                begin
                    state      <= PROCESSTASK;
                    slot       <= read_ptr + pick;
                    task_entry <= pick_entry;

                    if (pick_entry[11] == 1'b0)    // Read
                    begin
                        ReadTransactionID  <= pick_entry[10:1];
                        ReadAddress        <= pick_entry[21:14];
                        NumAddressesToRead <= 4'd1;
                        ReadRequest        <= 1'b1;
                    end
                    else                           // Write
                    begin
                        WriteTransactionID <= pick_entry[10:1];
                        WriteAddress       <= pick_entry[21:14];
                        WriteValue         <= pick_entry[29:22];
                        WriteRequest       <= 1'b1;
                        WriteLast          <= 1'b1;
                        WriteDataValid     <= 1'b1;
                    end
`ifndef SYNTHESIS
                    if (dependency_test)
                        $display("%0s Task Started: R/W: %d, ID: %d, Instruction: %b",
                                 NAME, pick_entry[11], pick_entry[10:1], pick_entry);
`endif
                end

            PROCESSTASK:
            begin
                // Drop each request once the master has taken it
                if (qr_ARValid)
                    ReadRequest <= 1'b0;
                if (qw_AWValid)
                    WriteRequest <= 1'b0;

                if (transfer_done)
                begin
                    state <= FINDTASK;
`ifndef SYNTHESIS
                    if (dependency_test)
                        $display("%0s Task Complete: R/W: %d, ID: %d, Instruction: %b",
                                 NAME, task_entry[11], task_entry[10:1],
                                 {task_entry[29:1], 1'b1});
`endif
                end
            end

            default:
                state <= FINDTASK;
        endcase
end

master_axi master (
    .clk(clk),
    .reset(reset),
    .ARREADY(ARREADY),
    .RDATA(RDATA),
    .RVALID(RVALID),
    .AWREADY(AWREADY),
    .WREADY(WREADY),
    .BRESP(BRESP),
    .BVALID(BVALID),
    .AWADDR(AWADDR),
    .AWVALID(AWVALID),
    .WDATA(WDATA),
    .WLAST(WLAST),
    .WVALID(WVALID),
    .BREADY(BREADY),
    .ARADDR(ARADDR),
    .ARVALID(ARVALID),
    .RREADY(RREADY),
    .qr_Idle(qr_Idle),
    .qr_ARValid(qr_ARValid),
    .qr_ReadReady(qr_ReadReady),
    .qr_Read(qr_Read),
    .qw_Idle(qw_Idle),
    .qw_AWValid(qw_AWValid),
    .qw_GetDataToWrite(qw_GetDataToWrite),
    .qw_Write(qw_Write),
    .qw_WaitForResponse(qw_WaitForResponse),
    .tb_ReadRequest(ReadRequest),
    .tb_ReadAddress(ReadAddress),
    .tb_NumAddressesToRead(NumAddressesToRead),
    .tb_ReadTransactionID(ReadTransactionID),
    .tb_ReadValue(ReadValue),
    .tb_WriteRequest(WriteRequest),
    .tb_WriteAddress(WriteAddress),
    .tb_WriteTransactionID(WriteTransactionID),
    .tb_WriteDataValid(WriteDataValid),
    .tb_WriteValue(WriteValue),
    .tb_WriteLast(WriteLast),
    .RLAST(RLAST)
);

endmodule
