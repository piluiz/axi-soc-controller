/*
 * controller -- SoC controller with a dependency-aware instruction queue
 *
 * Accepts 30-bit instructions from the testbench (tb_NewInstruction, valid when
 * tb_IsNewInstruction is high) into a 16-entry circular buffer, and dispatches
 * them to three units (ALU, Memory, and I/O), each through its own master_axi.
 *
 * Scheduling:
 *   - When a unit is free, it takes the oldest unfinished instruction for that
 *     unit...
 *   - ...unless an older unfinished instruction, for any unit, has the same
 *     transaction ID.
 *   - Instructions with different IDs may run out of order and in parallel;
 *     instructions with the same ID always run in arrival order.
 *
 * Each unit is a slave_axi. Its scheduler holds a task for a fixed number of
 * cycles (ALU ~9, Memory ~19, I/O ~29) to model different latencies, then marks
 * the buffer entry done. The read pointer advances past done entries at the
 * head of the queue.
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
input ALU_tb_LastWrite, Memory_tb_LastWrite, IO_tb_LastWrite;

// 8-bits[29:22]: Value
// 8-bits[21:14]: Address
// 2-bits[13:12]: ALU (0), Memory (1), I/O (2)
// 1-bit[11]: Read (0), Write (1)
// 10-bits[10:1]: Transaction ID
// 1-bit[0]: Done? (0 = No, 1 = Yes)
input [29:0] tb_NewInstruction;

input [7:0] ALU_tb_WriteValue;
input [7:0] ALU_tb_WriteAddress;
input [9:0] ALU_tb_WriteTransactionID;

input [7:0] Memory_tb_WriteValue;
input [7:0] Memory_tb_WriteAddress;
input [9:0] Memory_tb_WriteTransactionID;

input [7:0] IO_tb_WriteValue;
input [7:0] IO_tb_WriteAddress;
input [9:0] IO_tb_WriteTransactionID;

// ALU Master Inputs
input ALU_ARREADY, ALU_RVALID, ALU_AWREADY, ALU_WREADY, ALU_BVALID, ALU_RLAST;
reg ALU_ReadRequest, ALU_WriteRequest, ALU_WriteDataValid, ALU_WriteLast;
input [7:0] ALU_RDATA;  // 8-bit data values
input [9:0] ALU_BRESP;  // {status[1:0], overflow count[7:0]}; status 01 = OK, 10 = WARN
reg [7:0] ALU_ReadAddress;
reg [3:0] ALU_NumAddressesToRead;
reg [9:0] ALU_ReadTransactionID;
reg [7:0] ALU_WriteAddress;   // 8-bit address
reg [9:0] ALU_WriteTransactionID;
reg [7:0] ALU_WriteValue;

// ALU Master Outputs
output ALU_AWVALID, ALU_WLAST, ALU_WVALID, ALU_BREADY, ALU_ARVALID, ALU_RREADY;
output ALU_qr_Idle, ALU_qr_ARValid, ALU_qr_ReadReady, ALU_qr_Read, ALU_qw_Idle, ALU_qw_AWValid,
       ALU_qw_GetDataToWrite, ALU_qw_Write, ALU_qw_WaitForResponse;
output [17:0] ALU_AWADDR;   // 8-bit address, 10 transaction id bits
output [21:0] ALU_ARADDR;  // 8 bit address, 4 control bits, 10 bits transaction ID
output [7:0] ALU_WDATA;
output [7:0] ALU_tb_ReadValue;

master_axi ALU_Master(
    .clk(clk),
    .reset(reset),
    .ARREADY(ALU_ARREADY),
    .RDATA(ALU_RDATA),
    .RVALID(ALU_RVALID),
    .AWREADY(ALU_AWREADY),
    .WREADY(ALU_WREADY),
    .BRESP(ALU_BRESP),
    .BVALID(ALU_BVALID),
    .AWADDR(ALU_AWADDR),
    .AWVALID(ALU_AWVALID),
    .WDATA(ALU_WDATA),
    .WLAST(ALU_WLAST),
    .WVALID(ALU_WVALID),
    .BREADY(ALU_BREADY),
    .ARADDR(ALU_ARADDR),
    .ARVALID(ALU_ARVALID),
    .RREADY(ALU_RREADY),
    .qr_Idle(ALU_qr_Idle),
    .qr_ARValid(ALU_qr_ARValid),
    .qr_ReadReady(ALU_qr_ReadReady),
    .qr_Read(ALU_qr_Read),
    .qw_Idle(ALU_qw_Idle),
    .qw_AWValid(ALU_qw_AWValid),
    .qw_GetDataToWrite(ALU_qw_GetDataToWrite),
    .qw_Write(ALU_qw_Write),
    .qw_WaitForResponse(ALU_qw_WaitForResponse),
    .tb_ReadRequest(ALU_ReadRequest),
    .tb_ReadAddress(ALU_ReadAddress),
    .tb_NumAddressesToRead(ALU_NumAddressesToRead),
    .tb_ReadTransactionID(ALU_ReadTransactionID),
    .tb_ReadValue(ALU_tb_ReadValue),
    .tb_WriteRequest(ALU_WriteRequest),
    .tb_WriteAddress(ALU_WriteAddress),
    .tb_WriteTransactionID(ALU_WriteTransactionID),
    .tb_WriteDataValid(ALU_WriteDataValid),
    .tb_WriteValue(ALU_WriteValue),
    .tb_WriteLast(ALU_WriteLast),
    .RLAST(ALU_RLAST)
);

// Memory Master Inputs
input Memory_ARREADY, Memory_RVALID, Memory_AWREADY, Memory_WREADY, Memory_BVALID, Memory_RLAST;
reg Memory_ReadRequest, Memory_WriteRequest, Memory_WriteDataValid, Memory_WriteLast;
input [7:0] Memory_RDATA;  // 8-bit data values
input [9:0] Memory_BRESP;  // {status[1:0], overflow count[7:0]}; status 01 = OK, 10 = WARN
reg [7:0] Memory_ReadAddress;
reg [3:0] Memory_NumAddressesToRead;
reg [9:0] Memory_ReadTransactionID;
reg [7:0] Memory_WriteAddress;   // 8-bit address
reg [9:0] Memory_WriteTransactionID;
reg [7:0] Memory_WriteValue;

// Memory Master Outputs
output Memory_AWVALID, Memory_WLAST, Memory_WVALID, Memory_BREADY, Memory_ARVALID, Memory_RREADY;
output Memory_qr_Idle, Memory_qr_ARValid, Memory_qr_ReadReady, Memory_qr_Read, Memory_qw_Idle,
       Memory_qw_AWValid, Memory_qw_GetDataToWrite, Memory_qw_Write, Memory_qw_WaitForResponse;
output [17:0] Memory_AWADDR;   // 8-bit address, 10 transaction id bits
output [21:0] Memory_ARADDR;  // 8 bit address, 4 control bits, 10 bits transaction ID
output [7:0] Memory_WDATA;
output [7:0] Memory_tb_ReadValue;

master_axi Memory_Master(
    .clk(clk),
    .reset(reset),
    .ARREADY(Memory_ARREADY),
    .RDATA(Memory_RDATA),
    .RVALID(Memory_RVALID),
    .AWREADY(Memory_AWREADY),
    .WREADY(Memory_WREADY),
    .BRESP(Memory_BRESP),
    .BVALID(Memory_BVALID),
    .AWADDR(Memory_AWADDR),
    .AWVALID(Memory_AWVALID),
    .WDATA(Memory_WDATA),
    .WLAST(Memory_WLAST),
    .WVALID(Memory_WVALID),
    .BREADY(Memory_BREADY),
    .ARADDR(Memory_ARADDR),
    .ARVALID(Memory_ARVALID),
    .RREADY(Memory_RREADY),
    .qr_Idle(Memory_qr_Idle),
    .qr_ARValid(Memory_qr_ARValid),
    .qr_ReadReady(Memory_qr_ReadReady),
    .qr_Read(Memory_qr_Read),
    .qw_Idle(Memory_qw_Idle),
    .qw_AWValid(Memory_qw_AWValid),
    .qw_GetDataToWrite(Memory_qw_GetDataToWrite),
    .qw_Write(Memory_qw_Write),
    .qw_WaitForResponse(Memory_qw_WaitForResponse),
    .tb_ReadRequest(Memory_ReadRequest),
    .tb_ReadAddress(Memory_ReadAddress),
    .tb_NumAddressesToRead(Memory_NumAddressesToRead),
    .tb_ReadTransactionID(Memory_ReadTransactionID),
    .tb_ReadValue(Memory_tb_ReadValue),
    .tb_WriteRequest(Memory_WriteRequest),
    .tb_WriteAddress(Memory_WriteAddress),
    .tb_WriteTransactionID(Memory_WriteTransactionID),
    .tb_WriteDataValid(Memory_WriteDataValid),
    .tb_WriteValue(Memory_WriteValue),
    .tb_WriteLast(Memory_WriteLast),
    .RLAST(Memory_RLAST)
);

// I/O Master Inputs
input IO_ARREADY, IO_RVALID, IO_AWREADY, IO_WREADY, IO_BVALID, IO_RLAST;
reg IO_ReadRequest, IO_WriteRequest, IO_WriteDataValid, IO_WriteLast;
input [7:0] IO_RDATA;  // 8-bit data values
input [9:0] IO_BRESP;  // {status[1:0], overflow count[7:0]}; status 01 = OK, 10 = WARN
reg [7:0] IO_ReadAddress;
reg [3:0] IO_NumAddressesToRead;
reg [9:0] IO_ReadTransactionID;
reg [7:0] IO_WriteAddress;   // 8-bit address
reg [9:0] IO_WriteTransactionID;
reg [7:0] IO_WriteValue;

// I/O Master Outputs
output IO_AWVALID, IO_WLAST, IO_WVALID, IO_BREADY, IO_ARVALID, IO_RREADY;
output IO_qr_Idle, IO_qr_ARValid, IO_qr_ReadReady, IO_qr_Read, IO_qw_Idle, IO_qw_AWValid,
       IO_qw_GetDataToWrite, IO_qw_Write, IO_qw_WaitForResponse;
output [17:0] IO_AWADDR;   // 8-bit address, 10 transaction id bits
output [21:0] IO_ARADDR;  // 8 bit address, 4 control bits, 10 bits transaction ID
output [7:0] IO_WDATA;
output [7:0] IO_tb_ReadValue;

master_axi IO_Master(
    .clk(clk),
    .reset(reset),
    .ARREADY(IO_ARREADY),
    .RDATA(IO_RDATA),
    .RVALID(IO_RVALID),
    .AWREADY(IO_AWREADY),
    .WREADY(IO_WREADY),
    .BRESP(IO_BRESP),
    .BVALID(IO_BVALID),
    .AWADDR(IO_AWADDR),
    .AWVALID(IO_AWVALID),
    .WDATA(IO_WDATA),
    .WLAST(IO_WLAST),
    .WVALID(IO_WVALID),
    .BREADY(IO_BREADY),
    .ARADDR(IO_ARADDR),
    .ARVALID(IO_ARVALID),
    .RREADY(IO_RREADY),
    .qr_Idle(IO_qr_Idle),
    .qr_ARValid(IO_qr_ARValid),
    .qr_ReadReady(IO_qr_ReadReady),
    .qr_Read(IO_qr_Read),
    .qw_Idle(IO_qw_Idle),
    .qw_AWValid(IO_qw_AWValid),
    .qw_GetDataToWrite(IO_qw_GetDataToWrite),
    .qw_Write(IO_qw_Write),
    .qw_WaitForResponse(IO_qw_WaitForResponse),
    .tb_ReadRequest(IO_ReadRequest),
    .tb_ReadAddress(IO_ReadAddress),
    .tb_NumAddressesToRead(IO_NumAddressesToRead),
    .tb_ReadTransactionID(IO_ReadTransactionID),
    .tb_ReadValue(IO_tb_ReadValue),
    .tb_WriteRequest(IO_WriteRequest),
    .tb_WriteAddress(IO_WriteAddress),
    .tb_WriteTransactionID(IO_WriteTransactionID),
    .tb_WriteDataValid(IO_WriteDataValid),
    .tb_WriteValue(IO_WriteValue),
    .tb_WriteLast(IO_WriteLast),
    .RLAST(IO_RLAST)
);

// Instruction buffer: 16 entries
// 8-bits[29:22]: Value
// 8-bits[21:14]: Address
// 2-bits[13:12]: ALU (0), Memory (1), I/O (2)
// 1-bit[11]: Read (0), Write (1)
// 10-bits[10:1]: Transaction ID
// 1-bit[0]: Done? (0 = No, 1 = Yes)
reg [29:0] buffer[15:0];
reg [4:0] read_pointer;     // (n+1)-bit pointer
reg [4:0] write_pointer;    // (n+1)-bit pointer

reg ALUInstructionOK, MemoryInstructionOK, IOInstructionOK;
reg [3:0] ALU_Task_Pointer, Memory_Task_Pointer, IO_Task_Pointer;
integer ALU_Delay, Memory_Delay, IO_Delay, i, j, FIFO_depth, ALU_FIFO_depth, Memory_FIFO_depth,
        IO_FIFO_depth, FIFO_index, ALU_FIFO_index, ALU_FIFO_inner_index, Memory_FIFO_index,
        Memory_FIFO_inner_index, IO_FIFO_index, IO_FIFO_inner_index, ALU_i, ALU_j, Memory_i,
        Memory_j, IO_i, IO_j;

output ALU_Q_FINDTASK, ALU_Q_PROCESSTASK, FIFO_Q_NOTFULL, FIFO_Q_GOTVALUE, FIFO_Q_FULL,
       MEMORY_Q_FINDTASK, MEMORY_Q_PROCESSTASK, IO_Q_FINDTASK, IO_Q_PROCESSTASK;

reg [2:0] FIFO_State;
assign {FIFO_Q_FULL, FIFO_Q_GOTVALUE, FIFO_Q_NOTFULL} = FIFO_State;
reg [1:0] ALU_Master_State;
assign {ALU_Q_PROCESSTASK, ALU_Q_FINDTASK} = ALU_Master_State;
reg [1:0] Memory_Master_State;
assign {MEMORY_Q_PROCESSTASK, MEMORY_Q_FINDTASK} = Memory_Master_State;
reg [1:0] IO_Master_State;
assign {IO_Q_PROCESSTASK, IO_Q_FINDTASK} = IO_Master_State;

// One-hot encoded states
localparam FIFO_NOTFULL = 3'b001, FIFO_GOTVALUE = 3'b010, FIFO_FULL = 3'b100, ALU_FINDTASK = 2'b01,
           ALU_PROCESSTASK = 2'b10, MEMORY_FINDTASK = 2'b01, MEMORY_PROCESSTASK = 2'b10,
           IO_FINDTASK = 2'b01, IO_PROCESSTASK = 2'b10;

localparam DEBUG = 0;  // Set to 1 for verbose simulation output
output reg FIFO_IsEmpty;

// Instruction buffer (FIFO)
always @ (posedge clk, posedge reset)
begin
  if(reset)
  begin
    // Reset buffer and buffer pointers
    for (i = 0; i < 16; i = i + 1)
    begin
      buffer[i] <= 30'bx;
    end

    FIFO_depth <= 0;
    read_pointer <= 5'b0;
    write_pointer <= 5'b0;
    FIFO_State <= FIFO_NOTFULL;
    FIFO_IsEmpty <= 1;
  end

  else
    case(FIFO_State)
      FIFO_NOTFULL:
      begin
        FIFO_depth = (write_pointer - read_pointer) % 32;
        if(FIFO_depth == 0)
        begin
            FIFO_IsEmpty <= 1;
        end
        else
        begin
            FIFO_IsEmpty <= 0;
        end

        if(DEBUG)
          begin
            $display("\nFIFO State: FIFO_NOTFULL,\tFIFO Depth: %d\n", FIFO_depth);
          end
        // Advance the read pointer past completed entries at the head of the queue
        for(i = 0; i < FIFO_depth; i = i + 1)
        begin
          FIFO_index = (read_pointer + i) % 16;
          if (buffer[FIFO_index][0] == 1'b0)
          begin
            // Exit the loop (Verilog-2001 has no break)
            i = FIFO_depth;
          end
          else
          begin
            FIFO_depth = FIFO_depth - 1;
            read_pointer <= (read_pointer + i + 1) % 32;
          end
        end
        if(DEBUG)
          begin
            $display("\tRead Pointer: %d, Write Pointer: %d\n", read_pointer, write_pointer);
          end

        // Get new value if it's ready
        if (tb_IsNewInstruction == 1'b1)
        begin
          FIFO_State <= FIFO_GOTVALUE;
          buffer[(write_pointer%16)] <= tb_NewInstruction;
          write_pointer = (write_pointer + 1'b1) % 32;
          if(DEBUG)
            begin
                $display("\tGetting new instruction; New Write Pointer: %d\n", write_pointer);
            end
        end
      end

      FIFO_GOTVALUE:
      begin
        FIFO_depth = (write_pointer - read_pointer) % 32;
        if(FIFO_depth == 0)
        begin
            FIFO_IsEmpty <= 1;
        end
        else
        begin
            FIFO_IsEmpty <= 0;
        end
        if(DEBUG)
          begin
            $display("\nFIFO State: FIFO_GOTVALUE,\tFIFO Depth: %d\n", FIFO_depth);
          end

        // Advance the read pointer past completed entries at the head of the queue
        for(i = 0; i < FIFO_depth; i = i + 1)
        begin
          FIFO_index = (read_pointer + i) % 16;
          if (buffer[FIFO_index][0] == 1'b0)
          begin
            i = FIFO_depth;
          end
          else
          begin
            FIFO_depth = FIFO_depth - 1;
            read_pointer <= (read_pointer + i + 1) % 32;
          end
        end
        if(DEBUG)
          begin
            $display("\tRead Pointer: %d, Write Pointer: %d\n", read_pointer, write_pointer);
          end

        if (FIFO_depth < 16)
        begin
          FIFO_State <= FIFO_NOTFULL;
        end
        else
        begin
          FIFO_State <= FIFO_FULL;
        end
      end

      FIFO_FULL:
      begin
        FIFO_depth = (write_pointer - read_pointer) % 32;
        if(FIFO_depth == 0)
        begin
            FIFO_IsEmpty <= 1;
        end
        else
        begin
            FIFO_IsEmpty <= 0;
        end
        if(DEBUG)
          begin
            $display("\nFIFO State: FIFO_NOTFULL,\tFIFO Depth: %d\n", FIFO_depth);
          end
        // Advance the read pointer past completed entries at the head of the queue
        for(i = 0; i < FIFO_depth; i = i + 1)
        begin
          FIFO_index = (read_pointer + i) % 16;
          if (buffer[FIFO_index][0] == 1'b0)
          begin
            i = FIFO_depth;
          end
          else
          begin
            FIFO_depth = FIFO_depth - 1;
            read_pointer <= (read_pointer + i + 1) % 32;
          end
        end
        if(DEBUG)
          begin
            $display("\tRead Pointer: %d, Write Pointer: %d\n", read_pointer, write_pointer);
          end

        if (FIFO_depth < 16)
        begin
          FIFO_State <= FIFO_NOTFULL;
        end
      end
    endcase
end

// ALU scheduler
always @ (posedge clk, posedge reset)
begin
  if(reset)
    begin
      // Reset ALU master request signals
      ALU_ReadRequest <= 0;
      ALU_WriteRequest <= 0;
      ALU_WriteDataValid <= 0;
      ALU_WriteLast <= 0;
      ALU_ReadAddress <= 8'bx;
      ALU_NumAddressesToRead <= 4'bx;
      ALU_ReadTransactionID <= 10'bx;
      ALU_WriteAddress <= 8'bx;
      ALU_WriteTransactionID <= 10'bx;
      ALU_WriteValue <= 8'bx;
      ALU_Task_Pointer <= 4'bx;
      ALU_FIFO_index <= 0;
      ALU_Master_State <= ALU_FINDTASK;
    end

  else
    case(ALU_Master_State)
      ALU_FINDTASK:
      begin

        ALU_FIFO_depth = (write_pointer - read_pointer) % 32;
        if(DEBUG)
          begin
            $display("\nController: ALU Master State: ALU_FINDTASK,\tFIFO Depth: %d\n", ALU_FIFO_depth);
          end

        for (ALU_i = 0; ALU_i < ALU_FIFO_depth; ALU_i = ALU_i + 1)
        begin
          ALU_FIFO_index = (read_pointer + ALU_i) % 16;
          // ALU instruction
          if ((buffer[ALU_FIFO_index][13:12] == 2'b00) & (buffer[ALU_FIFO_index][0] == 1'b0))
          begin
            ALUInstructionOK = 1'b1;
            for(ALU_j = 0; ALU_j < ALU_i; ALU_j = ALU_j + 1)
            begin
              ALU_FIFO_inner_index = (read_pointer + ALU_j) % 16;
              if(DEBUG)
              begin
                $display("\tALU FIFO Inner Index: %d, ALU FIFO Outer Index: %d", ALU_FIFO_inner_index, ALU_FIFO_index);
              end
              // Check if there are other transaction IDs that instruction depends on
              if ((buffer[ALU_FIFO_inner_index][10:1] == buffer[ALU_FIFO_index][10:1]) & (buffer[ALU_FIFO_inner_index][0] == 1'b0))
              begin
                ALUInstructionOK = 1'b0;
                if(DEBUG)
                begin
                    $display("\t\tInstruction conflict, trying to break...");
                end
                ALU_j = ALU_i;
              end
            end

            if (ALUInstructionOK == 1'b1)
            begin
              if(dependency_test)
                $display("ALU Task Started: R/W: %d, ID: %d, Instruction: %b", buffer[ALU_FIFO_index][11], buffer[ALU_FIFO_index][10:1], buffer[ALU_FIFO_index]);
              ALU_Task_Pointer <= ALU_FIFO_index;
              ALU_Delay <= 1;
              ALU_Master_State <= ALU_PROCESSTASK;
              ALU_i = ALU_FIFO_depth;

              if(buffer[ALU_FIFO_index][11] == 1'b0)    // Read
              begin
                ALU_ReadTransactionID <= buffer[ALU_FIFO_index][10:1];
                ALU_ReadAddress <= buffer[ALU_FIFO_index][21:14];
                ALU_NumAddressesToRead <= 4'd1;
                ALU_ReadRequest <= 1;
              end
              else    // Write
              begin
                ALU_WriteTransactionID <= buffer[ALU_FIFO_index][10:1];
                ALU_WriteAddress <= buffer[ALU_FIFO_index][21:14];
                ALU_WriteRequest <= 1;
                ALU_WriteLast <= 1;
                ALU_WriteValue <= buffer[ALU_FIFO_index][29:22];
                ALU_WriteDataValid <= 1;
              end
            end
          end
        end
      end

      ALU_PROCESSTASK:
      begin
        if(DEBUG)
          begin
            $display("\nALU Master State: ALU_PROCESSTASK,\tALU Delay: %d\n", ALU_Delay);
          end
        ALU_Delay <= ALU_Delay + 1'b1;
        if(ALU_Delay == 8)
        begin
            buffer[ALU_Task_Pointer][0] <= 1'b1;
        end
        if (ALU_Delay == 9)
        begin
          ALU_Master_State <= ALU_FINDTASK;
          ALU_Task_Pointer <= 4'bx;
          if(dependency_test)
            $display("ALU Task Complete: R/W: %d, ID: %d, Instruction: %b", buffer[ALU_Task_Pointer][11], buffer[ALU_Task_Pointer][10:1], buffer[ALU_Task_Pointer]);
        end

        if (ALU_qr_ARValid)
        begin
          ALU_ReadRequest <= 0;
        end

        if (ALU_qw_AWValid)
        begin
          ALU_WriteRequest <= 0;
        end

      end

    endcase
end


// Memory scheduler
always @ (posedge clk, posedge reset)
begin
  if(reset)
    begin
      // Reset Memory master request signals
      Memory_ReadRequest <= 0;
      Memory_WriteRequest <= 0;
      Memory_WriteDataValid <= 0;
      Memory_WriteLast <= 0;
      Memory_ReadAddress <= 8'bx;
      Memory_NumAddressesToRead <= 4'bx;
      Memory_ReadTransactionID <= 10'bx;
      Memory_WriteAddress <= 8'bx;
      Memory_WriteTransactionID <= 10'bx;
      Memory_WriteValue <= 8'bx;
      Memory_Task_Pointer <= 4'bx;
      MemoryInstructionOK <= 1'b0;
      Memory_Master_State <= MEMORY_FINDTASK;
    end

  else
    case(Memory_Master_State)
      MEMORY_FINDTASK:
      begin

        Memory_FIFO_depth = (write_pointer - read_pointer) % 32;
        if(DEBUG)
          begin
            $display("\nController: Memory Master State: MEMORY_FINDTASK,\tFIFO Depth: %d\n", Memory_FIFO_depth);
          end

        for (Memory_i = 0; Memory_i < Memory_FIFO_depth; Memory_i = Memory_i + 1)
        begin
          Memory_FIFO_index = (read_pointer + Memory_i) % 16;
          // Memory instruction
          if ((buffer[Memory_FIFO_index][13:12] == 2'b01) & (buffer[Memory_FIFO_index][0] == 1'b0))
          begin
            MemoryInstructionOK = 1'b1;
            for(Memory_j = 0; Memory_j < Memory_i; Memory_j = Memory_j + 1)
            begin
              Memory_FIFO_inner_index = (read_pointer + Memory_j) % 16;
              if(DEBUG)
              begin
                $display("\tMemory FIFO Inner Index: %d, Memory FIFO Outer Index: %d", Memory_FIFO_inner_index, Memory_FIFO_index);
              end
              // Check if there are other transaction IDs that instruction depends on
              if ((buffer[Memory_FIFO_inner_index][10:1] == buffer[Memory_FIFO_index][10:1]) & (buffer[Memory_FIFO_inner_index][0] == 1'b0))
              begin
                if(DEBUG)
                begin
                    $display("\t\tInstruction conflict, trying to break...");
                end
                MemoryInstructionOK = 1'b0;
                Memory_j = Memory_i;
              end
              else
                begin
                    if(DEBUG)
                    begin
                        $display("\t\tNo internal conflict; ID at inner: %d, ID at outer: %d", buffer[Memory_FIFO_inner_index][10:1], buffer[Memory_FIFO_index][10:1]);
                    end
                end
            end

            if (MemoryInstructionOK == 1'b1)
            begin
              Memory_Task_Pointer <= Memory_FIFO_index;
              Memory_Delay <= 1;
              Memory_Master_State <= MEMORY_PROCESSTASK;
              Memory_i = Memory_FIFO_depth;
              if(dependency_test)
                $display("Memory Task Started: R/W: %d, ID: %d, Instruction: %b", buffer[Memory_FIFO_index][11], buffer[Memory_FIFO_index][10:1], buffer[Memory_FIFO_index]);

              if(buffer[Memory_FIFO_index][11] == 1'b0)    // Read
              begin
                Memory_ReadTransactionID <= buffer[Memory_FIFO_index][10:1];
                Memory_ReadAddress <= buffer[Memory_FIFO_index][21:14];
                Memory_NumAddressesToRead <= 4'd1;
                Memory_ReadRequest <= 1;
              end
              else    // Write
              begin
                Memory_WriteTransactionID <= buffer[Memory_FIFO_index][10:1];
                Memory_WriteAddress <= buffer[Memory_FIFO_index][21:14];
                Memory_WriteRequest <= 1;
                Memory_WriteLast <= 1;
                Memory_WriteValue <= buffer[Memory_FIFO_index][29:22];
                Memory_WriteDataValid <= 1;
              end
            end
          end
        end

      end

      MEMORY_PROCESSTASK:
      begin
        if(DEBUG)
          begin
            $display("\nMemory Master State: MEMORY_PROCESSTASK,\tMemory Delay: %d\n", Memory_Delay);
          end
        Memory_Delay <= Memory_Delay + 1'b1;
        if(Memory_Delay == 18)
        begin
            buffer[Memory_Task_Pointer][0] <= 1'b1;
        end
        if (Memory_Delay == 19)
        begin
          Memory_Master_State <= MEMORY_FINDTASK;
          Memory_Task_Pointer <= 4'bx;
          if(dependency_test)
            $display("Memory Task Complete: R/W: %d, ID: %d, Instruction: %b", buffer[Memory_Task_Pointer][11], buffer[Memory_Task_Pointer][10:1], buffer[Memory_Task_Pointer]);
        end

        if (Memory_qr_ARValid)
        begin
          Memory_ReadRequest <= 0;
        end

        if (Memory_qw_AWValid)
        begin
          Memory_WriteRequest <= 0;
        end
      end
    endcase
end

// I/O scheduler
always @ (posedge clk, posedge reset)
begin
  if(reset)
    begin
      // Reset I/O master request signals
      IO_ReadRequest <= 0;
      IO_WriteRequest <= 0;
      IO_WriteDataValid <= 0;
      IO_WriteLast <= 0;
      IO_ReadAddress <= 8'bx;
      IO_NumAddressesToRead <= 4'bx;
      IO_ReadTransactionID <= 10'bx;
      IO_WriteAddress <= 8'bx;
      IO_WriteTransactionID <= 10'bx;
      IO_WriteValue <= 8'bx;
      IO_Task_Pointer <= 4'bx;
      IOInstructionOK <= 1'b0;
      IO_Master_State <= IO_FINDTASK;
    end

  else
    case(IO_Master_State)
      IO_FINDTASK:
      begin

        IO_FIFO_depth = (write_pointer - read_pointer) % 32;
        if(DEBUG)
          begin
            $display("\nController: IO Master State: IO_FINDTASK,\tFIFO Depth: %d\n", IO_FIFO_depth);
          end
        for (IO_i = 0; IO_i < IO_FIFO_depth; IO_i = IO_i + 1)
        begin
          IO_FIFO_index = (read_pointer + IO_i) % 16;
          // IO instruction
          if ((buffer[IO_FIFO_index][13:12] == 2'b10) & (buffer[IO_FIFO_index][0] == 1'b0))
          begin
            IOInstructionOK = 1'b1;
            for(IO_j = 0; IO_j < IO_i; IO_j = IO_j + 1)
            begin
              IO_FIFO_inner_index = (read_pointer + IO_j) % 16;
              if(DEBUG)
              begin
                $display("\tIO FIFO Inner Index: %d, IO FIFO Outer Index: %d", IO_FIFO_inner_index, IO_FIFO_index);
              end
              // Check if there are other transaction IDs that instruction depends on
              if ((buffer[IO_FIFO_inner_index][10:1] == buffer[IO_FIFO_index][10:1]) & (buffer[IO_FIFO_inner_index][0] == 1'b0))
              begin
                if(DEBUG)
                begin
                    $display("\t\tInstruction conflict, trying to break...");
                end
                IOInstructionOK = 1'b0;
                // Exit the loop (Verilog-2001 has no break)
                IO_j = IO_i;
              end
            end

            if (IOInstructionOK == 1'b1)
            begin
              IO_Task_Pointer <= IO_FIFO_index;
              IO_Delay <= 1;
              IO_Master_State <= IO_PROCESSTASK;
              IO_i = IO_FIFO_depth;
              if(dependency_test)
                $display("IO Task Started: R/W: %d, ID: %d, Instruction: %b", buffer[IO_FIFO_index][11], buffer[IO_FIFO_index][10:1], buffer[IO_FIFO_index]);

              if(buffer[IO_FIFO_index][11] == 1'b0)    // Read
              begin
                IO_ReadTransactionID <= buffer[IO_FIFO_index][10:1];
                IO_ReadAddress <= buffer[IO_FIFO_index][21:14];
                IO_NumAddressesToRead <= 4'd1;
                IO_ReadRequest <= 1;
              end
              else    // Write
              begin
                IO_WriteTransactionID <= buffer[IO_FIFO_index][10:1];
                IO_WriteAddress <= buffer[IO_FIFO_index][21:14];
                IO_WriteRequest <= 1;
                IO_WriteLast <= 1;
                IO_WriteValue <= buffer[IO_FIFO_index][29:22];
                IO_WriteDataValid <= 1;
              end
            end
          end
        end
      end

      IO_PROCESSTASK:
      begin
        if(DEBUG)
          begin
            $display("\nI/O Master State: IO_PROCESSTASK,\tIO Delay: %d\n", IO_Delay);
          end
        IO_Delay <= IO_Delay + 1'b1;
        if(IO_Delay == 28)
        begin
            buffer[IO_Task_Pointer][0] <= 1'b1;
        end
        if (IO_Delay == 29)
        begin
          IO_Master_State <= IO_FINDTASK;
          IO_Task_Pointer <= 4'bx;
        if(dependency_test)
            $display("IO Task Complete: R/W: %d, ID: %d, Instruction: %b", buffer[IO_Task_Pointer][11], buffer[IO_Task_Pointer][10:1], buffer[IO_Task_Pointer]);
        end

        if (IO_qr_ARValid)
        begin
          IO_ReadRequest <= 0;
        end

        if (IO_qw_AWValid)
        begin
          IO_WriteRequest <= 0;
        end
      end

    endcase
end



endmodule
