/*
Testing for Controller module communicating with Slaves
*/

`timescale 1ns / 100ps

module controller_tb_v;

// Memory
reg [7:0] ALU_mem[255:0];
reg [7:0] Memory_mem[255:0];
reg [7:0] IO_mem[255:0];
reg clk, reset;

// Helper signals
wire FIFO_IsEmpty;

// Controller state outputs
wire ALU_qr_Idle, ALU_qr_ARValid, ALU_qr_ReadReady, ALU_qr_Read, ALU_qw_Idle, ALU_qw_AWValid, ALU_qw_GetDataToWrite, ALU_qw_Write, ALU_qw_WaitForResponse, Memory_qr_Idle, Memory_qr_ARValid, Memory_qr_ReadReady, Memory_qr_Read, Memory_qw_Idle, Memory_qw_AWValid, Memory_qw_GetDataToWrite, Memory_qw_Write, Memory_qw_WaitForResponse, IO_qr_Idle, IO_qr_ARValid, IO_qr_ReadReady, IO_qr_Read, IO_qw_Idle, IO_qw_AWValid, IO_qw_GetDataToWrite, IO_qw_Write, IO_qw_WaitForResponse, ALU_Q_FINDTASK, ALU_Q_PROCESSTASK, FIFO_Q_FULL, FIFO_Q_GOTVALUE, FIFO_Q_NOTFULL, MEMORY_Q_FINDTASK, MEMORY_Q_PROCESSTASK, IO_Q_FINDTASK, IO_Q_PROCESSTASK;

// ALU Master outputs
wire ALU_AWVALID, ALU_WLAST, ALU_WVALID, ALU_BREADY, ALU_ARVALID, ALU_RREADY;
wire [17:0] ALU_AWADDR;
wire [21:0] ALU_ARADDR;
wire [7:0] ALU_WDATA;
wire [7:0] ALU_ReadValue;

// Memory Master outputs
wire Memory_AWVALID, Memory_WLAST, Memory_WVALID, Memory_BREADY, Memory_ARVALID, Memory_RREADY;
wire [17:0] Memory_AWADDR;
wire [21:0] Memory_ARADDR;
wire [7:0] Memory_WDATA;
wire [7:0] Memory_ReadValue;

// I/O Master Outputs
wire IO_AWVALID, IO_WLAST, IO_WVALID, IO_BREADY, IO_ARVALID, IO_RREADY;
wire [17:0] IO_AWADDR;
wire [21:0] IO_ARADDR;
wire [7:0] IO_WDATA;
wire [7:0] IO_ReadValue;

// Things TB controls/changes
reg tb_IsNewInstruction;
reg [29:0] tb_NewInstruction;

// ALU Slave Outputs
wire ALU_ARREADY, ALU_RVALID, ALU_AWREADY, ALU_WREADY, ALU_BVALID, ALU_tb_LastWrite, ALU_RLAST;
wire [9:0] ALU_BRESP;
wire [7:0] ALU_RDATA;
wire [7:0] ALU_tb_WriteValue;
wire [7:0] ALU_tb_WriteAddress;
wire [9:0] ALU_tb_WriteTransactionID;

// Memory Slave Outputs
wire Memory_ARREADY, Memory_RVALID, Memory_AWREADY, Memory_WREADY, Memory_BVALID, Memory_tb_LastWrite, Memory_RLAST;
wire [7:0] Memory_RDATA;
wire [9:0] Memory_BRESP;
wire [7:0] Memory_tb_WriteValue;
wire [7:0] Memory_tb_WriteAddress;
wire [9:0] Memory_tb_WriteTransactionID;

// IO Slave Outputs
wire IO_ARREADY, IO_RVALID, IO_AWREADY, IO_WREADY, IO_BVALID, IO_tb_LastWrite, IO_RLAST;
wire [7:0] IO_RDATA;
wire [9:0] IO_BRESP;
wire [7:0] IO_tb_WriteValue;
wire [7:0] IO_tb_WriteAddress;
wire [9:0] IO_tb_WriteTransactionID;

reg dependency_test;

// Initialize Controller
controller controller_uut(
	.clk(clk),
	.reset(reset),
	.tb_IsNewInstruction(tb_IsNewInstruction),
	.tb_NewInstruction(tb_NewInstruction),
	.ALU_AWVALID(ALU_AWVALID),
	.ALU_WLAST(ALU_WLAST),
	.ALU_WVALID(ALU_WVALID),
	.ALU_BREADY(ALU_BREADY),
	.ALU_ARVALID(ALU_ARVALID),
	.ALU_RREADY(ALU_RREADY),
	.ALU_qr_Idle(ALU_qr_Idle),
	.ALU_qr_ARValid(ALU_qr_ARValid),
	.ALU_qr_ReadReady(ALU_qr_ReadReady),
	.ALU_qr_Read(ALU_qr_Read),
	.ALU_qw_Idle(ALU_qw_Idle),
	.ALU_qw_AWValid(ALU_qw_AWValid),
	.ALU_qw_GetDataToWrite(ALU_qw_GetDataToWrite),
	.ALU_qw_Write(ALU_qw_Write),
	.ALU_qw_WaitForResponse(ALU_qw_WaitForResponse),
	.ALU_AWADDR(ALU_AWADDR),
	.ALU_ARADDR(ALU_ARADDR),
	.ALU_WDATA(ALU_WDATA),
	.ALU_tb_ReadValue(ALU_ReadValue),
	.ALU_ARREADY(ALU_ARREADY),
	.ALU_RVALID(ALU_RVALID),
	.ALU_AWREADY(ALU_AWREADY),
	.ALU_WREADY(ALU_WREADY),
	.ALU_BRESP(ALU_BRESP),
	.ALU_BVALID(ALU_BVALID),
	.ALU_tb_LastWrite(ALU_tb_LastWrite),
	.ALU_RLAST(ALU_RLAST),
	.ALU_RDATA(ALU_RDATA),
	.ALU_tb_WriteValue(ALU_tb_WriteValue),
	.ALU_tb_WriteAddress(ALU_tb_WriteAddress),
	.ALU_tb_WriteTransactionID(ALU_tb_WriteTransactionID),
	.Memory_AWVALID(Memory_AWVALID),
	.Memory_WLAST(Memory_WLAST),
	.Memory_WVALID(Memory_WVALID),
	.Memory_BREADY(Memory_BREADY),
	.Memory_ARVALID(Memory_ARVALID),
	.Memory_RREADY(Memory_RREADY),
	.Memory_qr_Idle(Memory_qr_Idle),
	.Memory_qr_ARValid(Memory_qr_ARValid),
	.Memory_qr_ReadReady(Memory_qr_ReadReady),
	.Memory_qr_Read(Memory_qr_Read),
	.Memory_qw_Idle(Memory_qw_Idle),
	.Memory_qw_AWValid(Memory_qw_AWValid),
	.Memory_qw_GetDataToWrite(Memory_qw_GetDataToWrite),
	.Memory_qw_Write(Memory_qw_Write),
	.Memory_qw_WaitForResponse(Memory_qw_WaitForResponse),
	.Memory_AWADDR(Memory_AWADDR),
	.Memory_ARADDR(Memory_ARADDR),
	.Memory_WDATA(Memory_WDATA),
	.Memory_tb_ReadValue(Memory_ReadValue),
	.Memory_ARREADY(Memory_ARREADY),
	.Memory_RVALID(Memory_RVALID),
	.Memory_AWREADY(Memory_AWREADY),
	.Memory_WREADY(Memory_WREADY),
	.Memory_BRESP(Memory_BRESP),
	.Memory_BVALID(Memory_BVALID),
	.Memory_tb_LastWrite(Memory_tb_LastWrite),
	.Memory_RLAST(Memory_RLAST),
	.Memory_RDATA(Memory_RDATA),
	.Memory_tb_WriteValue(Memory_tb_WriteValue),
	.Memory_tb_WriteAddress(Memory_tb_WriteAddress),
	.Memory_tb_WriteTransactionID(Memory_tb_WriteTransactionID),
	.IO_AWVALID(IO_AWVALID),
	.IO_WLAST(IO_WLAST),
	.IO_WVALID(IO_WVALID),
	.IO_BREADY(IO_BREADY),
	.IO_ARVALID(IO_ARVALID),
	.IO_RREADY(IO_RREADY),
	.IO_qr_Idle(IO_qr_Idle),
	.IO_qr_ARValid(IO_qr_ARValid),
	.IO_qr_ReadReady(IO_qr_ReadReady),
	.IO_qr_Read(IO_qr_Read),
	.IO_qw_Idle(IO_qw_Idle),
	.IO_qw_AWValid(IO_qw_AWValid),
	.IO_qw_GetDataToWrite(IO_qw_GetDataToWrite),
	.IO_qw_Write(IO_qw_Write),
	.IO_qw_WaitForResponse(IO_qw_WaitForResponse),
	.IO_AWADDR(IO_AWADDR),
	.IO_ARADDR(IO_ARADDR),
	.IO_WDATA(IO_WDATA),
	.IO_tb_ReadValue(IO_ReadValue),
	.IO_ARREADY(IO_ARREADY),
	.IO_RVALID(IO_RVALID),
	.IO_AWREADY(IO_AWREADY),
	.IO_WREADY(IO_WREADY),
	.IO_BRESP(IO_BRESP),
	.IO_BVALID(IO_BVALID),
	.IO_tb_LastWrite(IO_tb_LastWrite),
	.IO_RLAST(IO_RLAST),
	.IO_RDATA(IO_RDATA),
	.IO_tb_WriteValue(IO_tb_WriteValue),
	.IO_tb_WriteAddress(IO_tb_WriteAddress),
	.IO_tb_WriteTransactionID(IO_tb_WriteTransactionID),
	.ALU_Q_FINDTASK(ALU_Q_FINDTASK),
	.ALU_Q_PROCESSTASK(ALU_Q_PROCESSTASK),
	.FIFO_Q_FULL(FIFO_Q_FULL),
	.FIFO_Q_GOTVALUE(FIFO_Q_GOTVALUE),
	.FIFO_Q_NOTFULL(FIFO_Q_NOTFULL),
	.MEMORY_Q_FINDTASK(MEMORY_Q_FINDTASK),
	.MEMORY_Q_PROCESSTASK(MEMORY_Q_PROCESSTASK),
	.IO_Q_FINDTASK(IO_Q_FINDTASK),
	.IO_Q_PROCESSTASK(IO_Q_PROCESSTASK),
	.FIFO_IsEmpty(FIFO_IsEmpty),
	.dependency_test(dependency_test)
);

// ALU Slave States
wire ALU_Slave_qw_Idle, ALU_Slave_qw_GetData, ALU_Slave_qw_WriteData, ALU_Slave_qw_Response, ALU_Slave_qr_Idle, ALU_Slave_qr_ReadData, ALU_Slave_qr_HoldData;

slave_axi ALU_slave(
	.clk(clk),
    .reset(reset),
    .ARADDR(ALU_ARADDR),
    .ARVALID(ALU_ARVALID),
    .RREADY(ALU_RREADY),
    .ARREADY(ALU_ARREADY),
    .RDATA(ALU_RDATA),
    .RLAST(ALU_RLAST),
    .RVALID(ALU_RVALID),
    .qr_Idle(ALU_Slave_qr_Idle),
    .qr_ReadData(ALU_Slave_qr_ReadData),
    .qr_HoldData(ALU_Slave_qr_HoldData),
    .AWADDR(ALU_AWADDR),
    .AWVALID(ALU_AWVALID),
    .AWREADY(ALU_AWREADY),
    .WREADY(ALU_WREADY),
    .BRESP(ALU_BRESP),
    .BVALID(ALU_BVALID),
    .qw_Idle(ALU_Slave_qw_Idle),
    .qw_GetData(ALU_Slave_qw_GetData),
    .qw_WriteData(ALU_Slave_qw_WriteData),
    .qw_Response(ALU_Slave_qw_Response),
    .tb_WriteValue(ALU_tb_WriteValue),
    .tb_LastWrite(ALU_tb_LastWrite),
    .tb_WriteAddress(ALU_tb_WriteAddress),
    .tb_WriteTransactionID(ALU_tb_WriteTransactionID),
    .WLAST(ALU_WLAST),
    .WVALID(ALU_WVALID),
    .BREADY(ALU_BREADY),
    .WDATA(ALU_WDATA)
);

// Memory Slave States
wire Memory_Slave_qw_Idle, Memory_Slave_qw_GetData, Memory_Slave_qw_WriteData, Memory_Slave_qw_Response, Memory_Slave_qr_Idle, Memory_Slave_qr_ReadData, Memory_Slave_qr_HoldData;

slave_axi Memory_slave(
	.clk(clk),
    .reset(reset),
    .ARADDR(Memory_ARADDR),
    .ARVALID(Memory_ARVALID),
    .RREADY(Memory_RREADY),
    .ARREADY(Memory_ARREADY),
    .RDATA(Memory_RDATA),
    .RLAST(Memory_RLAST),
    .RVALID(Memory_RVALID),
    .qr_Idle(Memory_Slave_qr_Idle),
    .qr_ReadData(Memory_Slave_qr_ReadData),
    .qr_HoldData(Memory_Slave_qr_HoldData),
    .AWADDR(Memory_AWADDR),
    .AWVALID(Memory_AWVALID),
    .AWREADY(Memory_AWREADY),
    .WREADY(Memory_WREADY),
    .BRESP(Memory_BRESP),
    .BVALID(Memory_BVALID),
    .qw_Idle(Memory_Slave_qw_Idle),
    .qw_GetData(Memory_Slave_qw_GetData),
    .qw_WriteData(Memory_Slave_qw_WriteData),
    .qw_Response(Memory_Slave_qw_Response),
    .tb_WriteValue(Memory_tb_WriteValue),
    .tb_LastWrite(Memory_tb_LastWrite),
    .tb_WriteAddress(Memory_tb_WriteAddress),
    .tb_WriteTransactionID(Memory_tb_WriteTransactionID),
    .WLAST(Memory_WLAST),
    .WVALID(Memory_WVALID),
    .BREADY(Memory_BREADY),
    .WDATA(Memory_WDATA)
);

// IO Slave States
wire IO_Slave_qw_Idle, IO_Slave_qw_GetData, IO_Slave_qw_WriteData, IO_Slave_qw_Response, IO_Slave_qr_Idle, IO_Slave_qr_ReadData, IO_Slave_qr_HoldData;

slave_axi IO_slave(
	.clk(clk),
    .reset(reset),
    .ARADDR(IO_ARADDR),
    .ARVALID(IO_ARVALID),
    .RREADY(IO_RREADY),
    .ARREADY(IO_ARREADY),
    .RDATA(IO_RDATA),
    .RLAST(IO_RLAST),
    .RVALID(IO_RVALID),
    .qr_Idle(IO_Slave_qr_Idle),
    .qr_ReadData(IO_Slave_qr_ReadData),
    .qr_HoldData(IO_Slave_qr_HoldData),
    .AWADDR(IO_AWADDR),
    .AWVALID(IO_AWVALID),
    .AWREADY(IO_AWREADY),
    .WREADY(IO_WREADY),
    .BRESP(IO_BRESP),
    .BVALID(IO_BVALID),
    .qw_Idle(IO_Slave_qw_Idle),
    .qw_GetData(IO_Slave_qw_GetData),
    .qw_WriteData(IO_Slave_qw_WriteData),
    .qw_Response(IO_Slave_qw_Response),
    .tb_WriteValue(IO_tb_WriteValue),
    .tb_LastWrite(IO_tb_LastWrite),
    .tb_WriteAddress(IO_tb_WriteAddress),
    .tb_WriteTransactionID(IO_tb_WriteTransactionID),
    .WLAST(IO_WLAST),
    .WVALID(IO_WVALID),
    .BREADY(IO_BREADY),
    .WDATA(IO_WDATA)
);


always
  begin
    #2; clk = ~ clk;
  end

integer i;

initial begin
    // Initialize Inputs
    clk = 0;
    reset = 0;

    // Initialize Memory
    for (i = 0; i < 256; i = i + 1)
      begin
        ALU_mem[i] <= 8'b0;
		Memory_mem[i] <= 8'b0;
		IO_mem[i] <= 8'b0;
      end

    // Reset
    @(posedge clk);
    #1;
    reset=1;
    @(posedge clk);
    #1;
    reset=0;

	$display("\n------- Writes with ALU (No Full FIFO) -------\n\n");
	for(i = 0; i < 20; i = i + 1)
	  begin
		// Set value for write
		tb_NewInstruction[29:22] = 8'd255;
		// Set address
		tb_NewInstruction[21:14] = i;
		// ALU (0), Memory (1), I/O (2)
		tb_NewInstruction[13:12] = 2'd0;
		// Read (0), Write (1)
		tb_NewInstruction[11] = 1'b1;
		// Transaction ID
		tb_NewInstruction[10:1] = i;
		// Done? (y=1, n=0)
		tb_NewInstruction[0] = 0;

		tb_IsNewInstruction = 1'b1;

		// Write Value to Memory
		ALU_mem[i] = 8'd255;

		wait(FIFO_Q_GOTVALUE);
		tb_IsNewInstruction = 0;

		wait(ALU_Q_PROCESSTASK);
		wait(ALU_Q_FINDTASK);
		if(ALU_BRESP == 10'd256)
		  begin
			$display("\t[WRITE OK] -- Transaction ID: %d, BRESP Expected Value: %d, BRESP Actual Value: %d\n", tb_NewInstruction[10:1], 256, ALU_BRESP);
		  end
		else
		  begin
			$display("\t[WRITE ERROR] -- Transaction ID: %d, BRESP Expected Value: %d, BRESP Actual Value: %d\n", tb_NewInstruction[10:1], 256, ALU_BRESP);
		  end
	  end

	$display("\n------- Reads with ALU (No Full FIFO) -------\n\n");
	for(i = 0; i < 20; i = i + 1)
	  begin
		// Set value -- ignore for read
		tb_NewInstruction[29:22] = 8'bx;
		// Set address
		tb_NewInstruction[21:14] = i;
		// ALU (0), Memory (1), I/O (2)
		tb_NewInstruction[13:12] = 2'd0;
		// Read (0), Write (1)
		tb_NewInstruction[11] = 1'b0;
		// Transaction ID
		tb_NewInstruction[10:1] = i;
		// Done? (y=1, n=0)
		tb_NewInstruction[0] = 1'b0;

		tb_IsNewInstruction = 1'b1;

		wait(FIFO_Q_GOTVALUE);
		tb_IsNewInstruction = 0;

		wait(ALU_Q_PROCESSTASK);
		wait(ALU_Q_FINDTASK);
		//wait(ALU_Slave_qw_Idle);
		//wait(ALU_Slave_qr_HoldData);
		if(ALU_ReadValue == ALU_mem[i])
		  begin
			$display("\t[READ OK] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], ALU_mem[i], ALU_ReadValue);
		  end
		else
		  begin
			$display("\t[READ ERROR] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], ALU_mem[i], ALU_ReadValue);
		  end
	  end


	$display("\n------- Writes and Reads with ALU (Full FIFO) -------\n\n");
	for(i = 0; i < 30; i = i + 1)
	begin
		if(FIFO_Q_FULL)
		begin
			$display("FIFO is FULL!");
		end

		if (i%2 == 1) 	// Write
		begin
			// Set value for write
			tb_NewInstruction[29:22] = 8'd13;
			// Set address
			tb_NewInstruction[21:14] = i;
			// ALU (0), Memory (1), I/O (2)
			tb_NewInstruction[13:12] = 2'd0;
			// Read (0), Write (1)
			tb_NewInstruction[11] = 1'b1;
			// Transaction ID
			tb_NewInstruction[10:1] = i;
			// Done? (y=1, n=0)
			tb_NewInstruction[0] = 0;

			tb_IsNewInstruction = 1'b1;

			// Write Value to Memory
			ALU_mem[i] = 8'd13;

			wait(FIFO_Q_GOTVALUE);
			tb_IsNewInstruction = 0;
		end
		else
		begin
			// Set value -- ignore for read
			tb_NewInstruction[29:22] = 8'bx;
			// Set address
			tb_NewInstruction[21:14] = i;
			// ALU (0), Memory (1), I/O (2)
			tb_NewInstruction[13:12] = 2'd0;
			// Read (0), Write (1)
			tb_NewInstruction[11] = 1'b0;
			// Transaction ID
			tb_NewInstruction[10:1] = i;
			// Done? (y=1, n=0)
			tb_NewInstruction[0] = 1'b0;

			tb_IsNewInstruction = 1'b1;

			wait(FIFO_Q_GOTVALUE);
			tb_IsNewInstruction = 0;
		end
		wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);
	end

	wait(FIFO_IsEmpty);
	$display("FIFO is now empty");

	// Check that all values are correct
	for(i = 0; i < 20; i = i + 1)
	begin
		// Set value -- ignore for read
		tb_NewInstruction[29:22] = 8'bx;
		// Set address
		tb_NewInstruction[21:14] = i;
		// ALU (0), Memory (1), I/O (2)
		tb_NewInstruction[13:12] = 2'd0;
		// Read (0), Write (1)
		tb_NewInstruction[11] = 1'b0;
		// Transaction ID
		tb_NewInstruction[10:1] = i;
		// Done? (y=1, n=0)
		tb_NewInstruction[0] = 1'b0;

		tb_IsNewInstruction = 1'b1;

		if(FIFO_Q_FULL)
		begin
		$display("\tFIFO is Full! Holding value.");
		end

		wait(FIFO_Q_GOTVALUE);
		tb_IsNewInstruction = 0;

		wait(ALU_Q_PROCESSTASK);
		wait(ALU_Q_FINDTASK);

		if(ALU_ReadValue == ALU_mem[i])
		  begin
			$display("\t[READ OK] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], ALU_mem[i], ALU_ReadValue);
		  end
		else
		  begin
			$display("\t[READ ERROR] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], ALU_mem[i], ALU_ReadValue);
		  end
	end

	// ALU Exclusive Tests are Complete


	// Exclusive Tests for Memory
	$display("\n------- Writes with Memory (No Full FIFO) -------\n\n");
	for(i = 0; i < 20; i = i + 1)
	  begin
		// Set value for write
		tb_NewInstruction[29:22] = 8'd255;
		// Set address
		tb_NewInstruction[21:14] = i;
		// ALU (0), Memory (1), I/O (2)
		tb_NewInstruction[13:12] = 2'd1;
		// Read (0), Write (1)
		tb_NewInstruction[11] = 1'b1;
		// Transaction ID
		tb_NewInstruction[10:1] = i;
		// Done? (y=1, n=0)
		tb_NewInstruction[0] = 0;

		tb_IsNewInstruction = 1'b1;

		// Write Value to Memory
		Memory_mem[i] = 8'd255;

		wait(FIFO_Q_GOTVALUE);
		tb_IsNewInstruction = 0;

		wait(MEMORY_Q_PROCESSTASK);
		wait(MEMORY_Q_FINDTASK);
		if(Memory_BRESP == 10'd256)
		  begin
			$display("\t[WRITE OK] -- Transaction ID: %d, BRESP Expected Value: %d, BRESP Actual Value: %d\n", tb_NewInstruction[10:1], 256, Memory_BRESP);
		  end
		else
		  begin
			$display("\t[WRITE ERROR] -- Transaction ID: %d, BRESP Expected Value: %d, BRESP Actual Value: %d\n", tb_NewInstruction[10:1], 256, Memory_BRESP);
		  end
	  end

	$display("\n------- Reads with Memory (No Full FIFO) -------\n\n");
	for(i = 0; i < 20; i = i + 1)
	  begin
		// Set value -- ignore for read
		tb_NewInstruction[29:22] = 8'bx;
		// Set address
		tb_NewInstruction[21:14] = i;
		// ALU (0), Memory (1), I/O (2)
		tb_NewInstruction[13:12] = 2'd1;
		// Read (0), Write (1)
		tb_NewInstruction[11] = 1'b0;
		// Transaction ID
		tb_NewInstruction[10:1] = i;
		// Done? (y=1, n=0)
		tb_NewInstruction[0] = 1'b0;

		tb_IsNewInstruction = 1'b1;

		wait(FIFO_Q_GOTVALUE);
		tb_IsNewInstruction = 0;

		wait(MEMORY_Q_PROCESSTASK);
		wait(MEMORY_Q_FINDTASK);
		//wait(ALU_Slave_qw_Idle);
		//wait(ALU_Slave_qr_HoldData);
		if(Memory_ReadValue == Memory_mem[i])
		  begin
			$display("\t[READ OK] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], Memory_mem[i], Memory_ReadValue);
		  end
		else
		  begin
			$display("\t[READ ERROR] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], Memory_mem[i], Memory_ReadValue);
		  end
	  end


	$display("\n------- Writes and Reads with Memory (Full FIFO) -------\n\n");
	for(i = 0; i < 30; i = i + 1)
	begin
		if(FIFO_Q_FULL)
		begin
			$display("FIFO is FULL!");
		end

		if (i%2 == 1) 	// Write
		begin
			// Set value for write
			tb_NewInstruction[29:22] = 8'd13;
			// Set address
			tb_NewInstruction[21:14] = i;
			// ALU (0), Memory (1), I/O (2)
			tb_NewInstruction[13:12] = 2'd1;
			// Read (0), Write (1)
			tb_NewInstruction[11] = 1'b1;
			// Transaction ID
			tb_NewInstruction[10:1] = i;
			// Done? (y=1, n=0)
			tb_NewInstruction[0] = 0;

			tb_IsNewInstruction = 1'b1;

			// Write Value to Memory
			Memory_mem[i] = 8'd13;

			wait(FIFO_Q_GOTVALUE);
			tb_IsNewInstruction = 0;
		end
		else
		begin
			// Set value -- ignore for read
			tb_NewInstruction[29:22] = 8'bx;
			// Set address
			tb_NewInstruction[21:14] = i;
			// ALU (0), Memory (1), I/O (2)
			tb_NewInstruction[13:12] = 2'd1;
			// Read (0), Write (1)
			tb_NewInstruction[11] = 1'b0;
			// Transaction ID
			tb_NewInstruction[10:1] = i;
			// Done? (y=1, n=0)
			tb_NewInstruction[0] = 1'b0;

			tb_IsNewInstruction = 1'b1;

			wait(FIFO_Q_GOTVALUE);
			tb_IsNewInstruction = 0;
		end
		wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);
	end

	wait(FIFO_IsEmpty);
	$display("FIFO is now empty");

	// Check that all values are correct
	for(i = 0; i < 20; i = i + 1)
	begin
		// Set value -- ignore for read
		tb_NewInstruction[29:22] = 8'bx;
		// Set address
		tb_NewInstruction[21:14] = i;
		// ALU (0), Memory (1), I/O (2)
		tb_NewInstruction[13:12] = 2'd1;
		// Read (0), Write (1)
		tb_NewInstruction[11] = 1'b0;
		// Transaction ID
		tb_NewInstruction[10:1] = i;
		// Done? (y=1, n=0)
		tb_NewInstruction[0] = 1'b0;

		tb_IsNewInstruction = 1'b1;

		if(FIFO_Q_FULL)
		begin
		$display("\tFIFO is Full! Holding value.");
		end

		wait(FIFO_Q_GOTVALUE);
		tb_IsNewInstruction = 0;

		wait(MEMORY_Q_PROCESSTASK);
		wait(MEMORY_Q_FINDTASK);

		if(Memory_ReadValue == Memory_mem[i])
		  begin
			$display("\t[READ OK] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], Memory_mem[i], Memory_ReadValue);
		  end
		else
		  begin
			$display("\t[READ ERROR] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], Memory_mem[i], Memory_ReadValue);
		  end
	end


	// Exclusive Tests for I/O
	$display("\n------- Writes with I/O (No Full FIFO) -------\n\n");
	for(i = 0; i < 20; i = i + 1)
	  begin
		// Set value for write
		tb_NewInstruction[29:22] = 8'd255;
		// Set address
		tb_NewInstruction[21:14] = i;
		// ALU (0), Memory (1), I/O (2)
		tb_NewInstruction[13:12] = 2'd2;
		// Read (0), Write (1)
		tb_NewInstruction[11] = 1'b1;
		// Transaction ID
		tb_NewInstruction[10:1] = i;
		// Done? (y=1, n=0)
		tb_NewInstruction[0] = 0;

		tb_IsNewInstruction = 1'b1;

		// Write Value to Memory
		IO_mem[i] = 8'd255;

		wait(FIFO_Q_GOTVALUE);
		tb_IsNewInstruction = 0;

		wait(IO_Q_PROCESSTASK);
		wait(IO_Q_FINDTASK);
		if(IO_BRESP == 10'd256)
		  begin
			$display("\t[WRITE OK] -- Transaction ID: %d, BRESP Expected Value: %d, BRESP Actual Value: %d\n", tb_NewInstruction[10:1], 256, IO_BRESP);
		  end
		else
		  begin
			$display("\t[WRITE ERROR] -- Transaction ID: %d, BRESP Expected Value: %d, BRESP Actual Value: %d\n", tb_NewInstruction[10:1], 256, IO_BRESP);
		  end
	  end

	$display("\n------- Reads with Memory (No Full FIFO) -------\n\n");
	for(i = 0; i < 20; i = i + 1)
	  begin
		// Set value -- ignore for read
		tb_NewInstruction[29:22] = 8'bx;
		// Set address
		tb_NewInstruction[21:14] = i;
		// ALU (0), Memory (1), I/O (2)
		tb_NewInstruction[13:12] = 2'd2;
		// Read (0), Write (1)
		tb_NewInstruction[11] = 1'b0;
		// Transaction ID
		tb_NewInstruction[10:1] = i;
		// Done? (y=1, n=0)
		tb_NewInstruction[0] = 1'b0;

		tb_IsNewInstruction = 1'b1;

		wait(FIFO_Q_GOTVALUE);
		tb_IsNewInstruction = 0;

		wait(IO_Q_PROCESSTASK);
		wait(IO_Q_FINDTASK);
		//wait(ALU_Slave_qw_Idle);
		//wait(ALU_Slave_qr_HoldData);
		if(IO_ReadValue == IO_mem[i])
		  begin
			$display("\t[READ OK] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], IO_mem[i], IO_ReadValue);
		  end
		else
		  begin
			$display("\t[READ ERROR] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], IO_mem[i], IO_ReadValue);
		  end
	  end


	$display("\n------- Writes and Reads with I/O (Full FIFO) -------\n\n");
	for(i = 0; i < 30; i = i + 1)
	begin
		if(FIFO_Q_FULL)
		begin
			$display("FIFO is FULL!");
		end

		if (i%2 == 1) 	// Write
		begin
			// Set value for write
			tb_NewInstruction[29:22] = 8'd13;
			// Set address
			tb_NewInstruction[21:14] = i;
			// ALU (0), Memory (1), I/O (2)
			tb_NewInstruction[13:12] = 2'd2;
			// Read (0), Write (1)
			tb_NewInstruction[11] = 1'b1;
			// Transaction ID
			tb_NewInstruction[10:1] = i;
			// Done? (y=1, n=0)
			tb_NewInstruction[0] = 0;

			tb_IsNewInstruction = 1'b1;

			// Write Value to Memory
			IO_mem[i] = 8'd13;

			wait(FIFO_Q_GOTVALUE);
			tb_IsNewInstruction = 0;
		end
		else
		begin
			// Set value -- ignore for read
			tb_NewInstruction[29:22] = 8'bx;
			// Set address
			tb_NewInstruction[21:14] = i;
			// ALU (0), Memory (1), I/O (2)
			tb_NewInstruction[13:12] = 2'd2;
			// Read (0), Write (1)
			tb_NewInstruction[11] = 1'b0;
			// Transaction ID
			tb_NewInstruction[10:1] = i;
			// Done? (y=1, n=0)
			tb_NewInstruction[0] = 1'b0;

			tb_IsNewInstruction = 1'b1;

			wait(FIFO_Q_GOTVALUE);
			tb_IsNewInstruction = 0;
		end
		wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);
	end

	wait(FIFO_IsEmpty);
	$display("FIFO is now empty");

	// Check that all values are correct
	for(i = 0; i < 20; i = i + 1)
	begin
		// Set value -- ignore for read
		tb_NewInstruction[29:22] = 8'bx;
		// Set address
		tb_NewInstruction[21:14] = i;
		// ALU (0), Memory (1), I/O (2)
		tb_NewInstruction[13:12] = 2'd2;
		// Read (0), Write (1)
		tb_NewInstruction[11] = 1'b0;
		// Transaction ID
		tb_NewInstruction[10:1] = i;
		// Done? (y=1, n=0)
		tb_NewInstruction[0] = 1'b0;

		tb_IsNewInstruction = 1'b1;

		if(FIFO_Q_FULL)
		begin
		$display("\tFIFO is Full! Holding value.");
		end

		wait(FIFO_Q_GOTVALUE);
		tb_IsNewInstruction = 0;

		wait(IO_Q_PROCESSTASK);
		wait(IO_Q_FINDTASK);

		if(IO_ReadValue == IO_mem[i])
		  begin
			$display("\t[READ OK] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], IO_mem[i], IO_ReadValue);
		  end
		else
		  begin
			$display("\t[READ ERROR] -- Transaction ID: %d: Expected Value: %d, Actual Value: %d\n", tb_NewInstruction[10:1], IO_mem[i], IO_ReadValue);
		  end
	end



	$display("\n------- Test From Blackboard -------\n\n");
	dependency_test = 1'b1;
	#5;
	/* Output when testing:

	(Note: Some things start/complete on the same clock, so printout ordering is a little subjective)

	# ALU Task Started: R/W: 0, ID:    3, Instruction: xxxxxxxx0011011100000000000110
	# Memory Task Started: R/W: 1, ID:    4, Instruction: 000011010011011101100000001000
	# ALU Task Complete: R/W: 0, ID:    3, Instruction: xxxxxxxx0011011100000000000111
	# ALU Task Started: R/W: 1, ID:    3, Instruction: 000011010011011100100000000110
	# ALU Task Complete: R/W: 1, ID:    3, Instruction: 000011010011011100100000000111
	# Memory Task Complete: R/W: 1, ID:    4, Instruction: 000011010011011101100000001001
	# IO Task Started: R/W: 1, ID:    4, Instruction: 000011010011011110100000001000
	# Memory Task Started: R/W: 0, ID:    3, Instruction: xxxxxxxx0011011101000000000110
	# Memory Task Complete: R/W: 0, ID:    3, Instruction: xxxxxxxx0011011101000000000111
	# Memory Task Started: R/W: 0, ID:    5, Instruction: xxxxxxxx0011011101000000001010
	# IO Task Complete: R/W: 1, ID:    4, Instruction: 000011010011011110100000001001
	# Memory Task Complete: R/W: 0, ID:    5, Instruction: xxxxxxxx0011011101000000001011
	# Memory Task Started: R/W: 0, ID:    4, Instruction: xxxxxxxx0011011101000000001000
	# ALU Task Started: R/W: 1, ID:    4, Instruction: 000011010011011100100000001000
	# Memory Task Complete: R/W: 0, ID:    4, Instruction: xxxxxxxx0011011101000000001001
	# ALU Task Complete: R/W: 1, ID:    4, Instruction: 000011010011011100100000001001

	*/

	// Read from ALU, ID 3
	// Set value -- ignore for read
	tb_NewInstruction[29:22] = 8'bx;
	// Set address
	tb_NewInstruction[21:14] = 8'd55;
	// ALU (0), Memory (1), I/O (2)
	tb_NewInstruction[13:12] = 2'd0;
	// Read (0), Write (1)
	tb_NewInstruction[11] = 1'b0;
	// Transaction ID
	tb_NewInstruction[10:1] = 10'd3;
	// Done? (y=1, n=0)
	tb_NewInstruction[0] = 1'b0;

	tb_IsNewInstruction = 1'b1;

	wait(FIFO_Q_GOTVALUE);
	tb_IsNewInstruction = 0;

	wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);

	// Write to Memory, ID 4
	tb_NewInstruction[29:22] = 8'd13;
	// Set address
	tb_NewInstruction[21:14] = 8'd55;
	// ALU (0), Memory (1), I/O (2)
	tb_NewInstruction[13:12] = 2'd1;
	// Read (0), Write (1)
	tb_NewInstruction[11] = 1'b1;
	// Transaction ID
	tb_NewInstruction[10:1] = 10'd4;
	// Done? (y=1, n=0)
	tb_NewInstruction[0] = 0;

	tb_IsNewInstruction = 1'b1;

	// Write Value to Memory
	Memory_mem[i] = 8'd13;

	wait(FIFO_Q_GOTVALUE);
	tb_IsNewInstruction = 0;

	wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);


	// Write to IO, ID 4
	tb_NewInstruction[29:22] = 8'd13;
	// Set address
	tb_NewInstruction[21:14] = 8'd55;
	// ALU (0), Memory (1), I/O (2)
	tb_NewInstruction[13:12] = 2'd2;
	// Read (0), Write (1)
	tb_NewInstruction[11] = 1'b1;
	// Transaction ID
	tb_NewInstruction[10:1] = 10'd4;
	// Done? (y=1, n=0)
	tb_NewInstruction[0] = 0;

	tb_IsNewInstruction = 1'b1;

	// Write Value to Memory
	IO_mem[i] = 8'd13;

	wait(FIFO_Q_GOTVALUE);
	tb_IsNewInstruction = 0;

	wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);


	// Read from Memory, ID 4
	// Set value -- ignore for read
	tb_NewInstruction[29:22] = 8'bx;
	// Set address
	tb_NewInstruction[21:14] = 8'd55;
	// ALU (0), Memory (1), I/O (2)
	tb_NewInstruction[13:12] = 2'd1;
	// Read (0), Write (1)
	tb_NewInstruction[11] = 1'b0;
	// Transaction ID
	tb_NewInstruction[10:1] = 10'd4;
	// Done? (y=1, n=0)
	tb_NewInstruction[0] = 1'b0;

	tb_IsNewInstruction = 1'b1;

	wait(FIFO_Q_GOTVALUE);
	tb_IsNewInstruction = 0;

	wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);

	// Write to ALU, ID 3
	tb_NewInstruction[29:22] = 8'd13;
	// Set address
	tb_NewInstruction[21:14] = 8'd55;
	// ALU (0), Memory (1), I/O (2)
	tb_NewInstruction[13:12] = 2'b0;
	// Read (0), Write (1)
	tb_NewInstruction[11] = 1'b1;
	// Transaction ID
	tb_NewInstruction[10:1] = 10'd3;
	// Done? (y=1, n=0)
	tb_NewInstruction[0] = 0;

	tb_IsNewInstruction = 1'b1;

	// Write Value to Memory
	ALU_mem[i] = 8'd13;

	wait(FIFO_Q_GOTVALUE);
	tb_IsNewInstruction = 0;

	wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);

	// Read from Memory, ID 3
	// Set value -- ignore for read
	tb_NewInstruction[29:22] = 8'bx;
	// Set address
	tb_NewInstruction[21:14] = 8'd55;
	// ALU (0), Memory (1), I/O (2)
	tb_NewInstruction[13:12] = 2'd1;
	// Read (0), Write (1)
	tb_NewInstruction[11] = 1'b0;
	// Transaction ID
	tb_NewInstruction[10:1] = 10'd3;
	// Done? (y=1, n=0)
	tb_NewInstruction[0] = 1'b0;

	tb_IsNewInstruction = 1'b1;

	wait(FIFO_Q_GOTVALUE);
	tb_IsNewInstruction = 0;

	wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);

	// Read from Memory, ID 5
	// Set value -- ignore for read
	tb_NewInstruction[29:22] = 8'bx;
	// Set address
	tb_NewInstruction[21:14] = 8'd55;
	// ALU (0), Memory (1), I/O (2)
	tb_NewInstruction[13:12] = 2'd1;
	// Read (0), Write (1)
	tb_NewInstruction[11] = 1'b0;
	// Transaction ID
	tb_NewInstruction[10:1] = 10'd5;
	// Done? (y=1, n=0)
	tb_NewInstruction[0] = 1'b0;

	tb_IsNewInstruction = 1'b1;

	wait(FIFO_Q_GOTVALUE);
	tb_IsNewInstruction = 0;

	wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);

	// Write to ALU, ID 4
	tb_NewInstruction[29:22] = 8'd13;
	// Set address
	tb_NewInstruction[21:14] = 8'd55;
	// ALU (0), Memory (1), I/O (2)
	tb_NewInstruction[13:12] = 2'd0;
	// Read (0), Write (1)
	tb_NewInstruction[11] = 1'b1;
	// Transaction ID
	tb_NewInstruction[10:1] = 10'd4;
	// Done? (y=1, n=0)
	tb_NewInstruction[0] = 0;

	tb_IsNewInstruction = 1'b1;

	// Write Value to Memory
	ALU_mem[i] = 8'd13;

	wait(FIFO_Q_GOTVALUE);
	tb_IsNewInstruction = 0;

	wait(FIFO_Q_FULL | FIFO_Q_NOTFULL);

	wait(FIFO_IsEmpty);
	$display("FIFO is empty!");


end



endmodule