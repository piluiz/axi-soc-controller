/*
This module implements the slave implementation of the AXI Protocol. This code will support read and write requests.

Addresses will be 8-bit, accessing a memory space of 256 8-bit values (contained within the slave module). When an 8-bit key is provided, you must return the 8-bit value corresponding to that location. All memories start at zeroes, upon a reset.

You must include a space for transaction IDs, sent along with addresses.

Your modules must allow burst-based writing and reading:

— BURST WRITE: When multiple 8-bit data values are provided sequentially, you must write to the provided write address, then the next address in order, and so on, until WLAST is asserted. If this would force an address overflow (as in, you’re requested to write to a location past the maximum address of the memory), ignore them: do not over-write the low-addressed locations.

— BURST READ: When asking for a read, in addition to the read address, the master module will specify a number of distinct memory addresses to be read (up to 16), starting at the provided address and continuing sequentially. This number of requested addresses (4 bit) will be provided along with the read address, concatenated to the end. As in, if the master wants to read addresses 72-80 (0x48-0x50 in hex), it would provide the slave with 0x48_9. This means that 12 bits total will be sent via the read address handshake: 8 bit address, 4 bit control. (Plus transaction ID.)

Your write response should warn if there were any errors or address overflows, and specify how many if so.
*/
`timescale 1 ns / 100 ps

module slave_axi(clk, reset, ARADDR, ARVALID, RREADY, ARREADY, RDATA, RLAST, RVALID, qr_Idle, qr_ReadData, qr_HoldData, AWADDR, AWVALID, AWREADY, WREADY, BRESP, BVALID, qw_Idle, qw_GetData, qw_WriteData, qw_Response, tb_WriteValue, tb_LastWrite, tb_WriteAddress, tb_WriteTransactionID, WLAST, WVALID, BREADY, WDATA);

input   clk, reset, ARVALID, RREADY, ARVALID, RREADY, WLAST, WVALID, BREADY, AWVALID;

input [21:0] ARADDR;
input [17:0] AWADDR;    // 8-bit address, 10 bit transaction id
input [7:0] WDATA;

output reg ARREADY, RLAST, RVALID, AWREADY, WREADY, BVALID, tb_LastWrite;
output qr_Idle, qr_ReadData, qr_HoldData, qw_Idle, qw_GetData, qw_WriteData, qw_Response;

output reg [7:0] RDATA;
output reg [9:0] BRESP; // 2 bits: one hot encoded for OK (2'b01) and NOT OK / WARN (2'b10) ; 8-bits for num of overflows (up to 15)
output reg [7:0] tb_WriteValue;     // Value being written into memory currently
output reg [7:0] tb_WriteAddress;
output reg [9:0] tb_WriteTransactionID;

reg [2:0] read_state;
reg [3:0] write_state;
reg DEBUG;

assign {qr_HoldData, qr_ReadData, qr_Idle} = read_state;
assign {qw_Response, qw_WriteData, qw_GetData, qw_Idle} = write_state;

localparam R_IDLE = 3'b001, R_READDATA = 3'b010, R_HOLDDATA = 3'b100, W_IDLE = 4'b0001, W_GETDATA = 4'b0010, W_WRITEDATA = 4'b0100, W_RESPONSE = 4'b1000;

reg [3:0] NumAddressesLeft;
reg [9:0] ReadTransactionID;
reg [7:0] ReadAddress;
reg ReadOverflow;
reg WriteOverflow;

// Memory: 8-bit wide, 256 locations
integer i;
reg [3:0] NumWriteOverflows;
reg [7:0] ModuleMemory[255:0];

reg [7:0] debug_start_addy;
reg [3:0] debug_num_to_write;

// Slave Read
always @ (posedge clk, posedge reset)
begin
    if (reset)
    begin
        read_state <= R_IDLE;
        RDATA <= 8'bxxxxxxxx;
		NumAddressesLeft <= 4'bX;
		ReadTransactionID <= 10'bX;
		ARREADY <= 1;
		RLAST <= 0;
		RVALID <= 0;
		ReadOverflow <= 0;

		// Used for debugging purposes
		DEBUG <= 0;
    end
    else
        case(read_state)
            R_IDLE:
              begin
				if(DEBUG)
				  begin
					$display("Slave State: READ IDLE\n");
				  end
                if(ARVALID)
                  begin
					if(DEBUG)
					begin
						$display("\tARVALID\n");
					end
                    read_state <= R_READDATA;
                    ReadAddress <= ARADDR[21:14];
                    NumAddressesLeft <= ARADDR[13:10];
                    ReadTransactionID <= ARADDR[9:0];
					ARREADY <= 0;
                  end
              end

            R_READDATA:
            begin
				if(DEBUG)
				begin
					$display("Slave State: READ DATA\n");
				end
				RVALID <= 1;
                if (ReadOverflow)
                  begin
                    // Send a dummy value if reading out of bounds
                    RDATA <= 8'bxxxxxxxx;
                  end
                else
                  begin
				    if(DEBUG)
						$display("~~~~~~Reading from memory address %d, value: %d", ReadAddress, ModuleMemory[ReadAddress]);
                    RDATA <= ModuleMemory[ReadAddress];
                  end
                NumAddressesLeft <= NumAddressesLeft - 8'b1;
                if (NumAddressesLeft == 8'b1)
                    RLAST <= 1;

                read_state <= R_HOLDDATA;
            end

            R_HOLDDATA:
            begin
				if(DEBUG)
				begin
				$display("Slave State: READ HOLD DATA\n");
				end
                if (RREADY)
                begin
					if(DEBUG)
					  begin
						$display("\tRREADY\n");
					  end
                    if (RLAST)
                      begin
						if(DEBUG)
						begin
							$display("\RLAST\n");
						end
						RVALID <= 0;
						RLAST <= 0;
						ARREADY <= 1;
                        read_state <= R_IDLE;
                      end
                    else
                      begin
                        if (ReadAddress == 8'b11111111)
                          begin
                            // Overflow!
                            ReadOverflow = 1;
                          end
                        else
                          begin
                            // Increment address!
                            ReadAddress <= ReadAddress + 8'b00000001;
                          end
                        read_state <= R_READDATA;
						RVALID <= 0;
                      end
                end

            end
		endcase
end

integer j;

// Slave Write
always @ (posedge clk, posedge reset)
begin
    if (reset)
      begin
        tb_WriteValue <= 8'bx;
        tb_WriteAddress <= 8'bx;
        tb_WriteTransactionID <= 10'bx;
        BRESP <= 10'bx;
        NumWriteOverflows <= 4'b0;
        // Set memory to all zeros
        for (i = 0; i < 256; i = i + 1)
          begin
            ModuleMemory[i] <= 8'b0;
          end
        write_state <= W_IDLE;
		AWREADY <= 1;
		WREADY <= 0;
		BVALID <= 0;
		tb_LastWrite <= 0;
		WriteOverflow <= 0;
		// Used for debugging purposes
		DEBUG <= 0;
      end
    else
        case(write_state)
        W_IDLE:
          begin
		    if(DEBUG)
				$display("------SLAVE IDLE------\n");
            if(AWVALID)
              begin
                write_state <= W_GETDATA;
                tb_WriteAddress <= AWADDR[17:10];
				debug_start_addy <= AWADDR[17:10];
				debug_num_to_write <= 4'b0;
                tb_WriteTransactionID <= AWADDR[9:0];
				WREADY <= 1;
				AWREADY <= 0;
              end
          end

        W_GETDATA:
          begin
			if(DEBUG)
				$display("------SLAVE GETDATA------\n");
            if(WVALID)
              begin
                write_state <= W_WRITEDATA;
                tb_WriteValue <= WDATA;
                tb_LastWrite <= WLAST;
				WREADY <= 0;
              end
          end

        W_WRITEDATA:
          begin
			if (DEBUG)
				$display("------SLAVE WRITEDATA------\n");
            if (WriteOverflow == 1'b0)
              begin
			    debug_num_to_write <= debug_num_to_write + 4'b1;
                ModuleMemory[tb_WriteAddress] <= tb_WriteValue;
				if(DEBUG)
					$display("------Writing %d to Address %d------\n", tb_WriteValue, tb_WriteAddress);
                if (tb_WriteAddress == 8'b11111111)
                  begin
                    WriteOverflow <= 1;
                  end
                else
                  begin
                    tb_WriteAddress <= tb_WriteAddress + 8'b00000001;
                  end
              end
            else
              begin
                NumWriteOverflows <= NumWriteOverflows + 4'b0001;
              end

            if (tb_LastWrite)
              begin
                write_state <= W_RESPONSE;
				BVALID <= 1;
                if (WriteOverflow)
                  begin
                    // Warn
                    BRESP[9:8] <= 2'b10;
                    BRESP[7:0] <= (NumWriteOverflows + 4'b0001);
                  end
                else
                  begin
                    BRESP <= 10'b0;
                    BRESP[9:8] <= 2'b01;
                  end
              end
            else
              begin
                write_state <= W_GETDATA;
				WREADY <= 1;
              end
          end

        W_RESPONSE:
        begin
		    if(DEBUG)
			  begin
				$display("------SLAVE RESPONSE------\n");
				$display("** MEMORY **\n");
				for(j = 0; j < debug_num_to_write; j = j + 1)
				  begin
					$display("**Address: %d, Value: %d", debug_start_addy + j, ModuleMemory[j + debug_start_addy]);
				  end
		      end

            if(BREADY)
			  begin
                write_state <= W_IDLE;
				BVALID <= 0;
				AWREADY <= 1;
				WriteOverflow <= 0;
				tb_LastWrite <= 0;
			  end
        end
	endcase
end



endmodule