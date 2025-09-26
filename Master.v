/*
This module implements the master implementation of the AXI Protocol. This code will support read and write requests.

Addresses will be 8-bit, accessing a memory space of 256 8-bit values (contained within the slave module). When an 8-bit key is provided, you must return the 8-bit value corresponding to that location. All memories start at zeroes, upon a reset.

You must include a space for transaction IDs, sent along with addresses.

Your modules must allow burst-based writing and reading:

— BURST WRITE: When multiple 8-bit data values are provided sequentially, you must write to the provided write address, then the next address in order, and so on, until WLAST is asserted. If this would force an address overflow (as in, you’re requested to write to a location past the maximum address of the memory), ignore them: do not over-write the low-addressed locations.

— BURST READ: When asking for a read, in addition to the read address, the master module will specify a number of distinct memory addresses to be read (up to 16), starting at the provided address and continuing sequentially. This number of requested addresses (4 bit) will be provided along with the read address, concatenated to the end. As in, if the master wants to read addresses 72-80 (0x48-0x50 in hex), it would provide the slave with 0x48_9. This means that 12 bits total will be sent via the read address handshake: 8 bit address, 4 bit control. (Plus transaction ID.)

Your write response should warn if there were any errors or address overflows, and specify how many if so.
*/
`timescale 1 ns / 100 ps


module master_axi(clk, reset, ARREADY, RDATA, RVALID, AWREADY, WREADY, BRESP, BVALID, AWADDR, AWVALID, WDATA, WLAST, WVALID, BREADY, ARADDR, ARVALID, RREADY, qr_Idle, qr_ARValid, qr_ReadReady, qr_Read, qw_Idle, qw_AWValid, qw_GetDataToWrite, qw_Write, qw_WaitForResponse, tb_ReadRequest, tb_ReadAddress, tb_NumAddressesToRead, tb_ReadTransactionID, tb_ReadValue, tb_WriteRequest, tb_WriteAddress, tb_WriteTransactionID, tb_WriteDataValid, tb_WriteValue, tb_WriteLast, RLAST);

input   clk, reset, ARREADY, RVALID, AWREADY, WREADY, BVALID, tb_ReadRequest, tb_WriteRequest, tb_WriteDataValid, tb_WriteLast, RLAST;
input [7:0] RDATA;  // 8-bit data values
input [9:0] BRESP;  // 2 bits: one hot encoded for OK (2'b01) and NOT OK / WARN (2'b10) ; 4-bits for num of overflows (up to 15)
input [7:0] tb_ReadAddress;
input [3:0] tb_NumAddressesToRead;
input [9:0] tb_ReadTransactionID;
input [7:0] tb_WriteAddress;   // 8-bit address
input [9:0] tb_WriteTransactionID;
input [7:0] tb_WriteValue;

output reg AWVALID, WLAST, WVALID, BREADY, ARVALID, RREADY;
output qr_Idle, qr_ARValid, qr_ReadReady, qr_Read, qw_Idle, qw_AWValid, qw_GetDataToWrite, qw_Write, qw_WaitForResponse;
output reg [17:0] AWADDR;   // 8-bit address, 10 transaction id bits
output reg [21:0] ARADDR;  // 8 bit address, 4 control bits, 10 bits transaction ID
output reg [7:0] WDATA;
output reg [7:0] tb_ReadValue;  // 8-bit data values

reg [3:0] read_state;
reg [4:0] write_state;


assign {qr_Read, qr_ReadReady, qr_ARValid, qr_Idle} = read_state;
assign {qw_WaitForResponse, qw_Write, qw_GetDataToWrite, qw_AWValid, qw_Idle} = write_state;

localparam R_IDLE = 4'b0001, R_ARVALID = 4'b0010, R_READREADY = 4'b0100, R_READ = 4'b1000, W_IDLE = 5'b00001, W_AWVALID = 5'b00010, W_GETDATATOWRITE = 5'b00100, W_WRITE = 5'b01000, W_WAITFORRESPONSE = 5'b10000;

reg [3:0] NumAddressesLeft;
reg DEBUG;

// Master Read
always @ (posedge clk, posedge reset)
begin
    if(reset)
      begin
        read_state <= R_IDLE;
        NumAddressesLeft <= 4'b0;
        ARADDR <= 22'bx;
        tb_ReadValue <= 8'bx;
		ARVALID <= 0;
        RREADY <= 0;
		// For debugging
		DEBUG <= 0;
      end
    else
        case(read_state)
            R_IDLE:
            begin
				if(DEBUG)
				  begin
					$display("\nMaster State: READ IDLE\n");
				  end
                if (tb_ReadRequest)
                begin
					if(DEBUG)
					begin
						$display("\tRead Request\n");
					end
                    read_state <= R_ARVALID;
                    ARADDR <= {tb_ReadAddress, tb_NumAddressesToRead, tb_ReadTransactionID};
                    NumAddressesLeft <= tb_NumAddressesToRead;
					ARVALID <= 1;
                end
            end

            R_ARVALID:
            begin
				if(DEBUG)
				begin
					$display("\nMaster State: READ ARVALID\n");
				end
                if (ARREADY)
				  begin
                    read_state <= R_READREADY;
					ARVALID <= 0;
					RREADY <= 1;
				  end
            end

            R_READREADY:
            begin
				if(DEBUG)
				begin
					$display("\nMaster State: READ READY\n");
				end
                if (RVALID)
                  begin
                    tb_ReadValue <= RDATA;
                    read_state <= R_READ;
					RREADY <= 0;
                  end
            end

            R_READ:
            begin
				if(DEBUG)
				begin
					$display("\nMaster State: READ\n");
				end
                NumAddressesLeft <= NumAddressesLeft - 1;

                if (NumAddressesLeft == 4'd1)
                  begin
                    read_state <= R_IDLE;
                  end
				else
				  begin
					read_state <= R_READREADY;
					RREADY <= 1;
				  end
            end
		endcase
end

// Master Write
always @ (posedge clk, posedge reset)
begin
    if(reset)
    begin
        write_state <= W_IDLE;
        AWADDR <= 18'bx;
        WDATA <= 8'bx;
		BREADY <= 0;
		AWVALID <= 0;
		WLAST <= 0;
		WVALID <= 0;
		DEBUG <= 0;
    end
    else
        case(write_state)
        W_IDLE:
        begin
			if(DEBUG)
				$display("------MASTER WRITE IDLE------\n");
            if (tb_WriteRequest)
            begin
                write_state <= W_AWVALID;
                AWADDR <= {tb_WriteAddress, tb_WriteTransactionID};
				AWVALID <= 1;
            end
        end

        W_AWVALID:
        begin
			if(DEBUG)
				$display("------MASTER WRITE AWVALID------\n");
            if (AWREADY)
            begin
                write_state <= W_GETDATATOWRITE;
				AWVALID <= 0;
            end
        end

        W_GETDATATOWRITE:
          begin
			if(DEBUG)
				$display("------MASTER WRITE GETDATATOWRITE------\n");
            if (tb_WriteDataValid)
              begin
                WDATA <= tb_WriteValue;
                write_state <= W_WRITE;
                WLAST <= tb_WriteLast;

				WVALID <= 1;
              end
          end

        W_WRITE:
          begin
			if(DEBUG)
				$display("------MASTER WRITE WRITE------\n");
            if (WREADY)
              begin
				WVALID <= 0;
				WLAST <= 0;
                if (WLAST)
				  begin
                    write_state <= W_WAITFORRESPONSE;
					BREADY <= 1;
				  end
                else
                    write_state <= W_GETDATATOWRITE;
              end
          end

        W_WAITFORRESPONSE:
          begin
			if(DEBUG)
				$display("------MASTER WRITE WAITFORRESPONSE------\n");
            if (BVALID)
              begin
                write_state <= W_IDLE;
				BREADY <= 0;
				WLAST <= 0;
              end
          end
	endcase

end



endmodule