/*
 * controller_stress_tb -- randomized and corner-case stress test
 *
 * Drives thousands of instructions into the controller and checks the
 * scheduling rules on every task, instead of comparing against a fixed log.
 *
 * A monitor watches each unit_scheduler. On every task START it checks:
 *   - the task is addressed to that unit, and wasn't started before
 *   - ORDERING: no older instruction with the same ID is still unfinished
 *   - PRIORITY: the unit didn't skip an older instruction that was ready for it
 * On every task COMPLETE it checks:
 *   - the task was started and completes exactly once
 *   - DATA: a read returns the value predicted by a reference model, and a
 *     write returns BRESP = OK
 * It also fails on deadlock (no progress for a long time) and checks that the
 * queue is empty at the end.
 *
 * The reference model works despite out-of-order execution because every
 * address belongs to exactly one transaction ID (address[7:4] = ID index), so
 * all accesses to an address are ordered.
 *
 * Timing note: a scheduler that starts a task at clock edge E saw the done
 * bits as they were before E. A completion pulse in the cycle before edge E1
 * sets its done bit at E1, so it only unblocks starts at edges after E1. The
 * monitor records that edge (done_edge) and uses it for its checks.
 *
 * Plusargs: +seed=<n> (default 1)  +n=<random instructions> (default 4000)
 *           +ids=<distinct IDs in random phase, 1-16> (default 6)
 * Parameters (set with iverilog -P): ALU_LAT, MEM_LAT, IO_LAT (slave latency)
 *
 * Prints "STRESS PASS" or "STRESS FAIL" at the end.
 */

`timescale 1ns / 100ps

module controller_stress_tb;

parameter ALU_LAT = 4;
parameter MEM_LAT = 14;
parameter IO_LAT  = 24;

localparam MAX    = 8192;   // Max instructions per run
localparam WINDOW = 64;     // How far back to look for older instructions (> queue depth)

reg clk, reset;
reg tb_IsNewInstruction;
reg [29:0] tb_NewInstruction;
reg dependency_test;

// ---------------------------------------------------------------------------
// Design under test: controller + three slaves
// ---------------------------------------------------------------------------

wire FIFO_IsEmpty;
wire ALU_Q_FINDTASK, ALU_Q_PROCESSTASK, MEMORY_Q_FINDTASK, MEMORY_Q_PROCESSTASK, IO_Q_FINDTASK,
     IO_Q_PROCESSTASK, FIFO_Q_FULL, FIFO_Q_GOTVALUE, FIFO_Q_NOTFULL;

wire ALU_AWVALID, ALU_WLAST, ALU_WVALID, ALU_BREADY, ALU_ARVALID, ALU_RREADY;
wire [17:0] ALU_AWADDR;
wire [21:0] ALU_ARADDR;
wire [7:0]  ALU_WDATA, ALU_ReadValue;
wire Memory_AWVALID, Memory_WLAST, Memory_WVALID, Memory_BREADY, Memory_ARVALID, Memory_RREADY;
wire [17:0] Memory_AWADDR;
wire [21:0] Memory_ARADDR;
wire [7:0]  Memory_WDATA, Memory_ReadValue;
wire IO_AWVALID, IO_WLAST, IO_WVALID, IO_BREADY, IO_ARVALID, IO_RREADY;
wire [17:0] IO_AWADDR;
wire [21:0] IO_ARADDR;
wire [7:0]  IO_WDATA, IO_ReadValue;

wire ALU_ARREADY, ALU_RVALID, ALU_AWREADY, ALU_WREADY, ALU_BVALID, ALU_tb_LastWrite, ALU_RLAST;
wire [9:0] ALU_BRESP;
wire [7:0] ALU_RDATA, ALU_tb_WriteValue, ALU_tb_WriteAddress;
wire [9:0] ALU_tb_WriteTransactionID;
wire Memory_ARREADY, Memory_RVALID, Memory_AWREADY, Memory_WREADY, Memory_BVALID,
     Memory_tb_LastWrite, Memory_RLAST;
wire [9:0] Memory_BRESP;
wire [7:0] Memory_RDATA, Memory_tb_WriteValue, Memory_tb_WriteAddress;
wire [9:0] Memory_tb_WriteTransactionID;
wire IO_ARREADY, IO_RVALID, IO_AWREADY, IO_WREADY, IO_BVALID, IO_tb_LastWrite, IO_RLAST;
wire [9:0] IO_BRESP;
wire [7:0] IO_RDATA, IO_tb_WriteValue, IO_tb_WriteAddress;
wire [9:0] IO_tb_WriteTransactionID;

controller controller_uut(
    // Clock and reset
    .clk(clk), .reset(reset),
    // Instruction interface
    .tb_IsNewInstruction(tb_IsNewInstruction), .tb_NewInstruction(tb_NewInstruction),
    .dependency_test(dependency_test),
    // Queue status
    .FIFO_Q_NOTFULL(FIFO_Q_NOTFULL), .FIFO_Q_GOTVALUE(FIFO_Q_GOTVALUE), .FIFO_Q_FULL(FIFO_Q_FULL),
    .FIFO_IsEmpty(FIFO_IsEmpty),
    // Scheduler state per unit
    .ALU_Q_FINDTASK(ALU_Q_FINDTASK), .ALU_Q_PROCESSTASK(ALU_Q_PROCESSTASK),
    .MEMORY_Q_FINDTASK(MEMORY_Q_FINDTASK), .MEMORY_Q_PROCESSTASK(MEMORY_Q_PROCESSTASK),
    .IO_Q_FINDTASK(IO_Q_FINDTASK), .IO_Q_PROCESSTASK(IO_Q_PROCESSTASK),
    // ALU
    .ALU_ARVALID(ALU_ARVALID), .ALU_ARREADY(ALU_ARREADY), .ALU_ARADDR(ALU_ARADDR),
    .ALU_RVALID(ALU_RVALID), .ALU_RREADY(ALU_RREADY), .ALU_RDATA(ALU_RDATA), .ALU_RLAST(ALU_RLAST),
    .ALU_AWVALID(ALU_AWVALID), .ALU_AWREADY(ALU_AWREADY), .ALU_AWADDR(ALU_AWADDR),
    .ALU_WVALID(ALU_WVALID), .ALU_WREADY(ALU_WREADY), .ALU_WDATA(ALU_WDATA), .ALU_WLAST(ALU_WLAST),
    .ALU_BVALID(ALU_BVALID), .ALU_BREADY(ALU_BREADY), .ALU_BRESP(ALU_BRESP),
    .ALU_ReadValue(ALU_ReadValue),
    // Memory
    .Memory_ARVALID(Memory_ARVALID), .Memory_ARREADY(Memory_ARREADY), .Memory_ARADDR(Memory_ARADDR),
    .Memory_RVALID(Memory_RVALID), .Memory_RREADY(Memory_RREADY), .Memory_RDATA(Memory_RDATA),
    .Memory_RLAST(Memory_RLAST), .Memory_AWVALID(Memory_AWVALID), .Memory_AWREADY(Memory_AWREADY),
    .Memory_AWADDR(Memory_AWADDR), .Memory_WVALID(Memory_WVALID), .Memory_WREADY(Memory_WREADY),
    .Memory_WDATA(Memory_WDATA), .Memory_WLAST(Memory_WLAST), .Memory_BVALID(Memory_BVALID),
    .Memory_BREADY(Memory_BREADY), .Memory_BRESP(Memory_BRESP), .Memory_ReadValue(Memory_ReadValue),
    // I/O
    .IO_ARVALID(IO_ARVALID), .IO_ARREADY(IO_ARREADY), .IO_ARADDR(IO_ARADDR), .IO_RVALID(IO_RVALID),
    .IO_RREADY(IO_RREADY), .IO_RDATA(IO_RDATA), .IO_RLAST(IO_RLAST), .IO_AWVALID(IO_AWVALID),
    .IO_AWREADY(IO_AWREADY), .IO_AWADDR(IO_AWADDR), .IO_WVALID(IO_WVALID), .IO_WREADY(IO_WREADY),
    .IO_WDATA(IO_WDATA), .IO_WLAST(IO_WLAST), .IO_BVALID(IO_BVALID), .IO_BREADY(IO_BREADY),
    .IO_BRESP(IO_BRESP), .IO_ReadValue(IO_ReadValue)
);

slave_axi #(.LATENCY(ALU_LAT)) ALU_slave(
    .clk(clk), .reset(reset),
    .ARADDR(ALU_ARADDR), .ARVALID(ALU_ARVALID), .RREADY(ALU_RREADY), .ARREADY(ALU_ARREADY),
    .RDATA(ALU_RDATA), .RLAST(ALU_RLAST), .RVALID(ALU_RVALID),
    .qr_Idle(), .qr_ReadData(), .qr_HoldData(),
    .AWADDR(ALU_AWADDR), .AWVALID(ALU_AWVALID), .AWREADY(ALU_AWREADY), .WREADY(ALU_WREADY),
    .BRESP(ALU_BRESP), .BVALID(ALU_BVALID),
    .qw_Idle(), .qw_GetData(), .qw_WriteData(), .qw_Response(),
    .tb_WriteValue(ALU_tb_WriteValue), .tb_LastWrite(ALU_tb_LastWrite),
    .tb_WriteAddress(ALU_tb_WriteAddress), .tb_WriteTransactionID(ALU_tb_WriteTransactionID),
    .WLAST(ALU_WLAST), .WVALID(ALU_WVALID), .BREADY(ALU_BREADY), .WDATA(ALU_WDATA)
);

slave_axi #(.LATENCY(MEM_LAT)) Memory_slave(
    .clk(clk), .reset(reset),
    .ARADDR(Memory_ARADDR), .ARVALID(Memory_ARVALID), .RREADY(Memory_RREADY),
    .ARREADY(Memory_ARREADY), .RDATA(Memory_RDATA), .RLAST(Memory_RLAST),
    .RVALID(Memory_RVALID),
    .qr_Idle(), .qr_ReadData(), .qr_HoldData(),
    .AWADDR(Memory_AWADDR), .AWVALID(Memory_AWVALID), .AWREADY(Memory_AWREADY),
    .WREADY(Memory_WREADY), .BRESP(Memory_BRESP), .BVALID(Memory_BVALID),
    .qw_Idle(), .qw_GetData(), .qw_WriteData(), .qw_Response(),
    .tb_WriteValue(Memory_tb_WriteValue), .tb_LastWrite(Memory_tb_LastWrite),
    .tb_WriteAddress(Memory_tb_WriteAddress),
    .tb_WriteTransactionID(Memory_tb_WriteTransactionID),
    .WLAST(Memory_WLAST), .WVALID(Memory_WVALID), .BREADY(Memory_BREADY), .WDATA(Memory_WDATA)
);

slave_axi #(.LATENCY(IO_LAT)) IO_slave(
    .clk(clk), .reset(reset),
    .ARADDR(IO_ARADDR), .ARVALID(IO_ARVALID), .RREADY(IO_RREADY), .ARREADY(IO_ARREADY),
    .RDATA(IO_RDATA), .RLAST(IO_RLAST), .RVALID(IO_RVALID),
    .qr_Idle(), .qr_ReadData(), .qr_HoldData(),
    .AWADDR(IO_AWADDR), .AWVALID(IO_AWVALID), .AWREADY(IO_AWREADY), .WREADY(IO_WREADY),
    .BRESP(IO_BRESP), .BVALID(IO_BVALID),
    .qw_Idle(), .qw_GetData(), .qw_WriteData(), .qw_Response(),
    .tb_WriteValue(IO_tb_WriteValue), .tb_LastWrite(IO_tb_LastWrite),
    .tb_WriteAddress(IO_tb_WriteAddress), .tb_WriteTransactionID(IO_tb_WriteTransactionID),
    .WLAST(IO_WLAST), .WVALID(IO_WVALID), .BREADY(IO_BREADY), .WDATA(IO_WDATA)
);

// ---------------------------------------------------------------------------
// Scoreboard: one record per instruction, indexed by send order (seq)
// ---------------------------------------------------------------------------

reg [9:0] id_of     [0:MAX-1];
reg [1:0] unit_of   [0:MAX-1];
reg       op_of     [0:MAX-1];   // 0 = read, 1 = write
reg [7:0] exp_data  [0:MAX-1];   // Expected read value
reg       started   [0:MAX-1];
reg       completed [0:MAX-1];
integer   accept_edge [0:MAX-1]; // Edge at which the queue accepted it
integer   done_edge   [0:MAX-1]; // Edge at which its done bit is set

integer slot_seq [0:15];         // Instruction currently in each buffer slot
integer cur_task [0:2];          // Instruction each unit is working on
reg     prev_proc [0:2];         // Unit was in PROCESSTASK at the previous check

reg [7:0] ref_mem [0:767];       // Reference memories: unit*256 + address

integer edge_n;                  // Index of the most recent rising clock edge
integer nsent, ncompleted, errors;
integer seed, seed0, n_random, num_ids;
integer stat_ooo, stat_same_cycle, stat_max_occ, stat_full_cycles;
integer idle_cycles;
reg     running;

// Clock: 4 ns period, edge counter updated before any design logic sees the edge
initial clk = 0;
always #2 clk = ~clk;
always @(posedge clk) edge_n = edge_n + 1;

// ---------------------------------------------------------------------------
// Rule checks
// ---------------------------------------------------------------------------

// Instruction t was finished early enough to be visible to a start at edge e
function finished_before;
    input integer t;
    input integer e;
    begin
        finished_before = completed[t] && (done_edge[t] < e);
    end
endfunction

// Instruction t had no unfinished older same-ID instruction at edge e
function ready_at;
    input integer t;
    input integer e;
    integer v;
    begin
        ready_at = 1'b1;
        for (v = t - 1; v >= 0 && v >= t - WINDOW; v = v - 1)
            if (id_of[v] == id_of[t] && !finished_before(v, e))
                ready_at = 1'b0;
    end
endfunction

task fail;
    input integer seq;
    begin
        errors = errors + 1;
        if (errors >= 20)
        begin
            $display("Too many errors; stopping.");
            finish_test;
        end
    end
endtask

task on_start;
    input integer u;
    input integer s;
    integer t, e;
    reg skipped;
    begin
        e = edge_n;
        if (s < 0)
        begin
            $display("[STRESS ERROR] unit %0d started a task in an empty slot", u);
            fail(s);
        end
        else
        begin
            if (unit_of[s] != u)
            begin
                $display("[STRESS ERROR] seq %0d (unit %0d) started on unit %0d", s, unit_of[s], u);
                fail(s);
            end
            if (started[s])
            begin
                $display("[STRESS ERROR] seq %0d started twice", s);
                fail(s);
            end
            if (accept_edge[s] >= e)
            begin
                $display("[STRESS ERROR] seq %0d started before it was accepted", s);
                fail(s);
            end

            // ORDERING: no older same-ID instruction may still be unfinished
            for (t = s - 1; t >= 0 && t >= s - WINDOW; t = t - 1)
                if (id_of[t] == id_of[s] && !finished_before(t, e))
                begin
                    $display("[STRESS ERROR] ORDERING: seq %0d (ID %0d) started at edge %0d while older seq %0d with the same ID was unfinished",
                             s, id_of[s], e, t);
                    fail(s);
                end

            // PRIORITY: no older ready instruction for this unit may be skipped
            skipped = 1'b0;
            for (t = s - 1; t >= 0 && t >= s - WINDOW; t = t - 1)
            begin
                if (unit_of[t] == u && !started[t] && accept_edge[t] < e && ready_at(t, e))
                begin
                    $display("[STRESS ERROR] PRIORITY: unit %0d started seq %0d but older seq %0d was ready",
                             u, s, t);
                    fail(s);
                end
                if (!started[t])
                    skipped = 1'b1;
            end
            if (skipped)
                stat_ooo = stat_ooo + 1;   // Started ahead of an older instruction

            started[s]  = 1'b1;
            cur_task[u] = s;
        end
    end
endtask

task on_complete;
    input integer u;
    input integer s;
    input [7:0] rdata;
    input [9:0] bresp;
    begin
        if (s < 0 || s != cur_task[u])
        begin
            $display("[STRESS ERROR] unit %0d completed slot holding seq %0d, but is working on seq %0d",
                     u, s, cur_task[u]);
            fail(s);
        end
        else if (!started[s] || completed[s])
        begin
            $display("[STRESS ERROR] seq %0d completed without starting, or twice", s);
            fail(s);
        end
        else
        begin
            completed[s] = 1'b1;
            done_edge[s] = edge_n + 1;   // Done bit is set at the next edge
            ncompleted   = ncompleted + 1;

            if (op_of[s] == 1'b0 && rdata !== exp_data[s])
            begin
                $display("[STRESS ERROR] DATA: seq %0d read %0d, expected %0d", s, rdata, exp_data[s]);
                fail(s);
            end
            if (op_of[s] == 1'b1 && bresp !== 10'h100)
            begin
                $display("[STRESS ERROR] DATA: seq %0d write response %h, expected 100", s, bresp);
                fail(s);
            end
        end
    end
endtask

// Monitor: sample in the middle of each cycle, when all signals are stable
always @(negedge clk)
    if (running)
    begin : monitor
        integer done_count;

        // Task starts (a scheduler entered PROCESSTASK at the last edge)
        if (ALU_Q_PROCESSTASK && !prev_proc[0])
            on_start(0, slot_seq[controller_uut.ALU_Scheduler.slot]);
        if (MEMORY_Q_PROCESSTASK && !prev_proc[1])
            on_start(1, slot_seq[controller_uut.Memory_Scheduler.slot]);
        if (IO_Q_PROCESSTASK && !prev_proc[2])
            on_start(2, slot_seq[controller_uut.IO_Scheduler.slot]);
        prev_proc[0] = ALU_Q_PROCESSTASK;
        prev_proc[1] = MEMORY_Q_PROCESSTASK;
        prev_proc[2] = IO_Q_PROCESSTASK;

        // Task completions (done pulse this cycle)
        done_count = 0;
        if (controller_uut.alu_done)
        begin
            on_complete(0, slot_seq[controller_uut.alu_done_slot], ALU_RDATA, ALU_BRESP);
            done_count = done_count + 1;
        end
        if (controller_uut.mem_done)
        begin
            on_complete(1, slot_seq[controller_uut.mem_done_slot], Memory_RDATA, Memory_BRESP);
            done_count = done_count + 1;
        end
        if (controller_uut.io_done)
        begin
            on_complete(2, slot_seq[controller_uut.io_done_slot], IO_RDATA, IO_BRESP);
            done_count = done_count + 1;
        end

        // Statistics
        if (done_count >= 2)
            stat_same_cycle = stat_same_cycle + 1;
        if (controller_uut.count > stat_max_occ)
            stat_max_occ = controller_uut.count;
        if (FIFO_Q_FULL)
            stat_full_cycles = stat_full_cycles + 1;

        // Deadlock watchdog
        if (done_count == 0 && ncompleted < nsent)
            idle_cycles = idle_cycles + 1;
        else
            idle_cycles = 0;
        if (idle_cycles > 5000)
        begin
            $display("[STRESS ERROR] DEADLOCK: no task completed in 5000 cycles (%0d of %0d done)",
                     ncompleted, nsent);
            errors = errors + 1;
            finish_test;
        end
    end

// ---------------------------------------------------------------------------
// Stimulus
// ---------------------------------------------------------------------------

// ID for index k (0-15). Spreads bits across the 10-bit field so IDs that
// differ only in their high bits are exercised.
function [9:0] id_for;
    input [3:0] k;
    begin
        id_for = {k[3:2], 6'b000000, k[1:0]};
    end
endfunction

// Send one instruction and record it in the scoreboard once it's accepted
task send;
    input [1:0] unit;
    input       op;
    input [3:0] k;          // ID index; also selects the address range
    input [3:0] offset;     // Address within the ID's range
    input [7:0] value;
    reg   [7:0] addr;
    begin
        if (nsent >= MAX)
        begin
            $display("[STRESS ERROR] too many instructions for the scoreboard");
            errors = errors + 1;
            finish_test;
        end
        addr = {k, offset};

        @(negedge clk);
        tb_NewInstruction   = {value, addr, unit, op, id_for(k), 1'b0};
        tb_IsNewInstruction = 1'b1;
        wait (FIFO_Q_GOTVALUE);

        // Accepted at the edge that just happened
        accept_edge[nsent] = edge_n;
        id_of[nsent]       = id_for(k);
        unit_of[nsent]     = unit;
        op_of[nsent]       = op;
        if (op)
        begin
            ref_mem[unit*256 + addr] = value;
            exp_data[nsent] = 8'd0;
        end
        else
            exp_data[nsent] = ref_mem[unit*256 + addr];
        slot_seq[nsent % 16] = nsent;
        nsent = nsent + 1;

        @(negedge clk);
        tb_IsNewInstruction = 1'b0;
    end
endtask

task idle;
    input integer cycles;
    begin
        repeat (cycles) @(negedge clk);
    end
endtask

task wait_drain;
    begin
        while (ncompleted < nsent)
            @(negedge clk);
        idle(4);
    end
endtask

task finish_test;
    begin
        running = 1'b0;
        $display("");
        $display("Stress summary (seed %0d, latencies ALU=%0d Memory=%0d IO=%0d)",
                 seed0, ALU_LAT, MEM_LAT, IO_LAT);
        $display("  Instructions sent / completed:    %0d / %0d", nsent, ncompleted);
        $display("  Started ahead of older ones:      %0d", stat_ooo);
        $display("  Cycles with 2+ completions:       %0d", stat_same_cycle);
        $display("  Max queue occupancy:              %0d", stat_max_occ);
        $display("  Cycles with queue full:           %0d", stat_full_cycles);
        $display("  Errors:                           %0d", errors);
        if (errors == 0 && ncompleted == nsent)
            $display("STRESS PASS");
        else
            $display("STRESS FAIL");
        $finish;
    end
endtask

integer i, r;
reg [1:0] unit;
reg [3:0] k;

initial
begin
    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    seed0 = seed;   // $random updates seed; keep the original for the report
    if (!$value$plusargs("n=%d", n_random)) n_random = 4000;
    if (!$value$plusargs("ids=%d", num_ids)) num_ids = 6;
    if (num_ids < 1)  num_ids = 1;
    if (num_ids > 16) num_ids = 16;
    if (n_random > MAX - 500) n_random = MAX - 500;

    for (i = 0; i < MAX; i = i + 1)
    begin
        started[i] = 0; completed[i] = 0; accept_edge[i] = 0; done_edge[i] = 0;
    end
    for (i = 0; i < 768; i = i + 1) ref_mem[i] = 8'd0;
    for (i = 0; i < 16; i = i + 1)  slot_seq[i] = -1;
    for (i = 0; i < 3; i = i + 1)
    begin
        cur_task[i] = -1; prev_proc[i] = 0;
    end
    edge_n = 0; nsent = 0; ncompleted = 0; errors = 0; idle_cycles = 0;
    stat_ooo = 0; stat_same_cycle = 0; stat_max_occ = 0; stat_full_cycles = 0;
    dependency_test = 0;
    tb_IsNewInstruction = 0;
    tb_NewInstruction = 30'd0;
    running = 0;

    reset = 0;
    @(posedge clk); #1 reset = 1;
    @(posedge clk); #1 reset = 0;
    running = 1;

    // 1. One unit at a time: every instruction goes to the ALU
    $display("Phase 1: single unit");
    for (i = 0; i < 40; i = i + 1)
    begin
        r = $random(seed);
        send(2'd0, r[0], {$random(seed)} % 4, r[7:4], r[15:8]);
    end
    wait_drain;

    // 2. All instructions share one ID: execution must be fully serialized
    $display("Phase 2: single ID across all units");
    for (i = 0; i < 40; i = i + 1)
    begin
        r = $random(seed);
        send({$random(seed)} % 3, r[0], 4'd5, r[7:4], r[15:8]);
    end
    wait_drain;

    // 3. Full queue behind a blocked head: a slow I/O task, then ALU work that
    //    alternates between its ID (blocked) and independent IDs
    $display("Phase 3: full queue with a blocked head");
    for (i = 0; i < 4; i = i + 1)
    begin
        r = $random(seed);
        send(2'd2, 1'b1, 4'd0, r[7:4], r[15:8]);
        repeat (24)
        begin
            r = $random(seed);
            k = r[1] ? 4'd0 : 4'd1 + ({$random(seed)} % 3);
            send(2'd0, r[0], k, r[7:4], r[15:8]);
        end
    end
    wait_drain;

    // 4. Back-to-back independent work on all three units
    $display("Phase 4: three units in parallel");
    for (i = 0; i < 60; i = i + 1)
    begin
        r = $random(seed);
        send(i % 3, r[0], 4'd8 + (i % 3) * 2 + r[1], r[7:4], r[15:8]);
    end
    wait_drain;

    // 5. Random traffic: random units, IDs, and gaps that let the queue
    //    alternate between draining and filling
    $display("Phase 5: %0d random instructions over %0d IDs", n_random, num_ids);
    for (i = 0; i < n_random; i = i + 1)
    begin
        r = $random(seed);
        unit = {$random(seed)} % 3;
        k = {$random(seed)} % num_ids;
        send(unit, r[0], k, r[11:8], r[23:16]);
        if (r[30:28] == 3'd0)
            idle({$random(seed)} % 48);
    end
    wait_drain;

    // The queue retires one completed entry per cycle, so up to 16 finished
    // entries can still be waiting to retire when the last task completes
    i = 0;
    while (!FIFO_IsEmpty && i < 32)
    begin
        @(negedge clk);
        i = i + 1;
    end
    if (!FIFO_IsEmpty)
    begin
        $display("[STRESS ERROR] queue not empty 32 cycles after all instructions completed");
        errors = errors + 1;
    end
    else
        $display("Queue empty %0d cycles after the final drain", i);
    finish_test;
end

endmodule
