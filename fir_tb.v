


`timescale 1ns / 1ps

`define CYCLE_TIME 10
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/20/2023 10:38:55 AM
// Design Name: 
// Module Name: fir_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module fir_tb
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter Data_Num    = 600
)();
    wire                        awready;
    wire                        wready;
    reg                         awvalid;
    reg   [(pADDR_WIDTH-1): 0]  awaddr;
    reg                         wvalid;
    reg signed [(pDATA_WIDTH-1) : 0] wdata;
    wire                        arready;
    reg                         rready;
    reg                         arvalid;
    reg         [(pADDR_WIDTH-1): 0] araddr;
    wire                        rvalid;
    wire signed [(pDATA_WIDTH-1): 0] rdata;
    reg                         ss_tvalid;
    reg signed [(pDATA_WIDTH-1) : 0] ss_tdata;
    reg                         ss_tlast;
    wire                        ss_tready;
    reg                         sm_tready;
    wire                        sm_tvalid;
    wire signed [(pDATA_WIDTH-1) : 0] sm_tdata;
    wire                        sm_tlast;
    reg                         axis_clk;
    reg                         axis_rst_n;

// ram for tap
    wire [3:0]               tap_WE;
    wire                     tap_EN;
    wire [(pDATA_WIDTH-1):0] tap_Di;
    wire [(pADDR_WIDTH-1):0] tap_A;
    wire [(pDATA_WIDTH-1):0] tap_Do;

// ram for data RAM
    wire [3:0]               data_WE;
    wire                     data_EN;
    wire [(pDATA_WIDTH-1):0] data_Di;
    wire [(pADDR_WIDTH-1):0] data_A;
    wire [(pDATA_WIDTH-1):0] data_Do;



    fir fir_DUT(
        .awready(awready),
        .wready(wready),
        .awvalid(awvalid),
        .awaddr(awaddr),
        .wvalid(wvalid),
        .wdata(wdata),
        .arready(arready),
        .rready(rready),
        .arvalid(arvalid),
        .araddr(araddr),
        .rvalid(rvalid),
        .rdata(rdata),
        .ss_tvalid(ss_tvalid),
        .ss_tdata(ss_tdata),
        .ss_tlast(ss_tlast),
        .ss_tready(ss_tready),
        .sm_tready(sm_tready),
        .sm_tvalid(sm_tvalid),
        .sm_tdata(sm_tdata),
        .sm_tlast(sm_tlast),

        // ram for tap
        .tap_WE(tap_WE),
        .tap_EN(tap_EN),
        .tap_Di(tap_Di),
        .tap_A(tap_A),
        .tap_Do(tap_Do),

        // ram for data
        .data_WE(data_WE),
        .data_EN(data_EN),
        .data_Di(data_Di),
        .data_A(data_A),
        .data_Do(data_Do),

        .axis_clk(axis_clk),
        .axis_rst_n(axis_rst_n)

        );
    
    // RAM for tap
    bram11 tap_RAM (
        .CLK(axis_clk),
        .WE(tap_WE),
        .EN(tap_EN),
        .Di(tap_Di),
        .A(tap_A),
        .Do(tap_Do)
    );

    // RAM for data: choose bram11 or bram12
    bram11 data_RAM(
        .CLK(axis_clk),
        .WE(data_WE),
        .EN(data_EN),
        .Di(data_Di),
        .A(data_A),
        .Do(data_Do)
    );

    reg signed [(pDATA_WIDTH-1):0] Din_list[0:(Data_Num-1)];
    reg signed [(pDATA_WIDTH-1):0] golden_list[0:(Data_Num-1)];
	
	real	CYCLE = `CYCLE_TIME;
	integer SEED = 123;
	integer i,j,k;
	integer PATNUM = 10;
	integer patcount;

	reg [31:0]  data_length;
    integer Din, golden, input_data, golden_data, m;
	reg signed [31:0] coef[0:10];
	
	reg error;
    reg status_error;
	reg error_coef;
	reg mask_done_idle;
	reg done;
	
    initial begin
        $dumpfile("fir.vcd");
        $dumpvars();
    end

	//Clock
	initial axis_clk = 1'b0;
	always #(CYCLE/2.0) axis_clk = (~axis_clk);

    initial begin
		for(patcount = 1;patcount <= PATNUM; patcount = patcount + 1)begin
			reset_task; // necessary for resetting every time 
            test_case_gen_task;
			//Check FIR is idle, if not, wait until FIR is idle
			check_idle_task;
			axi_in_task;
			fork
				transmit_Xn_task;
				receive_Yn_task;
				polling_ap_done_task;
			join
			@(posedge axis_clk);@(posedge axis_clk);
		end
		$finish;
	end

	task reset_task; begin
		axis_rst_n <= 0;
		done <= 0;
		arvalid <= 0;
		rready  <= 0;
		araddr  <= 12'h00;
		
		awvalid <= 0; 
		wvalid  <= 0;
		
		ss_tvalid <= 0;
        ss_tdata <= 0;
		ss_tlast <= 0;
		
		sm_tready <= 0;
		error_coef <= 0;
		
		@(posedge axis_clk); 
		@(posedge axis_clk);
		axis_rst_n <= 1;
	end endtask
	

	task test_case_gen_task; begin
        data_length = {$random}%500;
        for(i = 0;i<data_length;i=i+1)begin
            Din_list[i] = $random%1000;
        end

        for(i = 0;i<11;i=i+1)begin
            coef[i] = $random%1000;
        end

        for(i = 0;i<data_length;i=i+1)begin
            golden_list[i] = 0;
            for(j=0;j<11;j=j+1)begin
                if((i-j)>=0 && (i-j)<data_length) golden_list[i] = golden_list[i] + Din_list[i-j]*coef[j];
            end
        end
    end
    endtask
	// check idle = 0
	task check_idle_task; begin
		$display("-----------Start Check your ap_idle-----------");
		arvalid <= 1; 
		araddr <= 12'h00;
        rready <= 1;
		@(posedge axis_clk);
        while (!rvalid) @(posedge axis_clk);
        if(rdata[2] == 1'b1) begin
            $display("PASS : ap_idle = %d",rdata[2]);
        end 
		else begin
            $display("ERROR : ap_idle should be 1, your ap_idle = %d",rdata[2]);
        end
		arvalid <= 0;
		rready  <= 0;
		$display("---------End Check ap_idle task---------");
	end endtask

	task axi_in_task; begin
		$display("----Start the coefficient input(AXI-lite)----");
		axi_write(12'h10, data_length);
        $display("---data length %d----",data_length);
		for(k=0; k< Tape_Num; k=k+1) begin
            axi_write(12'h20+4*k, coef[k]);
        end
		
		// read-back and check
		$display(" Check Coefficient ...");
		for(k=0; k < Tape_Num; k=k+1) begin
            axi_read_check(12'h20+4*k, coef[k], 32'hffffffff);
        end
		arvalid <= 0;
		$display("----End the coefficient input and check(AXI-lite)----");

        $display("----Start FIR : programing ap_start = 1----");
        @(posedge axis_clk) axi_write(12'h00, 32'h0000_0001);    // ap_start = 1
        $display("----End the ap_start = 1 input(AXI-lite)----");
	end endtask

    task axi_write;
        input [11:0]    addr;
        input [31:0]    data;
        begin
            awvalid <= 0; 
			wvalid  <= 0;
            @(posedge axis_clk);
            awvalid <= 1; 
            wvalid  <= 1; 
			awaddr <= addr;
			wdata <= data;
            @(posedge axis_clk);
            while (!wready) @(posedge axis_clk);
			awvalid <= 0; 
			wvalid  <= 0;
        end
    endtask
	task axi_read_check;
        input [11:0]        addr;
        input signed [31:0] exp_data;
        input [31:0]        mask;
        begin
            arvalid <= 0;
            @(posedge axis_clk);
            arvalid <= 1; araddr <= addr;
            rready <= 1;
            @(posedge axis_clk);
            while (!rvalid) @(posedge axis_clk);
            if( (rdata & mask) != (exp_data & mask)) begin
                $display("ERROR: exp = %d, rdata = %d", exp_data, rdata);
                error_coef <= 1;
            end else begin
                $display("OK: exp = %d, rdata = %d", exp_data, rdata);
            end
			arvalid <= 0;
        end
    endtask
	
	task transmit_Xn_task; begin
		$display("------------Start simulation-----------");
        $display("----Start the data input(AXI-Stream)----");
        for(i=0;i<(data_length-1);i=i+1) begin
            ss_tlast <= 0; stream_in(Din_list[i]);
        end
        ss_tlast <= 1; 
		stream_in(Din_list[(data_length-1)]);
		ss_tlast <= 0;
		ss_tvalid <= 0;
		ss_tdata <= 0;
        $display("------End the data input(AXI-Stream)------");
	end endtask
	
	task stream_in;
        input  signed [31:0] in1;
        begin
            ss_tvalid <= 1;
            ss_tdata  <= in1;
            @(posedge axis_clk);
            while (!ss_tready) begin
                @(posedge axis_clk);
            end
        end
    endtask
	
	task receive_Yn_task; begin
        error <= 0; status_error <= 0;
        sm_tready <= 1;
        wait (sm_tvalid);
        for(k=0;k < data_length;k=k+1) begin
            stream_out(golden_list[k],k);
        end
    end endtask
	
	task stream_out;
        input  signed [31:0] in2; // golden data
        input         [31:0] pcnt; // pattern count
        begin
            sm_tready <= 1;
            @(posedge axis_clk) 
            wait(sm_tvalid);
            while(!sm_tvalid) @(posedge axis_clk);
            if (sm_tdata !== in2) begin
                $display("[ERROR] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
                error <= 1;
            end
            else begin
                $display("[PASS] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
            end
            @(posedge axis_clk);
        end
    endtask

	task polling_ap_done_task; begin
		while(done !== 1)begin
			arvalid <= 0;
            @(posedge axis_clk);
            arvalid <= 1; 
			araddr  <= 12'h0;
            rready  <= 1;
            @(posedge axis_clk);
            while (!rvalid) @(posedge axis_clk);
            if(rdata[1] == 1)begin
                $display("OK check ap_done = 1, FIR done");
            end
			done = rdata[1];
			arvalid <= 0;
		end
		if (error == 0 & error_coef == 0) begin
            $display("---------------------------------------------");
            $display("-----------Congratulations! Pass-------------");
        end
        else begin
            $display("--------Simulation Failed---------");
        end
	end endtask
	
	reg [31:0] data_ram_addr_00;
    reg [31:0] data_ram_addr_04;
    reg [31:0] data_ram_addr_08;
    reg [31:0] data_ram_addr_0C;
    reg [31:0] data_ram_addr_10;
    reg [31:0] data_ram_addr_14;
    reg [31:0] data_ram_addr_18;
    reg [31:0] data_ram_addr_1C;
    reg [31:0] data_ram_addr_20;
    reg [31:0] data_ram_addr_24;
    reg [31:0] data_ram_addr_28;

    reg [31:0] tap_ram_addr_00;
    reg [31:0] tap_ram_addr_04;
    reg [31:0] tap_ram_addr_08;
    reg [31:0] tap_ram_addr_0C;
    reg [31:0] tap_ram_addr_10;
    reg [31:0] tap_ram_addr_14;
    reg [31:0] tap_ram_addr_18;
    reg [31:0] tap_ram_addr_1C;
    reg [31:0] tap_ram_addr_20;
    reg [31:0] tap_ram_addr_24;
    reg [31:0] tap_ram_addr_28;
    
    always@(*) begin
        
        data_ram_addr_00 = data_RAM.RAM[0];
        data_ram_addr_04 = data_RAM.RAM[1];
        data_ram_addr_08 = data_RAM.RAM[2];
        data_ram_addr_0C = data_RAM.RAM[3];
        data_ram_addr_10 = data_RAM.RAM[4];
        data_ram_addr_14 = data_RAM.RAM[5];
        data_ram_addr_18 = data_RAM.RAM[6];
        data_ram_addr_1C = data_RAM.RAM[7];
        data_ram_addr_20 = data_RAM.RAM[8];
        data_ram_addr_24 = data_RAM.RAM[9];
        data_ram_addr_28 = data_RAM.RAM[10];

        tap_ram_addr_00  = tap_RAM.RAM[0];
        tap_ram_addr_04  = tap_RAM.RAM[1];
        tap_ram_addr_08  = tap_RAM.RAM[2];
        tap_ram_addr_0C  = tap_RAM.RAM[3];
        tap_ram_addr_10  = tap_RAM.RAM[4];
        tap_ram_addr_14  = tap_RAM.RAM[5];
        tap_ram_addr_18  = tap_RAM.RAM[6];
        tap_ram_addr_1C  = tap_RAM.RAM[7];
        tap_ram_addr_20  = tap_RAM.RAM[8];
        tap_ram_addr_24  = tap_RAM.RAM[9];
        tap_ram_addr_28  = tap_RAM.RAM[10];
    end

    // Prevent hang
    integer timeout = (1000000);
    initial begin
        while(timeout > 0) begin
            @(posedge axis_clk);
            timeout <= timeout - 1;
        end
        $display($time, "Simualtion Hang ....");
        $finish;
    end

	
endmodule
