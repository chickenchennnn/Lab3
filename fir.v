`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //axilite interface==============================
    //write(input)--
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    //read(output)---
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    //stream slave (input data)=========================
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    //stream master (output data)=======================
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

/*
Overall system design:
1. Only one multiplier and adder can be used, so if takes about 11~13(?) clk cycles to generated one output data,
2. Read and store all tap into bram. Once input data received, read all taps in the fir and store them in a tap_buffer 
   to prevent additional SRAM access.
3. After takeing about 11~13(?) clk cycles to generated one output data, axis_write outputs it. the calculation process don't need
   to wait for outputs process to finish.
*/




// =====FSM design=========== //
localparam STAT_IDLE = 3'd0;
localparam STAT_STORE_1_INPUT = 3'd1;
localparam STAT_CAL = 3'd2;
localparam STAT_FINISH = 3'd3;
reg [3-1:0]state, next_state;
// =====Axilite ctrl========= //

reg [3-1:0] cs_lite, ns_lite;
reg [4-1:0] cs_str, ns_str;
//axilite read module
reg arready_reg, rvalid_reg;
reg [(pADDR_WIDTH-1):0] araddr_buf;
//axilite write module
reg awready_reg,wready_reg;
reg [(pADDR_WIDTH-1):0] awaddr_buf;
reg [(pDATA_WIDTH-1):0] wdata_buf;
//axilite to config_ctrl module
reg [1:0] axilite_req; //(req)
reg [(pADDR_WIDTH-1):0] config_addr;
reg [(pDATA_WIDTH-1):0] w_data;
// =====Config ctrl=====//
//config_ctrl to axilite
wire [(pDATA_WIDTH-1):0] r_data;
//internal signal
reg desti; //destination 0:config_reg 1.tap_ram
reg desti_delay; //for r_data
reg W_EN; // indicate whether change the value of tap_ram or config reg
//confit ctrl to tap_ram_ctrl
wire[3:0]  config_tap_WE;
reg[(pDATA_WIDTH-1):0] config_tap_Di;
reg[(pADDR_WIDTH-1):0] config_tap_A;
reg[(pDATA_WIDTH-1):0] config_tap_Do;

//config ctrl to cfg_reg_ctrl
reg [8-1:0]config_ctrl_reg_out;
wire config_ctrl_reg_wen;
reg [8-1:0]config_ctrl_reg_in;
// =====taps_ram_ctrl======== //
reg [3:0]               tap_WE_reg;
reg [(pDATA_WIDTH-1):0] tap_Di_reg;
reg [(pADDR_WIDTH-1):0] tap_A_reg;

// data length 


reg [3:0] cnt;
reg [3:0] tap_cnt;
reg [(pADDR_WIDTH-1):0]curr_coef_addr;

wire signed[(pDATA_WIDTH-1):0] mult_coef;
wire signed[(pDATA_WIDTH-1):0] mult_din;

reg signed[(pDATA_WIDTH-1):0] fir_out;
// =====Axis-read ctrl======= //
// =====Axis-write ctrl====== //
// =====memory ctrl========== //
// =====calculation unit===== //

wire [2:0] ap_ctrl; // [0]ap_start , [1]ap_done, [2]ap_idle
reg ap_start;
reg ap_done;
reg ap_idle;
reg finish_flag;
assign ap_ctrl = {ap_idle,ap_done,ap_start};
//******************************//
// FSM                          //
//******************************//
/*
FSM design:
state
0.idle  : (1)wait for coefficients(wait for axilite arvalid), if arvalid=1 go to state 1
          (2)wait for ap_start. If ap_start is set, go to state 1.
1.store_1_input: wait for 1 data input from axis. If 1 input is received, go to state 2.
2.calculation: Do the fir calculation and output 1 data. if tlast==1 go to state 3. else go to state 1.
3.finish: send axilite signal (ap_done, ap_idle) to testbench.
*/



//******************************//
// AxiLite Controller           //
//******************************//
//=====lite fsm=====

localparam LITE_idle     = 3'd0;
localparam LITE_wfinish  = 3'd1;
localparam LITE_arready  = 3'd2;
localparam LITE_rreq     = 3'd3;
localparam LITE_read     = 3'd4;
localparam LITE_done     = 3'd5;




//stream fsm

localparam STR_IDLE = 1;
localparam STR_FIRST = 9;
localparam STR_RESET_DATARAM = 2;
localparam STR_WAIT_LITE = 3;
localparam STR_CAL = 4;
localparam STR_OUT = 5;
localparam STR_SHIFT_READ = 6;
localparam STR_SHIFT_WRITE = 7;
localparam STR_NEWIN = 8;
localparam STR_STORE = 10;

always@(posedge axis_clk) begin
    if(!axis_rst_n) ap_start <= 1'b0;
    else if(cs_lite == LITE_done && wvalid)
        ap_start <= wdata[0];
    else ap_start <= 0;
    
end

always@(posedge axis_clk) begin
    if(!axis_rst_n) ap_done <= 1'b0;
    else if(ns_str == STR_OUT && finish_flag) ap_done <= 1'b1;
    else if(cs_str == STR_FIRST && ss_tvalid) ap_done <= 1'b0;
    
end

always@(posedge axis_clk) begin
    if(!axis_rst_n) ap_idle <= 1'b1;
    else if(ns_str == STR_OUT && finish_flag) ap_idle <= 1'b1;
    else if(cs_str == STR_FIRST && ss_tvalid) ap_idle <= 1'b0;
end

always @(posedge axis_clk) begin
    if(!axis_rst_n) tap_cnt <= 0;
    else if(rvalid && araddr_buf >= 12'h020) tap_cnt <= tap_cnt+1;
    else if(cs_lite == LITE_done) tap_cnt <= 0;
end
always@(posedge axis_clk) begin
    if(!axis_rst_n)     cs_lite <= LITE_idle;
    else                cs_lite <= ns_lite;
end


always@(*) begin
    case(cs_lite)
        LITE_idle: begin
            if(tap_cnt >= Tape_Num)         ns_lite = LITE_done;
            else if(arvalid)                    ns_lite = LITE_arready;
            else if(wready_reg && awready_reg)  ns_lite = LITE_wfinish;
            else                                ns_lite = cs_lite;
        end
        LITE_wfinish:// by the time, axilite has already received awaddr and wdata 
            ns_lite = LITE_idle;
        LITE_arready: begin
            if(arready && arvalid)  ns_lite = LITE_rreq;
            else                    ns_lite = cs_lite;
        end
        LITE_rreq: begin
            if(rready)  ns_lite = LITE_read;
            else        ns_lite = cs_lite;
        end
        LITE_read:      ns_lite = LITE_idle;
        LITE_done:  if(cs_str == STR_OUT && finish_flag) ns_lite = LITE_idle;      
                    else ns_lite = LITE_done;
        default:        ns_lite = LITE_idle;
    endcase
end


//===axilite_to_config===
always@(*) begin
    case(cs_lite)
        LITE_wfinish:   config_addr = awaddr_buf; // for write
        LITE_rreq:      config_addr = araddr_buf;
        default:        config_addr = 12'd0; // addr not being used
    endcase
end
always@(*)  begin
    case(cs_lite)
        LITE_wfinish:   axilite_req = 2'd1; //write
        LITE_rreq:      axilite_req = 2'd2;
        default:        axilite_req = 2'd0; // no operation
    endcase
end

always@(*)  begin
    case(cs_lite)
        LITE_wfinish:   w_data = wdata_buf;
        default:        w_data = {pDATA_WIDTH{1'b0}};
    endcase
end



//===lite_write=====

assign awready = (cs_lite == LITE_idle || cs_lite == LITE_done)? awready_reg : 1'b0;
assign wready =  (cs_lite == LITE_idle || cs_lite == LITE_done)? wready_reg : 1'b0;

//hand shake block 

always@(posedge axis_clk) begin
    if(!axis_rst_n) awready_reg <= 1'b0;
    else begin
        case(cs_lite)
            LITE_idle: begin
                if(awvalid) awready_reg <= 1'b1;
                else        awready_reg <= awready_reg;
            end
            LITE_done: begin
                if(awvalid) awready_reg <= 1'b1;
                else        awready_reg <= 0;
            end
            default:        awready_reg <= 1'b0;
        endcase
    end
end
always@(posedge axis_clk) begin
    if(!axis_rst_n) wready_reg <=1'b0;
    else begin
        case(cs_lite)
            LITE_idle: begin
                if(wvalid)  wready_reg <= 1'b1;
                else        wready_reg <= wready_reg;
            end
            LITE_done: begin
                if(awvalid) wready_reg <= 1'b1;
                else        wready_reg <= 0;
            end
            default:        wready_reg <= 1'b0;
        endcase
    end
end


//data_addr block
always@(posedge axis_clk or negedge axis_rst_n)
    if(!axis_rst_n)
        awaddr_buf <= {pADDR_WIDTH{1'b0}};
    else
        case(cs_lite)
            LITE_idle:
                if(awready) awaddr_buf <= awaddr;
                else        awaddr_buf <= awaddr_buf;
            LITE_wfinish:
                // clean the buffer
                awaddr_buf <= {pADDR_WIDTH{1'b0}};
            default:
                awaddr_buf <= {pADDR_WIDTH{1'b0}};
        endcase
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n)
        wdata_buf <= {pDATA_WIDTH{1'b0}};
    else
        case(cs_lite)
            LITE_idle:
                if(wready)
                    wdata_buf <= wdata;
                else
                    wdata_buf <= wdata_buf;
            LITE_wfinish:
                // clean the buffer
                wdata_buf <= {pDATA_WIDTH{1'b0}};
            default:
                wdata_buf <= {pDATA_WIDTH{1'b0}};
        endcase
end


//===lite_read======
//wiring

assign arready = arready_reg;
assign rvalid = rvalid_reg;

// handshake block
always@(posedge axis_clk or negedge axis_rst_n)
    if(~axis_rst_n)
        arready_reg <= 1'b0;
    else
        case(cs_lite)
            LITE_idle:
                if(arvalid)
                    arready_reg <= 1'b1;
                else
                    arready_reg <= 1'b0;
            LITE_done:
                if(arvalid)
                    arready_reg <= 1'b1;
                else 
                    arready_reg <=1'b0;
            default:
                arready_reg <= 1'b0;
        endcase
always@(*)
    if(~axis_rst_n)
        rvalid_reg = 1'b0;
    else begin
        case(cs_lite)
            LITE_read:
                rvalid_reg=1'b1;
            LITE_done: 
                if(rready) rvalid_reg = 1'b1; 
                else rvalid_reg = 1'b0;
            default:
                rvalid_reg=1'b0;
        endcase
    end
// data_addr block
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n)
        araddr_buf <= {pADDR_WIDTH{1'b0}};
    else begin
        case(cs_lite)
            LITE_arready: begin
                if(arready)     araddr_buf <= araddr;
                else            araddr_buf <= araddr_buf;
            end
            LITE_rreq:          araddr_buf <= araddr_buf;
            LITE_read:          araddr_buf <= {pADDR_WIDTH{1'b0}};
            default:            araddr_buf <= {pADDR_WIDTH{1'b0}};
        endcase
    end
end

assign rdata = (cs_lite == LITE_read)?  r_data: (finish_flag? {ap_ctrl} : {pDATA_WIDTH{1'b0}}); // sent finish config
//******************************//
//Config ctrl                   //
//******************************//
//===internal signal=======
//desti
always@(*) begin
    if(config_addr >= 12'h20)// taps
        desti = 1;
    else if(config_addr == 12'h10)// d_length(taps)
        desti = 1;
    else// config reg
        desti = 0;
end
always@(posedge axis_clk) begin
    if(!axis_rst_n) desti_delay <= 1'b0;
    else            desti_delay <= desti;
end
//W_EN (write or not)
always@(*) begin
    case(axilite_req)
        2'b01:      W_EN = 1'b1;//write
        default:    W_EN = 1'b0;
    endcase
end
//===destination: tap_ram====
//config_tap_A
always@(*) begin
    if(cs_str == STR_CAL) begin
        config_tap_A = curr_coef_addr;
    end
    else if(config_addr >= 12'h20)
        config_tap_A = config_addr - 12'h20; //transform to tap_addr
    else if(config_addr == 12'h10)// d_length
        config_tap_A = 12'h10<<2;
    else
        config_tap_A = 12'h10<<2;
end
//config_tap_WE
assign config_tap_WE = (desti)? {4{W_EN}} : 4'd0;
//config_tap_Di
always@(*)  config_tap_Di = w_data;
always@(*)  config_tap_Do = tap_Do;
  
//===destination: config_reg====
//config_ctrl_reg_in
always@(*) config_ctrl_reg_in = w_data;
//config_ctrl_reg_wen
assign config_ctrl_reg_wen = (desti == 0)? W_EN:1'b0;
//config_ctrl_reg_out
    //寫在別的地方(cfg_reg_ctrl)7

//===config_ctrl to axilite
//rdata
assign r_data = (desti_delay)?config_tap_Do:ap_ctrl; //這邊要寫config_ctrl_reg_out 但先這樣寫


//******************************//
// taps_ram_ctrl                //
//******************************//
assign tap_WE = tap_WE_reg;
assign tap_Di = tap_Di_reg;
assign tap_A = tap_A_reg;
assign tap_EN = 1'b1;
always @(*) begin
    tap_WE_reg = config_tap_WE;
    tap_A_reg = config_tap_A;
    tap_Di_reg = config_tap_Di;
end
//test

// data (stream in, stream out, store in shift data ram )

always@(posedge axis_clk) begin
    if(!axis_rst_n)     cs_str <= STR_IDLE;
    else                cs_str <= ns_str;
end
always@(*) begin
    case(cs_str)
        STR_IDLE: ns_str = STR_FIRST;
        STR_FIRST: 
            if(ss_tvalid) ns_str = STR_RESET_DATARAM;
            else ns_str = cs_str;
        STR_RESET_DATARAM: begin
            if(cnt >= 10) ns_str = STR_WAIT_LITE;
            else ns_str = cs_str;
        end
        STR_WAIT_LITE: begin 
            if(cs_lite == LITE_done) ns_str = STR_CAL;
            else ns_str = cs_str;
        end
        STR_CAL: 
            if(cnt > 10) ns_str = STR_OUT; 
            else ns_str = cs_str;
        STR_OUT:    if(finish_flag) ns_str = STR_IDLE;
                    else ns_str = STR_SHIFT_READ;
        STR_SHIFT_READ: ns_str = STR_SHIFT_WRITE;
        STR_SHIFT_WRITE: 
            if(cnt == 10) ns_str = STR_NEWIN;
            else ns_str = STR_SHIFT_READ;
        STR_NEWIN: 
            if(ss_tvalid) ns_str = STR_STORE;
            else ns_str = cs_str;
        STR_STORE: ns_str = STR_CAL;
        default : ns_str = STR_IDLE;
    endcase
end



reg [3:0]               data_wen;
wire                     data_enable;
reg [(pDATA_WIDTH-1):0] data_in_reg;
wire [(pADDR_WIDTH-1):0] data_addr;
reg [(pADDR_WIDTH-1):0] data_addr_reg;


assign data_enable = 1;
assign data_addr = data_addr_reg;


assign data_WE = data_wen;
assign data_EN = data_enable;
assign data_Di = ((cs_str == STR_SHIFT_WRITE)? data_Do : data_in_reg);
assign data_A = data_addr;

always@(posedge axis_clk) begin
    if(!axis_rst_n) cnt <= 0;
    else begin
        case(cs_str)
            STR_RESET_DATARAM: cnt <= cnt + 1;
            STR_CAL: cnt <= cnt + 1;
            STR_OUT: cnt <= 0;
            STR_SHIFT_READ: cnt <= cnt + 1;
            STR_SHIFT_WRITE: cnt <= cnt ;
            default: cnt <= 0;
        endcase
    end
end
reg ss_tready_reg;
always @(posedge axis_clk) begin
    if(!axis_rst_n) ss_tready_reg <= 0;
    else if(cs_str == STR_FIRST && ns_str == STR_RESET_DATARAM &&  ss_tvalid == 1) ss_tready_reg <= 1;
    else if(cs_str == STR_SHIFT_WRITE && ns_str == STR_NEWIN) ss_tready_reg <= 1;
    else ss_tready_reg <= 0;
end

assign ss_tready = ss_tready_reg;
always @(*) begin
    case(cs_str)
        STR_RESET_DATARAM: data_wen = 4'b1111;
        STR_SHIFT_WRITE: data_wen =4'b1111;
        STR_STORE: data_wen = 4'b1111;
        default: data_wen = 4'b0000;
    endcase
end
always @(posedge axis_clk) begin
    if(!axis_rst_n) data_addr_reg <= 0;
    else begin  
        case(cs_str)
            STR_RESET_DATARAM: data_addr_reg <= data_addr_reg + 4;
            STR_WAIT_LITE: data_addr_reg <= 12'h000;
            STR_CAL: data_addr_reg <= data_addr_reg + 4;
            STR_OUT: data_addr_reg <= 12'h024;
            STR_SHIFT_READ: data_addr_reg <= data_addr_reg + 4; 
            STR_SHIFT_WRITE: data_addr_reg <= data_addr_reg - 8; 
            STR_NEWIN: data_addr_reg <= 0;
            STR_STORE: data_addr_reg <= 12'h000;
            default: data_addr_reg <= 0;
        endcase
    end

end


always @(posedge axis_clk) begin
    if(!axis_rst_n) data_in_reg <= 0;
    else if(ns_str == STR_RESET_DATARAM) begin
        if(cs_str == STR_FIRST) data_in_reg <= ss_tdata;
        else data_in_reg <= 0;
        // else data_in_reg <= cnt; // only for test
    end
    else if(cs_str == STR_NEWIN)
        data_in_reg <= ss_tdata;
    else 
        data_in_reg <= 0;
    
end


always @(posedge axis_clk) begin
    if(!axis_rst_n) curr_coef_addr <= 0;
    else begin
        case(cs_str)
            STR_IDLE: curr_coef_addr <= 0;
            STR_CAL:  curr_coef_addr <= curr_coef_addr + 4;
            STR_STORE: curr_coef_addr <= 0;
        endcase
    end
end



assign mult_coef = tap_Do;
assign mult_din = data_Do;

always @(posedge axis_clk) begin
    if(!axis_rst_n) fir_out <= 0;
    else begin
        case(cs_str)
            STR_CAL: 
                if(cnt > 0) fir_out <= fir_out + mult_coef * mult_din;
                else fir_out <= 0;
            STR_OUT: fir_out <= 0;
            STR_SHIFT_READ: fir_out <= 0;
            default: fir_out <= 0;
        endcase
    end
end

always @(posedge axis_clk) begin
    if(!axis_rst_n) finish_flag <= 0;
    else if(ss_tlast && ss_tready) finish_flag <= 1;
    else if(cs_str == STR_IDLE) finish_flag <= 0;
end


assign sm_tvalid = (cs_str == STR_OUT)? 1: 0;
assign sm_tdata = (cs_str == STR_OUT)? fir_out:0;
assign sm_tlast = finish_flag;
endmodule