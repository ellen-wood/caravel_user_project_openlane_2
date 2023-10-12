`default_nettype none
`timescale 1ns/1ns

//See also  >xcircuit  /root/isolator_transformer/driver_ic_v3_skywater_0u13_RISC-V/ps/ramp_and_samp_adc.ps &


`include "/root/eda/caravel_user_project_openlane_2/verilog/rtl/defines.v"


module TOP_digital #(
    parameter WIDTH = 8,  //PWM width
    parameter WIDTH_RAMP_AND_SAMP = 8,  //Of ADC counts
    parameter   [31:0]  BASE_ADDRESS    = 32'h3000_0000,        // base address
    parameter   [31:0]  PWM0_rise_addr     = BASE_ADDRESS,
    parameter   [31:0]  PWM0_fall_addr     = BASE_ADDRESS + 4,
    parameter   [31:0]  PWM1_rise_addr     = BASE_ADDRESS + 8,
    parameter   [31:0]  PWM1_fall_addr     = BASE_ADDRESS + 12,
    parameter   [31:0]  PWM2_rise_addr     = BASE_ADDRESS + 16,
    parameter   [31:0]  PWM2_fall_addr     = BASE_ADDRESS + 20,
    parameter   [31:0]  PWM3_rise_addr     = BASE_ADDRESS + 24,
    parameter   [31:0]  PWM3_fall_addr     = BASE_ADDRESS + 28,


    parameter   [31:0]  ramp_samp_count0_addr      = BASE_ADDRESS + 32,
                    //- captured value from ramp adc 0   -READ-
                    //   bits 31:24 = oxAA    for DEBUG
    parameter   [31:0]  ramp_samp_count1_addr      = BASE_ADDRESS + 36,
                    //- captured value from ramp adc 1  -READ-
                    //   bits 31:24 = 0x55   for DEBUG
    parameter   [31:0]  STATUS_reg_addr     = BASE_ADDRESS + 40,
                    //READ
                    //bit 0 = isolated_RX_data_bit_0   --- if it detects a signal it latches high. will be cleared when  tx  bit goes low
                    //bit 1 = isolated_RX_data_bit_1
                    //   bits 31:24 = 0x33  for DEBUG

    parameter   [31:0]  CTRL_reg_addr     = BASE_ADDRESS + 60
                    //WRITE
                    //bit0 = isolated_TX_data_bit_0   --  when goes to 1, it sends the bit.  when goes to 0 it resets the detector 
                    //bit1 = isolated_TX_data_bit_1 
                    //
                    //bit16 = PWM_with_trim_nPWM_normal_ch0  ---  mux the PWM output between Trimmed when high, or  Normal when low
                    //bit17 = PWM_with_trim_nPWM_normal_ch1
                    //bit18 = PWM_with_trim_nPWM_normal_ch2
                    //bit19 = PWM_with_trim_nPWM_normal_ch3
                    //
                    //bit31 = run_ramp_Nreset_counters   (when it goes 1 it lets counter run.  when 0 it reset the counter.
                    //                                        - have to set it  1, then wait a while until know the result will be in
                    //                                            then get result, then set this  0  to reset the count ready for next run.
)
(
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	    // User area 2 digital ground
`endif
  



    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,

    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output reg wbs_ack_o,
    output reg [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,
    


    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq,

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // USER I/O i.e. NON  user-project-wrapper pins

    inout analog_trigger_out,
    inout analog_trigger_in,

    output wire pwm_rise_out,
    output wire pwm_fall_out

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

);
    //reg wbs_ack_o;
    //reg [31:0] wbs_dat_o;
    wire reset;
    wire clk;
    wire sync;

    //registers to store 
    // Note, PWM?_????  registers  have lower 24 bits being normal PWM.  Upper 8 bits are the trim values
    //WRITE
    reg [31:0] pwm0_rise_pos;
    reg [31:0] pwm0_fall_pos;
    reg [31:0] pwm1_rise_pos;
    reg [31:0] pwm1_fall_pos;
    reg [31:0] pwm2_rise_pos;
    reg [31:0] pwm2_fall_pos;
    reg [31:0] pwm3_rise_pos;
    reg [31:0] pwm3_fall_pos;
  
    //READ
    reg [31:0] ramp_samp_count0;                    //- captured value from ramp adc 0   -READ-
    reg [31:0] ramp_samp_count1;                    //- captured value from ramp adc 1  -READ-
    reg [31:0]  STATUS_reg;                     //READ
   
    //Ramp and samp control tr
    reg [31:0] CTRL_reg;

    ///assign reset = wb_rst_i;   //Use wishbone reset line as a reset [see note in user_project_wrapper]
    assign reset = la_data_in[0];  // JW,  Matt uses logic analyser bit 0 to do a reset --- i connect it up at the lower level, not this level
 
 assign user_irq = 0;
 assign io_out = 0;
 assign io_oeb = 0;
 assign la_data_out = 0;


    //Make some assignments for the ramp_and_samp  unit


    assign clk = wb_clk_i;

    //wishbone interface and registers
    wire o_wb_stall;
    assign o_wb_stall = 0;  //-instant acknowledge

    // writes
    always @(posedge clk) begin
        if(reset)
            begin
                pwm0_rise_pos <= 0;
                pwm0_fall_pos <= 0;
            end
        else if(wbs_stb_i && wbs_cyc_i && wbs_we_i && !o_wb_stall)
            case(wbs_adr_i)
                PWM0_rise_addr:
                    pwm0_rise_pos <= wbs_dat_i[31:0];
                PWM0_fall_addr:
                    pwm0_fall_pos <= wbs_dat_i[31:0];
                PWM1_rise_addr:
                    pwm1_rise_pos <= wbs_dat_i[31:0];
                PWM1_fall_addr:
                    pwm1_fall_pos <= wbs_dat_i[31:0];
                PWM2_rise_addr:
                    pwm2_rise_pos <= wbs_dat_i[31:0];
                PWM2_fall_addr:
                    pwm2_fall_pos <= wbs_dat_i[31:0];
                PWM3_rise_addr:
                    pwm3_rise_pos <= wbs_dat_i[31:0];
                PWM3_fall_addr:
                    pwm3_fall_pos <= wbs_dat_i[31:0];
                CTRL_reg_addr:
                    CTRL_reg <= wbs_dat_i[31:0];

                    
            endcase
    end

    // reads
    always @(posedge clk) begin
        if(wbs_stb_i && wbs_cyc_i && !wbs_we_i && !o_wb_stall)
            case(wbs_adr_i)
                /*

                ramp_samp_count0_addr: 
                    wbs_dat_o <= {8'h0AA, 16'b0, ramp_samp_count0};  //something that can check for   
                    //- captured value from ramp adc 0   -READ-
                    //   bits 31:24 = oxAA    for DEBUG
                ramp_samp_count1_addr:
                    wbs_dat_o <= {8'h055, 16'b0, ramp_samp_count1};  //something that can check for   
                    //- captured value from ramp adc 1  -READ-
                    //   bits 31:24 = 0x55   for DEBUG
                STATUS_reg_addr:
                    wbs_dat_o <= {8'h033, 16'b0, STATUS_reg};  //something that can check for   
                    //READ
                    //bit 0 = isolated_RX_data_bit_0   --- if it detects a signal it latches high. will be cleared when  tx  bit goes low
                    //bit 1 = isolated_RX_data_bit_1
                    //   bits 31:24 = 0x33  for DEBUG


                */
                    //wbs_dat_o <= {24'b0, 8'b01010101};  //something that can check for   0x55 
                default:
                    wbs_dat_o <= 32'hAA553311;
            endcase
    end

    // acks
    always @(posedge clk) begin
        if(reset)
            wbs_ack_o <= 0;
        else
            // return ack immediately
            wbs_ack_o <= (wbs_stb_i && !o_wb_stall && (wbs_adr_i == PWM0_rise_addr || wbs_adr_i == PWM0_fall_addr) ); 
    end


//            o_wb_ack <= (i_wb_stb && !o_wb_stall && (i_wb_addr == LED_ADDRESS || i_wb_addr == BUTTON_ADDRESS));

    //JW: allowd to do synthesis here
    //FINDME assign io_oeb = 4'b0;  //-pokes '0's up to the higher level
    //assign sync = reset; //pinning back up for cocotb to spot when its time to apply stimulus (connected here to reset)
    //wire enc0_a_db, enc0_b_db;
    //wire enc1_a_db, enc1_b_db;
    //wire enc2_a_db, enc2_b_db;
    //wire [7:0] enc0, enc1, enc2;
    
    

    wire [WIDTH-1:0] count;

    //wire pwm_rise_out, pwm_fall_out;

    //assign outputs up to pins
    //FINDME assign io_out[`MPRJ_IO_PADS-1] = pwm_rise_out;
    //FINDME assign io_out[`MPRJ_IO_PADS-2] = pwm_fall_out;

    //Instantiate pwm modules
    pwm_counter #(.WIDTH(WIDTH)) pwm_counter0(.clk(clk), .reset(reset), .count(count));
    pwm_oneshot_detector #(.WIDTH(WIDTH)) pwm_rise_detector(.out(pwm_rise_out), .count(count), .pos(pwm0_rise_pos));
    pwm_oneshot_detector #(.WIDTH(WIDTH)) pwm_fall_detector(.out(pwm_fall_out), .count(count), .pos(pwm0_fall_pos));


   
    // pwm modules
    //pwm #(.WIDTH(8)) pwm0(.clk(clk), .reset(reset), .out(pwm0_out), .level(enc0));
   
endmodule
