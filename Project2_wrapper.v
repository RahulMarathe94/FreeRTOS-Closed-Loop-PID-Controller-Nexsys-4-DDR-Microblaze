//Copyright 1986-2017 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2017.4 (win64) Build 2086221 Fri Dec 15 20:55:39 MST 2017
//Date        : Fri May  4 17:18:44 2018
//Host        : Rahul Marathe,Kiyasul Arif
//Project:    : Closed Loop Implementation using PID
//Date        : 5th May,2018
//Target:     : Nexys4 DDR 
//Command     : generate_target Project2_wrapper.bd
//Design      : Project2_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module Project2_wrapper                                // HDL Wrapper Generated for acting as the top level module
   (DDR2_SDRAM_addr,                                   ////////////////////////////////////////////////////////////////////////
    DDR2_SDRAM_ba,									   //
    DDR2_SDRAM_cas_n,								   //
    DDR2_SDRAM_ck_n,								   //
    DDR2_SDRAM_ck_p,								   //
    DDR2_SDRAM_cke,									   //
    DDR2_SDRAM_cs_n,								   //
    DDR2_SDRAM_dm,									   // DDR 2 SDRAM MEMORY PORTS
    DDR2_SDRAM_dq,									   //
    DDR2_SDRAM_dqs_n,								   //
    DDR2_SDRAM_dqs_p,								   //
    DDR2_SDRAM_odt,									   //
    DDR2_SDRAM_ras_n,								   //
    DDR2_SDRAM_we_n,								   ////////////////////////////////////////////////////////////////////////////
    RGB1_Blue_0,									   // RGB LED 1 - BLUE
    RGB1_Green_0,									   // RGB LED 1 - GREEN
    RGB1_Red_0,										   // RGB LED 1 - RED
    RGB2_Blue_0,                                       // RGB LED 2 - BLUE
    RGB2_Green_0,                                      // RGB LED 2 - GREEN
    RGB2_Red_0,                                        // RGB LED 2 - RED
    UART_0_rxd,                                        // UARTLITE RX
    UART_0_txd,                                        // UARTLITE TX
    an_0,											   // SEVEN SEGMENT ENABLE
    btnC_0,                                            // PUSHBUTTON C
    btnD_0,											   // PUSHBUTTON D
    btnL_0,											   // PUSHBUTTON L
    btnR_0,											   // PUSHBUTTON R	
    btnU_0,											   // PUSHBUTTON U				
    dp_0,											   // Decimal Point Enable			
    led_0,                                             // [15:0] LEDS
    seg_0,                                             // Seven Segment Data Pins
    sw_0,                                              // [15:0] Slide Switches
    sys_clock,                                         // System Clock
    btnCpuReset,                                       // Active Low Button Reset
    JA,                                                // JA Connector- PMODOLED
    JB,                                                // JB Connector- Unused
    JC,                                                // JC Connector- HBridge
    JD);                                               // JD Connector- Rotary Encoder

    
  output [12:0]DDR2_SDRAM_addr;
  output [2:0]DDR2_SDRAM_ba;
  output DDR2_SDRAM_cas_n;
  output [0:0]DDR2_SDRAM_ck_n;
  output [0:0]DDR2_SDRAM_ck_p;
  output [0:0]DDR2_SDRAM_cke;
  output [0:0]DDR2_SDRAM_cs_n;
  output [1:0]DDR2_SDRAM_dm;
  inout [15:0]DDR2_SDRAM_dq;
  inout [1:0]DDR2_SDRAM_dqs_n;
  inout [1:0]DDR2_SDRAM_dqs_p;
  output [0:0]DDR2_SDRAM_odt;
  output DDR2_SDRAM_ras_n;
  output DDR2_SDRAM_we_n;
  output RGB1_Blue_0;
  output RGB1_Green_0;
  output RGB1_Red_0;
  output RGB2_Blue_0;
  output RGB2_Green_0;
  output RGB2_Red_0;
  input UART_0_rxd;
  output UART_0_txd;
  output [7:0]an_0;
  input btnC_0;
  input btnD_0;
  input btnL_0;
  input btnR_0;
  input btnU_0;
  output dp_0;
  output [15:0]led_0;
  output [6:0]seg_0;
  input [15:0]sw_0;
  input sys_clock;
  input btnCpuReset;
                                  
  inout	    [7:0] 		JA;	            // For connecting the PMODOLED
  output	[7:0] 		JB;				// JB Pmod connector Unused. Can be used for debugging purposes           
  inout 	[7:0] 		JC;	            // For connecting the Pmod HBridge
  inout  	[7:0] 		JD;	            // FOr connecting the Rotary Encoder
  wire [12:0]DDR2_SDRAM_addr;
  wire [2:0]DDR2_SDRAM_ba;
  wire DDR2_SDRAM_cas_n;
  wire [0:0]DDR2_SDRAM_ck_n;
  wire [0:0]DDR2_SDRAM_ck_p;
  wire [0:0]DDR2_SDRAM_cke;
  wire [0:0]DDR2_SDRAM_cs_n;
  wire [1:0]DDR2_SDRAM_dm;
  wire [15:0]DDR2_SDRAM_dq;
  wire [1:0]DDR2_SDRAM_dqs_n;
  wire [1:0]DDR2_SDRAM_dqs_p;
  wire [0:0]DDR2_SDRAM_odt;
  wire DDR2_SDRAM_ras_n;
  wire DDR2_SDRAM_we_n;
  ///////////////////////////////////////////

  
  /////////OLED PINS/////////////////////////
  wire PmodOLEDrgb_out_0_pin10_i;
  wire PmodOLEDrgb_out_0_pin10_io;
  wire PmodOLEDrgb_out_0_pin10_o;
  wire PmodOLEDrgb_out_0_pin10_t;
  wire PmodOLEDrgb_out_0_pin1_i;
  wire PmodOLEDrgb_out_0_pin1_io;
  wire PmodOLEDrgb_out_0_pin1_o;
  wire PmodOLEDrgb_out_0_pin1_t;
  wire PmodOLEDrgb_out_0_pin2_i;
  wire PmodOLEDrgb_out_0_pin2_io;
  wire PmodOLEDrgb_out_0_pin2_o;
  wire PmodOLEDrgb_out_0_pin2_t;
  wire PmodOLEDrgb_out_0_pin3_i;
  wire PmodOLEDrgb_out_0_pin3_io;
  wire PmodOLEDrgb_out_0_pin3_o;
  wire PmodOLEDrgb_out_0_pin3_t;
  wire PmodOLEDrgb_out_0_pin4_i;
  wire PmodOLEDrgb_out_0_pin4_io;
  wire PmodOLEDrgb_out_0_pin4_o;
  wire PmodOLEDrgb_out_0_pin4_t;
  wire PmodOLEDrgb_out_0_pin7_i;
  wire PmodOLEDrgb_out_0_pin7_io;
  wire PmodOLEDrgb_out_0_pin7_o;
  wire PmodOLEDrgb_out_0_pin7_t;
  wire PmodOLEDrgb_out_0_pin8_i;
  wire PmodOLEDrgb_out_0_pin8_io;
  wire PmodOLEDrgb_out_0_pin8_o;
  wire PmodOLEDrgb_out_0_pin8_t;
  wire PmodOLEDrgb_out_0_pin9_i;
  wire PmodOLEDrgb_out_0_pin9_io;
  wire PmodOLEDrgb_out_0_pin9_o;
  wire PmodOLEDrgb_out_0_pin9_t;
  
  /////////////////////ROTARY ENCODER PINS////////////////////
  wire Pmod_out_0_pin10_i;
  wire Pmod_out_0_pin10_io;
  wire Pmod_out_0_pin10_o;
  wire Pmod_out_0_pin10_t;
  wire Pmod_out_0_pin1_i;
  wire Pmod_out_0_pin1_io;
  wire Pmod_out_0_pin1_o;
  wire Pmod_out_0_pin1_t;
  wire Pmod_out_0_pin2_i;
  wire Pmod_out_0_pin2_io;
  wire Pmod_out_0_pin2_o;
  wire Pmod_out_0_pin2_t;
  wire Pmod_out_0_pin3_i;
  wire Pmod_out_0_pin3_io;
  wire Pmod_out_0_pin3_o;
  wire Pmod_out_0_pin3_t;
  wire Pmod_out_0_pin4_i;
  wire Pmod_out_0_pin4_io;
  wire Pmod_out_0_pin4_o;
  wire Pmod_out_0_pin4_t;
  wire Pmod_out_0_pin7_i;
  wire Pmod_out_0_pin7_io;
  wire Pmod_out_0_pin7_o;
  wire Pmod_out_0_pin7_t;
  wire Pmod_out_0_pin8_i;
  wire Pmod_out_0_pin8_io;
  wire Pmod_out_0_pin8_o;
  wire Pmod_out_0_pin8_t;
  wire Pmod_out_0_pin9_i;
  wire Pmod_out_0_pin9_io;
  wire Pmod_out_0_pin9_o;
  wire Pmod_out_0_pin9_t;
  ////////////////////////// RGB LED PINS /////////////////////////////
  wire RGB1_Blue_0;
  wire RGB1_Green_0;
  wire RGB1_Red_0;
  wire RGB2_Blue_0;
  wire RGB2_Green_0;
  wire RGB2_Red_0;
  /////////////////////////////////// UART TX/RX///////////////////////////
  wire UART_0_rxd;
  wire UART_0_txd;
  ///////////////////////////////////////////////////////////////////////
  wire [7:0]an_0;
  wire btnC_0;
  wire btnD_0;
  wire btnL_0;
  wire btnR_0;
  wire btnU_0;
  wire dp_0;
  wire gpio_0_DIR_tri_i;
  wire [15:0]led_0;
  wire pwm0_EN;
  wire [6:0]seg_0;
  wire [15:0]sw_0;
  wire sys_clock;
  wire sys_rst;
  wire [31:0] GPIO_RPMINPUT_tri_i1;
  wire sa_input;
  
	    //////////////////////////////Assign Ports JA[7:0] For the PMODOLED RGB/////////////////
   assign JA[0]=PmodOLEDrgb_out_0_pin1_io;
   assign JA[1]=PmodOLEDrgb_out_0_pin2_io;
   assign JA[2]=PmodOLEDrgb_out_0_pin3_io;
   assign JA[3]=PmodOLEDrgb_out_0_pin4_io;
   assign JA[4]=PmodOLEDrgb_out_0_pin7_io;
   assign JA[5]=PmodOLEDrgb_out_0_pin8_io;
   assign JA[6]=PmodOLEDrgb_out_0_pin9_io;
   assign JA[7]=PmodOLEDrgb_out_0_pin10_io;
   
   
  ///////////////////////////// Assign Ports Jc[2:0] /////////////////////////////////// 
    assign JC[0]=gpio_0_DIR_tri_i;                        // Direction 0/1 for the Drive Motor to spin Clockwise or AntiClockwise
    assign JC[1]=pwm0_EN;                                 // Enable is the Port at which the Generated PWM Signal is given to
    assign sa_input=JC[2];                                // SA_Input is th epin at which the Data from the Hall Sensor is read from.
   

   
    ////////////////////////// Assign Ports JD[3:0] to the Rotary ENcoder///////////////////
    assign Pmod_out_0_pin1_io=JD[0];                      // SA
    assign Pmod_out_0_pin2_io=JD[1];                      // SB
    assign Pmod_out_0_pin3_io=JD[2];                      // BUTTTON
    assign Pmod_out_0_pin4_io=JD[3];                      // SWITCH
    

   
   assign sys_rst=btnCpuReset;                             //Assign the on board button reset
 
  IOBUF PmodOLEDrgb_out_0_pin10_iobuf
       (.I(PmodOLEDrgb_out_0_pin10_o),
        .IO(PmodOLEDrgb_out_0_pin10_io),
        .O(PmodOLEDrgb_out_0_pin10_i),
        .T(PmodOLEDrgb_out_0_pin10_t));
  IOBUF PmodOLEDrgb_out_0_pin1_iobuf
       (.I(PmodOLEDrgb_out_0_pin1_o),
        .IO(PmodOLEDrgb_out_0_pin1_io),
        .O(PmodOLEDrgb_out_0_pin1_i),
        .T(PmodOLEDrgb_out_0_pin1_t));
  IOBUF PmodOLEDrgb_out_0_pin2_iobuf
       (.I(PmodOLEDrgb_out_0_pin2_o),
        .IO(PmodOLEDrgb_out_0_pin2_io),
        .O(PmodOLEDrgb_out_0_pin2_i),
        .T(PmodOLEDrgb_out_0_pin2_t));
  IOBUF PmodOLEDrgb_out_0_pin3_iobuf
       (.I(PmodOLEDrgb_out_0_pin3_o),
        .IO(PmodOLEDrgb_out_0_pin3_io),
        .O(PmodOLEDrgb_out_0_pin3_i),
        .T(PmodOLEDrgb_out_0_pin3_t));
  IOBUF PmodOLEDrgb_out_0_pin4_iobuf
       (.I(PmodOLEDrgb_out_0_pin4_o),
        .IO(PmodOLEDrgb_out_0_pin4_io),
        .O(PmodOLEDrgb_out_0_pin4_i),
        .T(PmodOLEDrgb_out_0_pin4_t));
  IOBUF PmodOLEDrgb_out_0_pin7_iobuf
       (.I(PmodOLEDrgb_out_0_pin7_o),
        .IO(PmodOLEDrgb_out_0_pin7_io),
        .O(PmodOLEDrgb_out_0_pin7_i),
        .T(PmodOLEDrgb_out_0_pin7_t));
  IOBUF PmodOLEDrgb_out_0_pin8_iobuf
       (.I(PmodOLEDrgb_out_0_pin8_o),
        .IO(PmodOLEDrgb_out_0_pin8_io),
        .O(PmodOLEDrgb_out_0_pin8_i),
        .T(PmodOLEDrgb_out_0_pin8_t));
  IOBUF PmodOLEDrgb_out_0_pin9_iobuf
       (.I(PmodOLEDrgb_out_0_pin9_o),
        .IO(PmodOLEDrgb_out_0_pin9_io),
        .O(PmodOLEDrgb_out_0_pin9_i),
        .T(PmodOLEDrgb_out_0_pin9_t));
  IOBUF Pmod_out_0_pin10_iobuf
       (.I(Pmod_out_0_pin10_o),
        .IO(Pmod_out_0_pin10_io),
        .O(Pmod_out_0_pin10_i),
        .T(Pmod_out_0_pin10_t));
  IOBUF Pmod_out_0_pin1_iobuf
       (.I(Pmod_out_0_pin1_o),
        .IO(Pmod_out_0_pin1_io),
        .O(Pmod_out_0_pin1_i),
        .T(Pmod_out_0_pin1_t));
  IOBUF Pmod_out_0_pin2_iobuf
       (.I(Pmod_out_0_pin2_o),
        .IO(Pmod_out_0_pin2_io),
        .O(Pmod_out_0_pin2_i),
        .T(Pmod_out_0_pin2_t));
  IOBUF Pmod_out_0_pin3_iobuf
       (.I(Pmod_out_0_pin3_o),
        .IO(Pmod_out_0_pin3_io),
        .O(Pmod_out_0_pin3_i),
        .T(Pmod_out_0_pin3_t));
  IOBUF Pmod_out_0_pin4_iobuf
       (.I(Pmod_out_0_pin4_o),
        .IO(Pmod_out_0_pin4_io),
        .O(Pmod_out_0_pin4_i),
        .T(Pmod_out_0_pin4_t));
  IOBUF Pmod_out_0_pin7_iobuf
       (.I(Pmod_out_0_pin7_o),
        .IO(Pmod_out_0_pin7_io),
        .O(Pmod_out_0_pin7_i),
        .T(Pmod_out_0_pin7_t));
  IOBUF Pmod_out_0_pin8_iobuf
       (.I(Pmod_out_0_pin8_o),
        .IO(Pmod_out_0_pin8_io),
        .O(Pmod_out_0_pin8_i),
        .T(Pmod_out_0_pin8_t));
  IOBUF Pmod_out_0_pin9_iobuf
       (.I(Pmod_out_0_pin9_o),
        .IO(Pmod_out_0_pin9_io),
        .O(Pmod_out_0_pin9_i),
        .T(Pmod_out_0_pin9_t));
        
    ///////////////////////////////////// Instantiation of the Hardware Detecttion Module /////////////////////////////////////////////////    
   rpmdetect rpmdetect(.clock(sys_clock), .reset(sys_rst), .sa_input(sa_input), .sb_input(), .rpm_output(GPIO_RPMINPUT_tri_i1));     
        
   ///////////////////////////////////// Instantiation of the EMBSYS Module /////////////////////////////////////////////////     
  Project2 Project2_i
       (.DDR2_SDRAM_addr(DDR2_SDRAM_addr),
        .DDR2_SDRAM_ba(DDR2_SDRAM_ba),
        .DDR2_SDRAM_cas_n(DDR2_SDRAM_cas_n),
        .DDR2_SDRAM_ck_n(DDR2_SDRAM_ck_n),
        .DDR2_SDRAM_ck_p(DDR2_SDRAM_ck_p),
        .DDR2_SDRAM_cke(DDR2_SDRAM_cke),
        .DDR2_SDRAM_cs_n(DDR2_SDRAM_cs_n),
        .DDR2_SDRAM_dm(DDR2_SDRAM_dm),
        .DDR2_SDRAM_dq(DDR2_SDRAM_dq),
        .DDR2_SDRAM_dqs_n(DDR2_SDRAM_dqs_n),
        .DDR2_SDRAM_dqs_p(DDR2_SDRAM_dqs_p),
        .DDR2_SDRAM_odt(DDR2_SDRAM_odt),
        .DDR2_SDRAM_ras_n(DDR2_SDRAM_ras_n),
        .DDR2_SDRAM_we_n(DDR2_SDRAM_we_n),
        .GPIO_RPMINPUT_tri_i(GPIO_RPMINPUT_tri_i1),
        .PmodOLEDrgb_out_0_pin10_i(PmodOLEDrgb_out_0_pin10_i),
        .PmodOLEDrgb_out_0_pin10_o(PmodOLEDrgb_out_0_pin10_o),
        .PmodOLEDrgb_out_0_pin10_t(PmodOLEDrgb_out_0_pin10_t),
        .PmodOLEDrgb_out_0_pin1_i(PmodOLEDrgb_out_0_pin1_i),
        .PmodOLEDrgb_out_0_pin1_o(PmodOLEDrgb_out_0_pin1_o),
        .PmodOLEDrgb_out_0_pin1_t(PmodOLEDrgb_out_0_pin1_t),
        .PmodOLEDrgb_out_0_pin2_i(PmodOLEDrgb_out_0_pin2_i),
        .PmodOLEDrgb_out_0_pin2_o(PmodOLEDrgb_out_0_pin2_o),
        .PmodOLEDrgb_out_0_pin2_t(PmodOLEDrgb_out_0_pin2_t),
        .PmodOLEDrgb_out_0_pin3_i(PmodOLEDrgb_out_0_pin3_i),
        .PmodOLEDrgb_out_0_pin3_o(PmodOLEDrgb_out_0_pin3_o),
        .PmodOLEDrgb_out_0_pin3_t(PmodOLEDrgb_out_0_pin3_t),
        .PmodOLEDrgb_out_0_pin4_i(PmodOLEDrgb_out_0_pin4_i),
        .PmodOLEDrgb_out_0_pin4_o(PmodOLEDrgb_out_0_pin4_o),
        .PmodOLEDrgb_out_0_pin4_t(PmodOLEDrgb_out_0_pin4_t),
        .PmodOLEDrgb_out_0_pin7_i(PmodOLEDrgb_out_0_pin7_i),
        .PmodOLEDrgb_out_0_pin7_o(PmodOLEDrgb_out_0_pin7_o),
        .PmodOLEDrgb_out_0_pin7_t(PmodOLEDrgb_out_0_pin7_t),
        .PmodOLEDrgb_out_0_pin8_i(PmodOLEDrgb_out_0_pin8_i),
        .PmodOLEDrgb_out_0_pin8_o(PmodOLEDrgb_out_0_pin8_o),
        .PmodOLEDrgb_out_0_pin8_t(PmodOLEDrgb_out_0_pin8_t),
        .PmodOLEDrgb_out_0_pin9_i(PmodOLEDrgb_out_0_pin9_i),
        .PmodOLEDrgb_out_0_pin9_o(PmodOLEDrgb_out_0_pin9_o),
        .PmodOLEDrgb_out_0_pin9_t(PmodOLEDrgb_out_0_pin9_t),
        .Pmod_out_0_pin10_i(Pmod_out_0_pin10_i),
        .Pmod_out_0_pin10_o(Pmod_out_0_pin10_o),
        .Pmod_out_0_pin10_t(Pmod_out_0_pin10_t),
        .Pmod_out_0_pin1_i(Pmod_out_0_pin1_i),
        .Pmod_out_0_pin1_o(Pmod_out_0_pin1_o),
        .Pmod_out_0_pin1_t(Pmod_out_0_pin1_t),
        .Pmod_out_0_pin2_i(Pmod_out_0_pin2_i),
        .Pmod_out_0_pin2_o(Pmod_out_0_pin2_o),
        .Pmod_out_0_pin2_t(Pmod_out_0_pin2_t),
        .Pmod_out_0_pin3_i(Pmod_out_0_pin3_i),
        .Pmod_out_0_pin3_o(Pmod_out_0_pin3_o),
        .Pmod_out_0_pin3_t(Pmod_out_0_pin3_t),
        .Pmod_out_0_pin4_i(Pmod_out_0_pin4_i),
        .Pmod_out_0_pin4_o(Pmod_out_0_pin4_o),
        .Pmod_out_0_pin4_t(Pmod_out_0_pin4_t),
        .Pmod_out_0_pin7_i(Pmod_out_0_pin7_i),
        .Pmod_out_0_pin7_o(Pmod_out_0_pin7_o),
        .Pmod_out_0_pin7_t(Pmod_out_0_pin7_t),
        .Pmod_out_0_pin8_i(Pmod_out_0_pin8_i),
        .Pmod_out_0_pin8_o(Pmod_out_0_pin8_o),
        .Pmod_out_0_pin8_t(Pmod_out_0_pin8_t),
        .Pmod_out_0_pin9_i(Pmod_out_0_pin9_i),
        .Pmod_out_0_pin9_o(Pmod_out_0_pin9_o),
        .Pmod_out_0_pin9_t(Pmod_out_0_pin9_t),
        .RGB1_Blue_0(RGB1_Blue_0),
        .RGB1_Green_0(RGB1_Green_0),
        .RGB1_Red_0(RGB1_Red_0),
        .RGB2_Blue_0(RGB2_Blue_0),
        .RGB2_Green_0(RGB2_Green_0),
        .RGB2_Red_0(RGB2_Red_0),
        .UART_0_rxd(UART_0_rxd),
        .UART_0_txd(UART_0_txd),
        .an_0(an_0),
        .btnC_0(btnC_0),
        .btnD_0(btnD_0),
        .btnL_0(btnL_0),
        .btnR_0(btnR_0),
        .btnU_0(btnU_0),
        .dp_0(dp_0),
        .gpio_0_DIR_tri_o(gpio_0_DIR_tri_i),
        .led_0(led_0),
        .pwm0_EN(pwm0_EN),
        .seg_0(seg_0),
        .sw_0(sw_0),
        .sys_clock(sys_clock),
        .sys_rst(sys_rst));
endmodule
