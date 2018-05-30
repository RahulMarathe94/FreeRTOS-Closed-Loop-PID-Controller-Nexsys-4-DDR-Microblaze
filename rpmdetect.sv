`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Portland State University
// Engineer: Kiyasul Arif & Rahul Marathe
// 
// Create Date: 04/16/2018 08:49:59 AM
// Design Name: HW RPM DETECTION
// Module Name: rpmdetect
// Project Name: Project Two
// Target Devices: NEXYS4DDR
// Tool Versions: VIVADO 2017.4.1
// Description: This module detects the number of revolutions per second of the DC motor
// 
// Dependencies: None
// 
// Revision: V1
// Revision 0.01 - File Created
// Additional Comments:  The Module detects the SA_input from the HBridge which origiantes from the Output of Hall Sensor. 
// The Hall Sensro Input is sampled for 1 Million times, as the Sampling Clock is 100 Mhz. 
// 100M per 100 Mhz Clock will give us 1 Sample/Second. 
//////////////////////////////////////////////////////////////////////////////////
 
module rpmdetect(clock, reset, sa_input , sb_input, rpm_output);
    // Inputs to the module
    input logic clock;
    input logic reset;
    input logic sa_input;
    input logic sb_input;
	// Ouputs from this module
    output logic [15:0] rpm_output;
    parameter millionSamples = 100_000_000;                 // The Clock Frequeny is 100 Mhz. Keeping a Sample COunt of 100M will give us 1 sample/count 
														    // Some counters and parameters for the logic
    logic     [31:0] SAMPLING_FREQUENCY = millionSamples;	// Total Count at which the Pulses from SA are detected for 1 second 
    logic     [31:0] SAMPLING_COUNT     = 32'd0; 			// Sampling counter
	logic 	  [31:0] HIGH_COUNT         = 32'd0;			// To store the SA_Count
	logic            R_FLAG;								// A Reset Flag to reset the 2nd always block.
	
always_ff@(posedge clock)
begin
  if(~reset|| (SAMPLING_COUNT==SAMPLING_FREQUENCY))			// Check if neg_reset if hit or Sampled Count is Equal to 100 M Samples
	   begin
	   SAMPLING_COUNT<=0;									// If yes, clear the counters
	   R_FLAG<=1;											// Set reset flag to 0
	   end
  else														// Else,
	   begin
	   SAMPLING_COUNT<=SAMPLING_COUNT+1;					// Reset the Flag 
	   R_FLAG<=0;											// Increment the total count by 1
	   end
end 

always_ff@(posedge sa_input or posedge R_FLAG)				// Always @ SA_input pulses as we have to calculate the Rotations per second
begin
	if(R_FLAG)												// If the reset flag is high
	   begin
	   HIGH_COUNT<= 0;										// Clear the counters
	   end
	else													// Else
	   begin
	   HIGH_COUNT<=HIGH_COUNT+1;							// Increment Count by 1
	   end
    end	
always_comb
begin
    if(SAMPLING_COUNT==SAMPLING_FREQUENCY-1)				// if total count is 100 million equal to 1 second in 100Mhz clk
        begin												// Assign output * 60 to get rotations per minute
        rpm_output<=HIGH_COUNT*60;							// RPM_Output is given to the Mocirblaze using a GPIO perihperhal
    end
end
	endmodule

	