//////////////////////////////////////////////////////////////////////////////////
// Company: IISc
// Engineer: Devanjan Maiti (devanjan@dese.iisc.ernet.in)
//           Mudasir Mir    (mudasir.mir7@gmail.com)

// Create Date: 17.05.2017 16:41:56
// Design Name: 
// Module Name: calibration_unit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: This module generates switching signals for the calibration unit
//              in the temperature sensor.
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module calibration_unit(
input  wire [11:0] count_i,
output reg  [3:0] switch_o
    );

// LOCAL PARAMETERS -------------------------------------------------------------- 
parameter VAL1 = 12'd62;    // comparison value 1
parameter VAL2 = 12'd130;   // comparison value 2
parameter VAL3 = 12'd190;   // comparison value 3
parameter VAL4 = 12'd260;   // comparison value 4
  
// LOCAL REG/WIRE DECLARATIONS ---------------------------------------------------  
reg  [12:0] val1;
reg  [12:0] val2;
reg  [12:0] val3;
reg  [12:0] val4;
wire [12:0] val1_pos;
wire [12:0] val2_pos;
wire [12:0] val3_pos;
wire [12:0] val4_pos;
wire [12:0] bmuxout;
wire [12:0] cmuxout;
wire [12:0] dmuxout;

// GLUE LOGIC --------------------------------------------------------------------
always@ (*) begin
  val1 = count_i - VAL1;
  val2 = count_i - VAL2;
  val3 = count_i - VAL3;
  val4 = count_i - VAL4;
end 
 
assign val1_pos = (val1[12] == 1) ? (~val1 + 1'b1) : val1;
assign val2_pos = (val2[12] == 1) ? (~val2 + 1'b1) : val2;
assign val3_pos = (val3[12] == 1) ? (~val3 + 1'b1) : val3;
assign val4_pos = (val4[12] == 1) ? (~val4 + 1'b1) : val4;

assign bmuxout = (val1_pos >= val2_pos) ? val2_pos : val1_pos;
assign cmuxout = (bmuxout  >= val3_pos) ? val3_pos : bmuxout;
assign dmuxout = (cmuxout  >= val4_pos) ? val4_pos : cmuxout;

// SWITCH OUTPUT ASSIGNMENT ------------------------------------------------------ 
always@ (*) begin
  case (dmuxout) 
    val1_pos: switch_o = 4'b0001;
    val2_pos: switch_o = 4'b0010;
    val3_pos: switch_o = 4'b0100;
    default : switch_o = 4'b1000;
  endcase
end

endmodule
