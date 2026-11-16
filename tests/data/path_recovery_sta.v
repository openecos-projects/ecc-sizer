module top(input clk, input in, input enable,
    output out0, out1, out2, out3, out4, out5, out6, out7, out8, out9);
  wire reset_n;
  NAND2_X1 driver (.A1(in), .A2(enable), .ZN(reset_n));
  DFFR_X1 sink0 (.D(enable), .CK(clk), .RN(reset_n), .Q(out0));
  DFFR_X1 sink1 (.D(enable), .CK(clk), .RN(reset_n), .Q(out1));
  DFFR_X1 sink2 (.D(enable), .CK(clk), .RN(reset_n), .Q(out2));
  DFFR_X1 sink3 (.D(enable), .CK(clk), .RN(reset_n), .Q(out3));
  DFFR_X1 sink4 (.D(enable), .CK(clk), .RN(reset_n), .Q(out4));
  DFFR_X1 sink5 (.D(enable), .CK(clk), .RN(reset_n), .Q(out5));
  DFFR_X1 sink6 (.D(enable), .CK(clk), .RN(reset_n), .Q(out6));
  DFFR_X1 sink7 (.D(enable), .CK(clk), .RN(reset_n), .Q(out7));
  DFFR_X1 sink8 (.D(enable), .CK(clk), .RN(reset_n), .Q(out8));
  DFFR_X1 sink9 (.D(enable), .CK(clk), .RN(reset_n), .Q(out9));
endmodule
