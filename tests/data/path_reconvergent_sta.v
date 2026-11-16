module top(input clk, input a, input b, input enable, output out);
  wire n0, n1, merged;
  NAND2_X1 left_driver (.A1(a), .A2(enable), .ZN(n0));
  NAND2_X1 right_driver (.A1(b), .A2(enable), .ZN(n1));
  NAND2_X1 merge (.A1(n0), .A2(n1), .ZN(merged));
  BUF_X1 sink (.A(merged), .Z(out));
endmodule
