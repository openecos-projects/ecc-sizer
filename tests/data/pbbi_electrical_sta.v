module top(input clk, input a, input b, output out0);
  wire pa, pb, n;
  BUF_X1 path_driver(.A(a), .Z(pa));
  BUF_X1 side_driver(.A(b), .Z(pb));
  NAND2_X1 resized(.A1(pa), .A2(pb), .ZN(n));
  BUF_X1 downstream(.A(n), .Z(out0));
endmodule
