module top(input clk, input in, output out0);
  wire n1, n2, n3;
  BUF_X1 driver (.A(in), .Z(n1));
  BUF_X1 load0 (.A(n1), .Z(n2));
  BUF_X1 tail (.A(n2), .Z(n3));
  BUF_X1 sink (.A(n3), .Z(out0));
endmodule
