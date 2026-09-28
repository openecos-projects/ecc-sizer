module top (
    input in,
    output out1,
    output out2
);
  wire n1;

  BUF_X1 driver (.A(in), .Z(n1));
  BUF_X2 load1 (.A(n1), .Z(out1));
  BUF_X4 load2 (.A(n1), .Z(out2));
endmodule
