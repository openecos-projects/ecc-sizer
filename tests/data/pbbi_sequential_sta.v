module top(
    input clk,
    input in,
    input enable,
    output out0,
    output out1,
    output out2,
    output out3,
    output out4,
    output out5,
    output out6,
    output out7,
    output out8,
    output out9
);
  wire high_fanout;
  wire state_q;
  DFF_X1 sink (.D(high_fanout), .CK(clk), .Q(state_q));

  NAND2_X1 driver (.A1(in), .A2(enable), .ZN(high_fanout));
  BUF_X1 load0 (.A(high_fanout), .Z(out0));
  BUF_X1 load1 (.A(high_fanout), .Z(out1));
  BUF_X1 load2 (.A(high_fanout), .Z(out2));
  BUF_X1 load3 (.A(high_fanout), .Z(out3));
  BUF_X1 load4 (.A(high_fanout), .Z(out4));
  BUF_X1 load5 (.A(high_fanout), .Z(out5));
  BUF_X1 load6 (.A(high_fanout), .Z(out6));
  BUF_X1 load7 (.A(high_fanout), .Z(out7));
  BUF_X1 load8 (.A(high_fanout), .Z(out8));
  BUF_X1 load9 (.A(state_q), .Z(out9));
endmodule
