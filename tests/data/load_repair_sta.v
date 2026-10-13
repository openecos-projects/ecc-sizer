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

  NAND2_X1 driver (.A1(in), .A2(enable), .ZN(high_fanout));
  NAND2_X4 load0 (.A1(high_fanout), .A2(high_fanout), .ZN(out0));
  NAND2_X4 load1 (.A1(high_fanout), .A2(high_fanout), .ZN(out1));
  NAND2_X4 load2 (.A1(high_fanout), .A2(high_fanout), .ZN(out2));
  NAND2_X4 load3 (.A1(high_fanout), .A2(high_fanout), .ZN(out3));
  NAND2_X4 load4 (.A1(high_fanout), .A2(high_fanout), .ZN(out4));
  NAND2_X4 load5 (.A1(high_fanout), .A2(high_fanout), .ZN(out5));
  NAND2_X4 load6 (.A1(high_fanout), .A2(high_fanout), .ZN(out6));
  NAND2_X4 load7 (.A1(high_fanout), .A2(high_fanout), .ZN(out7));
  NAND2_X4 load8 (.A1(high_fanout), .A2(high_fanout), .ZN(out8));
  NAND2_X4 load9 (.A1(high_fanout), .A2(high_fanout), .ZN(out9));
endmodule
