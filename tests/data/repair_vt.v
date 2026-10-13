module top(input clk, input in, output out0, output out1);
VTBUF_H driver(.A(in), .Z(out0));
VTBUF_L side(.A(in), .Z(out1));
endmodule
