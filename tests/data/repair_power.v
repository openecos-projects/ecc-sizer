// Two power moves share one setup recovery at their common driver.
module top(input clk, input in, output out0, output out1, output out2);
VTBUF_H driver(.A(in), .Z(out0));
VTBUF_L side(.A(out0), .Z(out1));
VTBUF_L side2(.A(out0), .Z(out2));
endmodule
