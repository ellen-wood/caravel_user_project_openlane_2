`default_nettype none
`timescale 1ns/1ns
//Give a one-cycle pulse at the detected value
module pwm_oneshot_detector #(
    parameter WIDTH = 8
    ) (
    output wire out,
    input wire [WIDTH-1:0] count,
    input wire [WIDTH-1:0] pos   //- where +Ve edge come
    );

    //reg [WIDTH-1:0] count;
    assign out = ( count == pos );

endmodule
