`default_nettype none
`timescale 1ns/1ns
module pwm_counter #(
    parameter WIDTH = 8
    ) (
    input wire clk,
    input wire reset,
    output reg [WIDTH-1:0] count
    );

    //wire clk;
    //wire reset;
    //reg [WIDTH-1:0] count;

    always @(posedge clk) begin
        if(reset)
            count <= 1'b0;
        else
            count <= count + 1'b1;
    end


endmodule

