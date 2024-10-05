module RegisterFile (
    input wire clk,
    input wire reset,
    output reg [15:0] registers [7:0] 
);

    initial begin
        registers[0] = 16'h0000;  // Initialize R0
        registers[1] = 16'h0001;  // Initialize R1
        registers[2] = 16'h0002;  // Initialize R2
        registers[3] = 16'h0003;  // Initialize R3
        registers[4] = 16'h0004;  // Initialize R4
        registers[5] = 16'h0005;  // Initialize R5
        registers[6] = 16'h0006;  // Initialize R6
        registers[7] = 16'h0007;  // Initialize R7
    end
    always @(posedge clk) begin
       $display("At time %0t, registers[3] = %h", $time, registers[3]);
    end

endmodule
