module tb_mips;

	reg clk;
	reg reset;

	NITC_RISC_24 DUT(clk,reset);

	initial
		forever #4 clk=~clk;

	initial begin
	clk=0;
	reset=1;

	#10 reset=0;

	#100 $finish;
	end

endmodule


