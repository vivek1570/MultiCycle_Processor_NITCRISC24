module alucontrol(AluOp,FnField,AluCtrl);

input [1:0] AluOp;
input [1:0] FnField; //for R-type instruction

output reg [3:0] AluCtrl;


always@(AluOp or FnField)begin
    
	casex({AluOp,FnField})
		4'b00_xx:AluCtrl=4'b0010; //lw / sw
		4'b01_xx:AluCtrl=4'b0110; //beq
		4'b1x_xx:AluCtrl=4'b0010; //add
		4'b1x_xx:AluCtrl=4'b0110; //sub
		4'b1x_xx:AluCtrl=4'b0000; //and
		4'b1x_xx:AluCtrl=4'b0001; //or
		4'b1x_xx:AluCtrl=4'b0111; //slt
	endcase
end

endmodule