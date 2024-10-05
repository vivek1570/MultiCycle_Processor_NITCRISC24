module datapath(clk, reset, IorD, MemRead, MemWrite, MemtoReg, IRWrite,
PCSource, ALUSrcB, ALUSrcA, RegWrite, RegDst, PCSel, ALUCtrl, Op, Zero, cz);

	parameter PCSTART = 0; //starting address of instruction memory
	input clk;
	input reset;
	input IorD;
	input MemWrite,MemRead,MemtoReg;
	input IRWrite;
	input PCSource;
	input RegDst,RegWrite;
	input ALUSrcA;
	input [1:0] ALUSrcB;
	input PCSel;
	input [3:0] ALUCtrl;

	output [3:0] Op;
	output Zero;
	output [1:0] cz;

	reg [15:0]PC;

	reg [15:0] ALUOut;

	reg [15:0] ALUResult;
	wire [15:0] OpA;
	reg [15:0] OpB;

	reg [15:0]A;
	reg [15:0]B;

	wire [15:0] address;

  wire [15:0] MemData;

	reg[15:0]mem[255:0];

	reg [15:0]Instruction;

	reg [15:0]mdr;

	wire [15:0] da;//read data 1
	wire [15:0] db;//read data 2

	reg[15:0]registers[7:0];
  initial begin
        registers[0] = 16'h0000;  // Initialize R0
        registers[1] = 16'h0001;  // Initialize R1
        registers[2] = 16'h0007;  // Initialize R2
        registers[3] = 16'h0003;  // Initialize R3
        registers[4] = 16'h0004;  // Initialize R4
        registers[5] = 16'h0005;  // Initialize R5
        registers[6] = 16'h0006;  // Initialize R6
        registers[7] = 16'h0007;  // Initialize R7
    end

	assign cz=Instruction[1:0];
	assign Op=Instruction[15:12];

	//data and instruction memory
	assign address=(IorD)?ALUOut:PC;

	initial
		$readmemh("/home/kpvivek/verilog/mc_nitc/mem.dat", mem);

	always @(posedge clk) begin
		if(MemWrite)
			mem[address]<=B;
	end

	assign
		MemData =(MemRead)? mem[address]:16'bx;

	//PC logic

	always@ (posedge clk)begin
		if(reset)
			PC<=PCSTART;
		else
		if(PCSel)begin
			case (PCSource)
				1'b0: PC<=ALUResult;
				1'b1: PC<=ALUOut;
			endcase
		end
	end

	//instruction register

	always @(posedge clk) begin
		if (IRWrite)
			Instruction <= MemData;
	end

	//memory data register
	always @(posedge clk) begin
		mdr <= MemData;
	end

	//register file
	//$r0 is always 0
	assign da = (Instruction[11:9]!=0) ? registers[Instruction[11:9]] : 0;
	assign db = (Instruction[8:6]!=0) ? registers[Instruction[8:6]] : 0;


	always @(posedge clk) begin
		if (RegWrite)begin
			if (RegDst)
				registers[Instruction[5:3]]<=(MemtoReg)?mdr:ALUOut;
			else
				registers[Instruction[8:6]]<=(MemtoReg)?mdr:ALUOut;
		end
	end

	//A and B registers

	always @(posedge clk) begin
		A<=da;
	end

	always@(posedge clk) begin
		B<=db;
	end


	//ALU

	assign OpA=(ALUSrcA)?A:PC;

	always@(ALUSrcB or B or Instruction[15:0])begin
		casex(ALUSrcB)
		2'b00:OpB=B;
		2'b01:OpB=1;
		2'b1x:OpB={{(10){Instruction[5]}},Instruction[5:0]};
		endcase
	end

	assign Zero = (ALUResult==0);//Zero == 1 when ALUResult is 0 (for branch)

  //ALU logic is mainly working in here
  /*

  */

	always @(ALUCtrl or OpA or OpB) begin
		case(ALUCtrl)
		4'b0000:ALUResult = OpA & OpB;
		4'b0001:ALUResult = OpA | OpB;
		4'b0010:ALUResult = OpA + OpB;
		4'b0110:ALUResult = OpA - OpB;
		4'b0111:ALUResult = OpA < OpB?1:0;
		4'b1100:ALUResult = ~(OpA | OpB);
		endcase
	end

	//ALUOut register

	always@(posedge clk) begin
		ALUOut<=ALUResult;
	end

endmodule