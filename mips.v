module mips(clk,reset);

	input clk,reset;

	wire [3:0] Op;
	wire Zero;
	wire IorD;
	wire MemRead;
	wire MemWrite;
	wire MemToReg;
	wire IRWrite;
	wire PCSource;
	wire [1:0] ALUSrcB;
	wire ALUSrcA;
	wire RegWrite;
	wire RegDst;
	wire PCSel;
	wire [1:0] ALUOp;
	wire [3:0] ALUCtrl;
	wire [1:0] cz;


	control control_D(clk, reset,Op, Zero, IorD, MemRead, MemWrite, MemtoReg,
	IRWrite, PCSource, ALUSrcB, ALUSrcA, RegWrite, RegDst, PCSel, ALUOp);

	alucontrol alucontrol_D(ALUOp, cz,Op, ALUCtrl);

	datapath  datapath_D(clk, reset,IorD, MemRead, MemWrite, MemtoReg, IRWrite, PCSource,
	ALUSrcB, ALUSrcA, RegWrite, RegDst, PCSel, ALUCtrl, Op, Zero, cz);

endmodule


module alucontrol(AluOp,cz,Op,AluCtrl);

input [1:0] AluOp;
input [1:0] cz; //for R-type instruction
input [3:0] Op;

output reg [3:0] AluCtrl;


always@(AluOp or cz or Op)begin
    
	casex({AluOp,cz})
		4'b00_xx:AluCtrl=4'b0010; //lw / sw
		4'b01_xx:AluCtrl=4'b0110; //beq
		4'b1x_xx:AluCtrl=4'b0010; //add
		4'b1x_xx:AluCtrl=4'b0110; //sub
		4'b1x_xx:AluCtrl=4'b0000; //and
		4'b1x_xx:AluCtrl=4'b0001; //or
		4'b1x_xx:AluCtrl=4'b0111; //slt
		// $display("hello from Aluop cz");
	endcase
	case(Op)
	4'b0010:begin
		AluCtrl=4'b1000; //for nand
	end
	endcase

end

endmodule

module control (clk, reset,Op, Zero, IorD, MemRead, MemWrite, MemtoReg, IRWrite,
PCSource, ALUSrcB, ALUSrcA, RegWrite, RegDst, PCSel, ALUOp);

	input clk;
	input reset;
	input [3:0] Op;
	input Zero;

	output reg IorD;
	output reg MemWrite;
	output reg MemRead;
	output reg MemtoReg;
	output reg IRWrite;
	output reg PCSource;
	output reg RegDst;
	output reg RegWrite;
	output reg ALUSrcA;
	output reg [1:0] ALUSrcB;
	output PCSel;
	output reg [1:0] ALUOp;

	reg PCWrite;
	reg PCWriteCond;

	assign
		PCSel = (PCWrite | (PCWriteCond & Zero));

	//states
	parameter FETCH = 4'b0000;
	parameter DECODE = 4'b0001;
	parameter MEMADRCOMP = 4'b0010;
	parameter MEMACCESSL = 4'b0011;//L1
	parameter MEMREADEND = 4'b0100;//L2
	parameter MEMACCESSS = 4'b0101;//S
	parameter EXECUTION = 4'b0110;
	parameter RTYPEEND = 4'b0111;
	parameter BEQ = 4'b1000;

	reg [3:0] state;
	reg [3:0] nextstate;

  always@(posedge clk)
    if (reset)
		state <= FETCH;
    else
		state <= nextstate;


	always@(state or Op) begin
      	case (state)
        FETCH:  begin
						nextstate = DECODE;
						case(Op)
						4'b1010:$display("Instr=> LW");
						4'b1001:$display("Instr=> SW");
						4'b0000:$display("ADD or ADC");
						4'b0010:$display("NDu or NDZ");
						4'b1011:$display("beq");
						endcase
				end
				
        DECODE:  case(Op)
					//OpCode
                   4'b1010:	begin
										nextstate = MEMADRCOMP;//LW
									 end
                   4'b1001:	begin
											nextstate = MEMADRCOMP;//SW
									 end
                   4'b0000: begin
											nextstate = EXECUTION; // ADD or ADC
									 end
									 4'b0010: begin
											 nextstate = EXECUTION;
									 end
                   4'b1011:	begin
											nextstate = BEQ;//BEQ
									 end
                   default: nextstate = FETCH;
                 endcase
        MEMADRCOMP:  case(Op)
                   4'b1010:      nextstate = MEMACCESSL;//lw
                   4'b1001:      nextstate = MEMACCESSS;//sw
                   default: nextstate = FETCH;
                 endcase
        MEMACCESSL:    nextstate = MEMREADEND;
        MEMREADEND:    nextstate = FETCH;
        MEMACCESSS:    nextstate = FETCH;
        EXECUTION: nextstate = RTYPEEND;
        RTYPEEND: nextstate = FETCH;
        BEQ:   nextstate = FETCH;
        default: nextstate = FETCH;
      endcase
    end


	always@(state) begin

	IorD=1'b0; MemRead=1'b0; MemWrite=1'b0; MemtoReg=1'b0; IRWrite=1'b0; PCSource=1'b0;
	ALUSrcB=2'b00; ALUSrcA=1'b0; RegWrite=1'b0; RegDst=1'b0; PCWrite=1'b0; PCWriteCond=1'b0; ALUOp=2'b00;
    	case (state)
        FETCH:
          begin
						$display("current state at %0t =FETCH\n",$time);
            MemRead = 1'b1;
            IRWrite = 1'b1;
            ALUSrcB = 2'b01;
            PCWrite = 1'b1;
          end
        DECODE:
				begin
					$display("current state at %0t =DECODE\n",$time);
	    ALUSrcB = 2'b11;
				end
        MEMADRCOMP:
          begin
						$display("current state at %0t =MEMADRCOMP\n",$time);
            ALUSrcA = 1'b1;
            ALUSrcB = 2'b10;
          end
        MEMACCESSL:
          begin
						$display("current state at %0t =MEMACCESSL\n",$time);
            MemRead = 1'b1;
            IorD    = 1'b1;
          end
        MEMREADEND:
          begin
						$display("current state at %0t =MEMREADEND\n",$time);
            RegWrite = 1'b1;
	    			MemtoReg = 1'b1;
            RegDst = 1'b0;
          end
        MEMACCESSS:
          begin
						$display("current state at %0t =MEMACCESSS\n",$time);
            MemWrite = 1'b1;
            IorD     = 1'b1;
          end
        EXECUTION:
          begin
						$display("current state at %0t =EXECUTION\n",$time);
            ALUSrcA = 1'b1;
            ALUOp   = 2'b10;
          end
        RTYPEEND:
          begin
						$display("current state at %0t =RTYEND\n",$time);
            RegDst   = 1'b1;
            RegWrite = 1'b1;
          end
        BEQ:
          begin
						$display("current state at %0t =BEQ\n",$time);
            ALUSrcA = 1'b1;
            ALUOp   = 2'b01;
            PCWriteCond = 1'b1;
	    			PCSource = 2'b01;
          end
      endcase
    end
endmodule

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

	reg[15:0]mem[0:19];

	reg [15:0]Instruction;

	reg [15:0]mdr;

	wire [15:0] da;//read data 1
	wire [15:0] db;//read data 2

	//carry flag and zero_flag
	reg Carry, ZeroFlag; 

	reg[15:0]registers[7:0];
  initial begin
        registers[0] = PCSTART;  // Initialize R0
        registers[1] = 16'h0003;  // Initialize R1
        registers[2] = 16'h0007;  // Initialize R2
        registers[3] = 16'h0003;  // Initialize R3
        registers[4] = 16'h0006;  // Initialize R4
        registers[5] = 16'h0006;  // Initialize R5
        registers[6] = 16'h0003;  // Initialize R6
        registers[7] = 16'h0007;  // Initialize R7
    end



always @(posedge clk or posedge reset) begin
        if(RegWrite) begin
        // $display("At time %0t, registers[5] = %d", $time, registers[5]);
				// $display("At time %0t, registers[3] = %d", $time, registers[3]);
				// $display("At time %0t, registers[2] = %d", $time, registers[2]);
				// $display("At time %0t, registers[4] = %d", $time, registers[4]);
				// $display("At time %0t, registers[1] = %d", $time, registers[1]);
				end
				// $display("At time %0t,ALUout= %d", $time,ALUOut);
    end


	assign cz=Instruction[1:0];
	assign Op=Instruction[15:12];

	//data and instruction memory
	assign address=(IorD)?ALUOut:PC;

	initial begin
		$readmemh("/home/kpvivek/verilog/mc_nitc/mem.dat", mem);
	end

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
			// $display("Address=%d MemData=%d",address,MemData);
		mdr <= MemData;
	end

	//register file
	//$r0 is always 0
	assign da = (Instruction[11:9]!=0) ? registers[Instruction[11:9]] : 0;
	assign db = (Instruction[8:6]!=0) ? registers[Instruction[8:6]] : 0;

	always @(posedge clk or posedge reset) begin
        
        // $display("At time %0t, registers[3] = %h", $time, registers[3]);
					// $display("At time %0t,RA= %d", $time,da);
					// if(Instruction[8:6]!=0)
					// $display("At time %0t,db= %d da=%d\n", $time,db,da);
					// $display("At time %0t,OPB= %d ", $time,OpB);
					// $display("regwrite=%d regdst=%d",RegWrite,RegDst);
    end


	always @(posedge clk) begin
		if (RegWrite)begin
			if (RegDst)
			begin
				// $display("op=%d  cz=%d",Op,cz);
				case(Op)
				4'b0000:
				begin
					case(cz)
						2'b00:registers[Instruction[5:3]] <= (MemtoReg) ? mdr : ALUOut;
						2'b10:
						begin
								// $display("carry=%d ",Carry);
								if(Carry==1'b1)begin
								registers[Instruction[5:3]] <= (MemtoReg) ? mdr : ALUOut;
								end
						end
					endcase
				end
				4'b0010:
				begin
					case(cz)
						2'b00:registers[Instruction[5:3]] <= (MemtoReg) ? mdr : ALUOut;
						2'b01:
						begin
								if(Carry==1'b1)
								registers[Instruction[5:3]] <= (MemtoReg) ? mdr : ALUOut;
						end
					endcase
				end
				endcase

			// 	if ((Op == 4'b0000) && (cz == 2'b00)) begin
			// 		$display("at 1");
    	// 	registers[Instruction[5:3]] <= (MemtoReg) ? mdr : ALUOut;
			// end
			// else if ((Op == 4'b0000) && (cz == 2'b10) && (Carry==1'b1)) begin
			// 	$display("at 2");
			// 		registers[Instruction[5:3]] <= (MemtoReg) ? mdr : ALUOut;
			// end
			// else if ((Op == 4'b0010) && (cz == 2'b00)) begin
			// 	$display("at 3");
			// 		registers[Instruction[5:3]] <= (MemtoReg) ? mdr : ALUOut;
			// end
			// else if ((Op == 4'b0010) && (cz == 2'b01) && (Carry==1'b1)) begin
			// 	$display("at 4");
			// 		registers[Instruction[5:3]] <= (MemtoReg) ? mdr : ALUOut;
			// end
			// $display("in regdst %d %d",ALUOut,MemtoReg);
			// 	registers[Instruction[5:3]] <= (MemtoReg) ? mdr : ALUOut;
			// $display("registers[Instruction[5:3]]=%d",registers[Instruction[5:3]]);
			end
			else begin
				registers[Instruction[8:6]]<=(MemtoReg)?mdr:ALUOut;
			end
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
		2'b10:begin
		OpB={{(10){Instruction[5]}},Instruction[5:0]};
		// $display("OpB=%d OpA=%d A=%d ALusrcA=%d",OpB,OpA,A,ALUSrcA);
		end
		2'b11:OpB={{(10){Instruction[5]}},Instruction[5:0]};
		
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
		4'b0010:begin
		{ALUResult} = OpA + OpB;
		if(ALUSrcA==1)begin
		Carry = (OpA) >= 16'hFFFF;
		end
		ZeroFlag = (ALUResult == 0);
		if(Op==4'b1010)
		begin
		{Carry,ALUResult}= A+ OpB;
		ZeroFlag= (ALUResult == 0);
		end
		// $display("hello OP value=%d",Instruction[15:12]);
		end
		4'b0110:ALUResult = OpA - OpB;
		4'b0111:ALUResult = OpA < OpB?1:0;
		4'b1100:ALUResult = ~(OpA | OpB);
		4'b1000:begin
			ALUResult = ~(OpA & OpB);
		ZeroFlag = (ALUResult == 0);
		end
		endcase
		// case(Instruction[15:12])
		// 4'b0010:begin
		// 	$display("in ndu");
		// 	ALUResult = ~(OpA & OpB);
		// $display("Opa=%d OpB=%d Aluresult=%d",OpA,OpB,ALUResult);
		// ZeroFlag = (ALUResult == 0);
		// end
		// endcase

		
	end

	//ALUOut register

	always@(posedge clk) begin
		ALUOut<=ALUResult;
		// $display("aluout is %d",ALUOut);
		// $display("At time %0t, registers[1] = %d", $time, registers[1]);
	end

endmodule


