// Klarein Wassaya - 1210279 - section 2
// Fatima Dawabsheh - 1210827 - section 1
module MUX_2X1_16bits(
  input [15:0] a,
  input [15:0] b,
  input s,
  output reg [15:0] y
);
  always @(*) begin
    y = (s == 1) ? a : b;
  end
endmodule

module MUX_2X1(
  input [2:0] a,
  input [2:0] b,
  input s,
  output reg [2:0] y
);
  always @(*) begin
    y = (s == 1) ? a : b;
  end
endmodule

module MUX_3X1(
  input [2:0] a,
  input [2:0] b,
  input [2:0] c,
  input [1:0] s,
  output reg [2:0] y
);
  always @(*) begin
    case(s)
		2'b00: y = a;
		2'b01: y = b;
		2'b10: y = c;
		default begin
			//do nothing
		end
	endcase
  end
endmodule

module MUX_3X1_16bits(
  input [15:0] a,
  input [15:0] b,
  input [15:0] c,
  input [1:0] s,
  output reg [15:0] y
);
  always @(*) begin
    case(s)
		2'b00: y = a;
		2'b01: y = b;
		2'b10: y = c;
		default begin
			// do nothing
		end
	endcase
  end
endmodule

module PC_control_unit(
  input [3:0] opcode,
  input Z,
  input N,
  input V,
  output reg [1:0] PCsrc
);
  always @(*) begin
    case (opcode)
      4'b1000: PCsrc = (!Z && !(V^N)) ? 2'b01 : 2'b00;
      4'b1001: PCsrc = (!Z && (N^V)) ? 2'b01 : 2'b00;
      4'b1010: PCsrc = (Z) ? 2'b01 : 2'b00;
      4'b1011: PCsrc = (!Z) ? 2'b01 : 2'b00;
      4'b1100, 4'b1101: PCsrc = 2'b10;
      4'b1110: PCsrc = 2'b11;
      default: PCsrc = 2'b00;
    endcase
  end
endmodule

module instruction_memory(
  input [15:0] address,
  output reg [15:0] instruction
);
  reg [7:0] memory [65535:0]; // 65536 bytes of memory, byte-addressable

  initial begin
    // Initial values for the memory
    	memory[0] = 8'h05; 
    	memory[1] = 8'h31; 
    	
		memory[2] = 8'h04; 
    	memory[3] = 8'hD0; 
    	
		memory[4] = 8'h08; 
    	memory[5] = 8'h37; 
    	
		memory[6] = 8'h08; 
    	memory[7] = 8'h37;
		
		memory[8] = 8'h08; 
    	memory[9] = 8'h18; 
    	
		memory[10] = 8'h01; 
    	memory[11] = 8'h37; 
		
		memory[12] = 8'h01; 
    	memory[13] = 8'h33;  
    	
		memory[14] = 8'h18; 
    	memory[15] = 8'h29;
		
		memory[16] = 8'h1E;
		memory[17] = 8'h8C;
			  
		memory[18] = 8'h00;
		memory[19] = 8'hE0;
		
		memory[20] = 8'h00;
		memory[21] = 8'hC0;
		
  end

  always @(*) begin
    // Combine two consecutive bytes to form a 16-bit instruction (Little endian byte ordering)
    instruction = {memory[address + 1], memory[address]};
  end
endmodule

module data_memory(
	input clock,
  	input W_control,
  	input [1:0] R_control,
  	input [2:0] address,
  	input [15:0] data_in,
  	output reg [15:0] data_out
);
	reg [7:0] memory [65535:0]; // 65536 bytes of memory, byte-addressable
  
  	initial begin
    	// Initial values for the memory
    	memory[0] = 8'h84; 
    	memory[1] = 8'h02; 
    	memory[2] = 8'h58; 
    	memory[3] = 8'h14; 
    	memory[4] = 8'hE0; 
    	memory[5] = 8'h2A; 
    	memory[6] = 8'h40; 
    	memory[7] = 8'h75;
		memory[8] = 8'h3F;
		memory[9] = 8'h31;
		memory[10] = 8'h0C;
		memory[11] = 8'h89;
		memory[12] = 8'h00;
		memory[13] = 8'hC0;
		memory[14] = 8'h01;
		memory[15] = 8'hC0;
  	end

  	always @(posedge clock) begin
    	if (R_control == 2'b01) begin   
			data_out = {memory[address + 1], memory[address]}; // read for instruction LW
    	end	
		if (R_control == 2'b10) begin
			data_out = {8'b00000000 ,memory[address]}; // LBu
		end
		if (R_control == 2'b11) begin
			data_out = {{(8){memory[address][7]}} ,memory[address]}; // LBs
		end
    	if (W_control) begin
    		{memory[address + 1], memory[address]} = data_in; // write
    	end
  	end
endmodule

module ALU_control_unit(
	input [3:0] opcode,
	output reg [1:0] ALUop,
);
	
	always @ (*) begin
		case (opcode)
			4'b0000, 4'b0100: ALUop = 2'b00; //AND
			4'b0010, 4'b1000, 4'b1001, 4'b1010, 4'b1011: ALUop = 2'b01; //SUB
			4'b0001, 4'b0011, 4'b0101, 4'b0110, 4'b0111: ALUop = 2'b10; //ADD
			default: ALUop = 2'b11; //NO OPERATION TO DO (jmp, call, Sv, ret)
		endcase
	end
	
	endmodule
	
module ALU(
  input [1:0] ALUop,
  input signed [15:0] a,
  input signed [15:0] b,
  output reg Z,
  output reg N,
  output reg V,
  output reg signed [15:0] res
);
	assign Z = (res == 16'b0) ? 1'b1 : 1'b0; // zero flag
	assign N = res[15]; // negative flag
	assign V = ((a[15] != b[15]) && (res[15] != a[15])) ? 1'b1 : 1'b0;
	
  always @(*) begin
	case (ALUop)
      2'b00: begin	// AND
        res = a & b;
      end
      2'b01: begin // SUB
        res = a - b;
      end
      2'b10: begin // ADD
        res = a + b;
      end
      2'b11: begin
        //nothing to do
      end
      default: begin
        //nothing to do
      end
    endcase 
  end
endmodule

module extender (
  input extOp,
  input [4:0] a,
  output reg [15:0] out
);
  always @(*) begin
    if ( extOp == 0 ) begin
      out = {{(11){1'b0}}, a};
    end
    else if ( extOp == 1 ) begin
      out = {{(11){a[4]}}, a};
    end
  end
endmodule

module extender2 (
  input extOp,
  input [7:0] a,
  output reg [15:0] out
);
  always @(*) begin
    if ( extOp == 0 ) begin
      out = {{(8){1'b0}}, a};
    end
    else if ( extOp == 1 ) begin
      out = {{(8){a[7]}}, a};
    end
  end
endmodule

module extender3 (
  input [2:0] rs,
  output reg [15:0] out
);
  always @(*) begin		   
	out = {{(13){1'b0}}, rs};
  end
endmodule

module register_file(
  	input clk,
	input regWrite,
  	input [2:0] RA,
  	input [2:0] RB,
  	input [2:0] RW,
  	input [15:0] BusW,
  	output reg [15:0] BusA,
  	output reg [15:0] BusB,
	output reg signed [15:0] registers [7:0]
);
  
  
  initial begin
    // Initial values for the registers
    registers[0] = 16'h0000;
    registers[1] = 16'h0120;
    registers[2] = 16'h1032;
    registers[3] = 16'h0040;
    registers[4] = 16'h0156;
    registers[5] = 16'h0341;
    registers[6] = 16'h0C4D;
    registers[7] = 16'hBE86;
  end
  
  assign BusA = registers[RA];
  assign BusB = registers[RB];
  always @(posedge clk) begin
	  if(regWrite == 1) begin
		  registers[RW] = BusW;
	  end
  end
    
endmodule

module MUX_4X1_16bits(
  	input [1:0] PCsrc,
  	input [11:0] jump_target_address, // PC[15:11] || jump_offset
  	input [15:0] branch_target_address, // PC + sign_extended(imm)
  	input [15:0] return_address, // PC + 4 (ret instruction)
  	input [15:0] PC, // PC + 2
  	output reg [15:0] nextPC
);
  	always @(*) begin
    	case (PCsrc)
    	  2'b00: nextPC = PC;
    	  2'b01: nextPC = branch_target_address;
    	  2'b10: nextPC = jump_target_address;
    	  2'b11: nextPC = return_address;
    	  default: begin
			  //nothing to do 
		  end
    	endcase
  	end
endmodule

module Add_4 (
	input [15:0] PC,
	output reg [15:0] result
);
	assign result = PC + 16'h0004;
endmodule

module Add_2 (
	input [15:0] PC,
	output reg [15:0] result
);
	assign result = PC + 16'h0002;
endmodule

module ADDER (
	input [15:0] PC,
	input [15:0] immediate,
	output reg [15:0] branch_target_address
);
	assign branch_target_address = PC + immediate;
endmodule

module control_signals(
	input [15:0] instruction,
	output reg [1:0] R_control,
  	output reg W_control,
  	output reg [1:0] Rs2_src,
  	output reg ALUsrcB,
  	output reg extOp,
  	output reg regWrite,
  	output reg [1:0] WBdata,
  	output reg data_in_src,
	output reg data_address_src,
	output reg [1:0] Rs1ctrl,
	output reg [1:0] Rdctrl
);	

	reg lock_R7;
	initial begin
		lock_R7 = 0;
	end
	
  	always @(*) begin
    	case (instruction[15:12])
      		4'b0000, 4'b0001, 4'b0010: begin // AND, ADD, SUB (R-type)
        		R_control = 0;
        		Rdctrl = 1;
				Rs1ctrl = 2;
				W_control = 0;
        		ALUsrcB = 0;
        		Rs2_src = 0;
				case (instruction[11:9])
					3'b000: regWrite = 0;
					3'b111: begin
						if( lock_R7 == 1'b1 ) begin
							regWrite = 0;
						end
						else begin
							regWrite = 1;
						end
					end
					default: regWrite =1;
				endcase
				WBdata = 0;
      		end
      		4'b0011: begin // ADDI (I-type)
				R_control = 0;
				Rdctrl = 0;
        		Rs1ctrl = 1;
				W_control = 0;
    			ALUsrcB = 1;
        		extOp = 1;
        		case (instruction[10:8])
					3'b000: regWrite = 0;
					3'b111: begin
						if( lock_R7 == 1'b1 ) begin
							regWrite = 0;
						end
						else begin
							regWrite = 1;
						end
					end
					default: regWrite =1;
				endcase
				WBdata = 0;
			end
			4'b0100: begin // ANDI (I-type)
        		R_control = 0;
        		W_control = 0;
        		Rs1ctrl = 1;
				Rdctrl = 0;
				ALUsrcB = 1;
        		extOp = 0;
        		case (instruction[10:8])
					3'b000: regWrite = 0;
					3'b111: begin
						if( lock_R7 == 1'b1 ) begin
							regWrite = 0;
						end
						else begin
							regWrite = 1;
						end
					end
					default: regWrite =1;
				endcase
				WBdata = 0;
      		end
      		4'b0101: begin // LW (I-type)
        		R_control = 1;
        		W_control = 0;
        		Rs1ctrl = 1;
				Rdctrl = 0;
				ALUsrcB = 1;
        		extOp = 1;
        		data_address_src = 1;
				case (instruction[10:8])
					3'b000: regWrite = 0;
					3'b111: begin
						if( lock_R7 == 1'b1 ) begin
							regWrite = 0;
						end
						else begin
							regWrite = 1;
						end
					end
					default: regWrite =1;
				endcase
				WBdata = 1;
      		end			  
			4'b0110: begin // LBu, LBs (I-type)
				W_control = 0;
        		Rs1ctrl = 1;
				ALUsrcB = 1;
        		Rdctrl = 0;
    			extOp = 1;
				data_address_src = 1;
				if ( instruction[11] == 1'b0 ) begin
					R_control = 2;
				end
				else begin
        			R_control = 3;
				end
        		case (instruction[10:8])
					3'b000: regWrite = 0;
					3'b111: begin
						if( lock_R7 == 1'b1 ) begin
							regWrite = 0;
						end
						else begin
							regWrite = 1;
						end
					end
					default: regWrite =1;
				endcase
				WBdata = 1;
      		end
      		4'b0111: begin // SW (I-type)
        		R_control = 0;
        		W_control = 1;
        		Rs1ctrl = 1;
				ALUsrcB = 1;
				Rs2_src = 2;
        		extOp = 1;
        		regWrite = 0;
				data_in_src = 1;
				data_address_src = 1;
      		end
      		4'b1000, 4'b1001, 4'b1010, 4'b1011: begin // BGT, BGTZ, BLT, BLTZ, BEQ, BEQZ, BNE, BNEZ
        		Rs1ctrl = 0;
				R_control = 0;
        		W_control = 0;
        		extOp = 1;
        		ALUsrcB = 0;
        		Rs2_src = 2;
       	 		regWrite = 0;
      		end
      		4'b1100: begin // JMP
        		R_control = 0;
        		W_control = 0;
        		regWrite = 0;
			end
      		4'b1101: begin // CALL
        		R_control = 0;
        		W_control = 0;
        		regWrite = 1;
				Rdctrl = 2;
        		WBdata = 2;
				lock_R7 = 1;
      		end
      		4'b1110: begin // RET
				R_control = 0;
        		W_control = 0;
        		regWrite = 0;
				Rs2_src = 1;
				lock_R7 = 0;
      		end
      		4'b1111: begin // Sv
        		R_control = 0;
        		W_control = 1;
        		data_in_src = 0;
        		data_address_src = 0;
        		regWrite = 0;
      		end
      		default: begin
        	lock_R7 = 0;
      		end
		endcase
	end
endmodule
        
module top_design(
	input clk,	
	output reg signed [15:0] registers [7:0],
	output reg [15:0] IR
);
	reg [15:0] PC;
	reg [15:0] instruction;
	reg [15:0] res;
	wire [15:0] Rs, return_address, nextPC, BusW, BusA, BusB, address, data_in, data_out, busB, PC_plus4, PC_plus2;
	wire [1:0] PCsrc;
  	wire [15:0] jump_target_address, branch_target_address;
  	wire [1:0] R_control, Rs1ctrl, Rdctrl, Rs2_src, WBdata;
	wire W_control, ALUsrcB, extOp, regWrite, data_in_src, data_address_src, lock_R7;
	wire [2:0] Rd, Rs1, Rs_2, Rs1_1;
	wire signed [15:0] I_type_immediate, S_type_immediate;
	wire Z, N, V;
	reg signed [15:0] A, B, ALUout, MDR;
	reg [2:0] state, RW;
	wire [1:0] ALUop;
	reg write;
	
	initial begin
		state = 3'b000;
		PC <= 16'h0000;
	end
	
//	wire [11:0] shifted_jmp_offset;
//	assign shifted_jmp_offset = IR[11:0] * 2;
	assign jump_target_address = {PC[15:12], (IR[11:0]*2)}; // determine the jump_target_address for jmp and call instructions
	
	// determine the ALUop depending on the opcode from the instruction
	ALU_control_unit h1(
	.opcode(IR[15:12]),
	.ALUop(ALUop)
	);
	
	// PC + 4 to store it in R7 for CALL instruction
	Add_4 A1(
	.PC(PC),
	.result(PC_plus4)
	);
	
	// PC + 2 to fetch the next	instruction
	Add_2 A2(
	.PC(PC),
	.result(PC_plus2)
	);
	
	// PC + sign_extended(immediate) to determine the branch target
	ADDER A3(
	.PC(PC),
	.immediate(I_type_immediate),
	.branch_target_address(branch_target_address)
	);
	
	// select the next PC to fetch the next instruction
	MUX_4X1_16bits M1(
	.PCsrc(PCsrc),
	.jump_target_address(jump_target_address),
	.branch_target_address(branch_target_address),
	.return_address(BusB),
	.PC(PC_plus2),
	.nextPC(nextPC)
	);
	
	// zero extension for Rs in Sv instruction
	extender3 E1(
	.rs(IR[11:9]),
	.out(Rs)
	);
	
	// extender for I-type instructions immediate value
	extender E2(
	.a(IR[4:0]),
	.extOp(extOp),
	.out(I_type_immediate)
	);
	
	// extender for Sv (S-type) immediate value
	extender2 E3(
	.a(IR[8:1]),
	.extOp(extOp),
	.out(S_type_immediate)
	);
	
	// fetch the instruction from the memory
	instruction_memory M2(
	.address(PC),
	.instruction(instruction)
	);
	
	// determine the control signals for the instruction
	control_signals M3(
	.instruction(IR),
	.R_control(R_control), 
	.W_control(W_control), 
	.Rs2_src(Rs2_src), 
	.ALUsrcB(ALUsrcB), 
	.extOp(extOp),
	.regWrite(regWrite), 
	.WBdata(WBdata), 
	.data_in_src(data_in_src), 
	.data_address_src(data_address_src),
	.Rs1ctrl(Rs1ctrl),
	.Rdctrl(Rdctrl)
	);
	
	// mux for choosing what to compare Rd with depending on m (IR[11]) , m = 0 --> compare Reg[Rd] with Reg[Rs1] , m = 1 --> compare Reg[Rd] with Reg[R0]
	MUX_2X1 X0(
	.a(3'b000),
	.b(IR[7:5]),
	.s(IR[11]),
	.y(Rs1_1)
	);
	
	// Mux for Rd
	MUX_3X1 X2(
	.a(IR[10:8]),
	.b(IR[11:9]),
	.c(3'b111),
	.s(Rdctrl),
	.y(Rd)
	);
	
	// mux for Rs1
	MUX_3X1 X1(
	.a(Rs1_1), // branch
	.b(IR[7:5]), // I-Type
	.c(IR[8:6]), // R-type
	.s(Rs1ctrl),
	.y(Rs1)
	);
	
	// mux for Rs2
	MUX_3X1 M12(
	.a(IR[5:3]),
	.b(3'b111),
	.c(Rd),
	.s(Rs2_src),
	.y(Rs_2)
	);
	
	// register file for controlling reading and writing in the registers
	register_file M5(
	.clk(clk),
	.regWrite(write),
  	.RA(Rs1),
  	.RB(Rs_2),
  	.RW(RW),
  	.BusW(MDR),
  	.BusA(BusA),
	.BusB(BusB),
	.registers(registers)
	);
	
	// determine the operand (sourceB) of the ALU
	MUX_2X1_16bits M14(
	.a(I_type_immediate),
	.b(BusB),
	.s(ALUsrcB),
	.y(busB)
	);
	
	// ALU inputs and outputs
	ALU M6(
  	.ALUop(ALUop),
  	.a(A),
  	.b(B),
	.Z(Z),
	.N(N),
	.V(V),
  	.res(res)
	);
	
	// determine the source for the PC
	PC_control_unit M8(
	.opcode(IR[15:12]),
	.Z(Z),
	.N(N),
	.V(V),
	.PCsrc(PCsrc)
	);		
	
	// determine the address for the data memory to read from or write in
	MUX_2X1_16bits M10(
	.a(ALUout),
	.b(Rs),
	.s(data_address_src),
	.y(address)
	);
	
	// determine the data to be stored in the data memory
	MUX_2X1_16bits M11(
	.a(BusB),
	.b(S_type_immediate),
	.s(data_in_src),
	.y(data_in)
	);
	
	// data memory for load and store instructions
	data_memory M9(
	.clock(clk),
  	.W_control(W_control),
  	.R_control(R_control),
  	.address(address),
  	.data_in(data_in),
  	.data_out(data_out)
	);				   
	
	// determine the data to be stored in the register file
	MUX_3X1_16bits M15(
	.a(ALUout),
	.b(data_out),
	.c(PC_plus4),
	.s(WBdata),
	.y(BusW)
	);
	
	always @(posedge clk) begin
		
    	case (state)
      		3'b000: begin // Fetch
				IR = instruction;
        		state = 3'b001;
      		end
      		3'b001: begin // Decode
        		A = BusA;
				B = busB;
			    case (IR[15:12]) 
					4'b1000, 4'b1001, 4'b1010, 4'b1011: begin //Bracnch instructions
						PC = nextPC;
						state = 3'b000;
					end				   
					4'b1100, 4'b1110: begin //JMP, RET
						PC = nextPC;
						state = 3'b000;
					end				   
					4'b1101: begin //CALL
						state = 3'b100;
					end
					4'b1111: begin //Sv
						state = 3'b011;
					end
        		    default: state = 3'b010;
				endcase
      		end
      		3'b010: begin // Execute (ALU)
				ALUout = res;
				case (IR[15:12]) 
					4'b0000, 4'b0001, 4'b0010, 4'b0011, 4'b0100: begin //ADD, AND, SUB, ADDI, ANDI
						state = 3'b100;
					end
        		    default: state = 3'b011;
				endcase
      		end
      		3'b011: begin // Memory Access
       			case (IR[15:12]) 
					4'b0111, 4'b1111: begin	//SW, Sv
						PC = nextPC;
						state = 3'b000;
					end
        		    default: state = 3'b100;
				endcase
      		end
      		3'b100: begin // Write-Back
        		write = regWrite;
				RW = Rd;
				MDR = BusW;
        		PC = nextPC;
				state = 3'b000;
      		end
			default: state = 3'b000;
    	endcase
  	end
	
endmodule	   

`timescale 1ns / 1ps

module tb;
	reg clk;
	reg signed [15:0] registers [7:0];
	reg [15:0] IR;

  // Instantiate the CPU
  top_design uut (
  	.clk(clk),
  	.registers(registers),
	.IR(IR)
	);

  // Clock generation
  always #5 clk = ~clk;

  // Initial stimulus
  initial begin
    // Initialize Inputs
    clk = 0;

    // Run the simulation for a certain period
    #600;

    // Finish the simulation
    $finish;
  end

endmodule
