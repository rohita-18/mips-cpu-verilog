// PC_mux
// Inputs: PCSrc, PC_Jump, PC_ALUOut,PC_ALUresult
// Outputs: PC

// ALU_A_mux:
// Inputs : PC,Reg_A,shamt, ALUSrcA
// Outputs : ALU_A

// ALU_B_mux:
// Inputs : Reg_B, SignExtImm, SignExtAddr, ZeroExtImm, ALUSrcB
// Outputs: ALU_B

// ALUOut:
// Inputs : ALU_Result, clk, ALUOutEn
// Output: ALUOut

module PC_mux(
	input wire [1:0] PCSrc,
	input wire [31:0] PC_ALUOut,PC_ALUresult,PC_Jump,
	output reg [31:0] PC
	);
always @(*) begin
	case (PCSrc)
		2'b00: PC = PC_Jump;
		2'b01: PC = PC_ALUresult;
		2'b10: PC = PC_ALUOut;
		default: PC = 32'd0;
	endcase
end
endmodule

module ALU_A_mux(
	input wire [31:0] PC,
	input wire [31:0] Reg_A,
	input wire [31:0] shamt,
	input wire [1:0] ALUSrcA,
	output reg [31:0] ALU_A
	);
always @(*) begin
	case (ALUSrcA)
	2'b00: ALU_A = PC;
	2'b01: ALU_A = Reg_A;
	2'b10: ALU_A = shamt;
	default: ALU_A = 32'd0;
	endcase
end
endmodule

module ALU_B_mux(
	input wire [31:0] Reg_B, SignExtImm, SignExtAddr, ZeroExtImm,
	input wire [2:0] ALUSrcB,
	output reg [31:0] ALU_B
	);
localparam [31:0] four = 32'd4;
always @(*) begin
	case (ALUSrcB)
	3'b000: ALU_B = four;
	3'b001: ALU_B = Reg_B;
	3'b010: ALU_B = SignExtImm;
	3'b011: ALU_B = SignExtAddr;
	3'b100: ALU_B = ZeroExtImm;
	default: ALU_B = 32'd0;
	endcase
end
endmodule

//interconnect will have to extend shamt to 32 bits

// IR:
// Inputs: IRWrite, Instr, clk
// Outputs: Reg_rs,Reg_rt, Reg_rd, Imm, shamt, funct, opcode, IR

// PC:
// Inputs : PCWrite, PC,clk
// Outputs: PC

// MemtoReg_mux:
// Inputs: MDR, ALUOut, MemtoReg
// Outputs: Reg_write_data

module Instr_reg(
	input wire IRWrite, clk,
	input wire [31:0] Instr,
	output reg [4:0] Reg_rd, Reg_rs, Reg_rt, shamt,
	output reg [5:0] opcode, funct,
	output reg [15:0] Imm,
	output reg [31:0] IR
	);
always @(posedge clk) begin
	if (IRWrite)
		IR <= Instr; //use non blocking assignment
end
always @(*) begin
	Reg_rd = IR[15:11];
	Reg_rs = IR[25:21];
	Reg_rt = IR[20:16];
	shamt = IR[10:6];
	opcode = IR[31:26];
	funct = IR[5:0];
	Imm = IR[15:0];
end

initial IR = 32'd0;
endmodule

module PC_reg(
	input wire clk, reset, PCWrite,
	input wire [31:0] PC_selected,
	output [31:0] PC
	);
always @(posedge clk) begin
	if (reset) 
		PC <= 32'd0;
	else if (PCWrite) 
		PC <= PC_selected;
end
endmodule

module MemtoReg_mux(
	input wire MemtoReg,
	input wire [31:0] ALUOut, MDR,
	output wire [31:0] Reg_write_data
	);
assign Reg_write_data = (MemtoReg)? ALUOut : MDR;
endmodule

module ALUOut_reg(
	input wire ALUOutEn,clk,reset,
	input wire [31:0] ALU_Result,
	output reg [31:0] ALUOut
	);
always @(posedge clk) begin
	if(reset) 
		ALUOut <= 32'd0;
	else if(ALUOutEn)
		ALUOut <= ALU_Result;
end	
endmodule

module Reg_A(
	input wire [31:0] read_data_1,
	input wire clk,
	output reg [31:0] Reg_A
	);
always @(posedge clk) begin
	Reg_A <= read_data_1;
end
endmodule

module Reg_B(
	input wire [31:0] read_data_2,
	input wire clk,
	output reg [31:0] Reg_B
	);
always @(posedge clk) begin
	Reg_B <= read_data_2;
end
endmodule

module MDR_reg(
	input wire [31:0] mem_data,
	input wire clk,
	output reg [31:0] MDR
	);
always @(posedge clk) begin
	MDR <= mem_data;
end
endmodule

// Register_File:
// Inputs : Read_reg_1, Read_reg_2, Write_reg, RegWrite, write_data, clk
// Outputs: Read_data_1, Read_data_2

module Register_File(
    input wire [4:0] Read_reg_1, Read_reg_2, Write_reg,
    input wire [31:0] write_data,
    input wire RegWrite, clk,
    output reg [31:0] Read_data_1, Read_data_2
);
    reg [31:0] registers [0:31];

    initial begin
        registers[0]  = 32'h00000000;
        registers[1]  = 32'h00000001;
        registers[2]  = 32'h10101010;
        registers[3]  = 32'h11110000;
        registers[4]  = 32'h00101010;
        registers[5]  = 32'h10101010;
        registers[6]  = 32'h00000000;
        registers[7]  = 32'h11000100;
        registers[8]  = 32'h01100100;
        integer i;
        for (i = 9; i < 32; i = i + 1)
            registers[i] = 32'h00000000;
    end

    always @(posedge clk) begin
        if (RegWrite && Write_reg != 0)
            registers[Write_reg] <= write_data;
    end

    always @(*) begin
        Read_data_1 = registers[Read_reg_1];
        Read_data_2 = registers[Read_reg_2];
    end
endmodule

// Instr_mem:
// Inputs: Instr_Addr, MemReadI,clk
// Outputs: Instr
module Instr_mem(Instr, Instr_Addr, MemReadI);

    output reg [31:0] Instr;
    input [31:0] Instr_Addr;
    input MemReadI;
    
    reg [31:0] ins_mem [0:127];   
       
    initial begin
        $readmemb("ins_mem.dat", ins_mem);
    end
    
    always @ (*) begin
        if(MemReadI) begin
            Instr = ins_mem[Instr_Addr[8:2]];
        end
    end

endmodule

// Data_mem:
// Inputs: Data_Addr, MemRead, MemWrite, Mem_write_data,clk
// Outputs: Mem_data

module Data_mem(
	input wire [31:0] Data_Addr, Mem_write_data,
	input wire MemRead, MemWrite,clk,
	output reg [31:0] mem_data
	);

reg [31:0] data_mem [0:127];

initial 
begin 
	$readmemb("dat_mem.dat" , data_mem);
end

always @(*) begin
	if(MemRead)
		mem_data = data_mem[Data_Addr[8:2]];
	else 
		mem_data = 32'd0;
end

always @(posedge clk) begin
	if(MemWrite) begin
		data_mem[Data_Addr[8:2]] <= Mem_write_data;
		$writememb("dat_mem.dat" , data_mem);
	end
end
endmodule


	
