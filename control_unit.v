// Control unit:
// Inputs: opcode, funct, Zero, clk, reset
// Outputs: PCSrc, ALUSrcA, ALUSrcB, ALUOp, ALUOutEn, RegWrite, IRWrite, RegDst, MemWrite, MemRead, MemReadI, MemtoReg, PCWrite
// Branch is an internal signal i suppose?
module control_unit(
	input wire [5:0] opcode, funct,
	input wire Zero, reset, clk,
	output reg ALUOutEn, RegWrite, IRWrite, RegDst, MemWrite, MemRead, MemReadI, MemtoReg, PCWrite,
	output reg [1:0] ALUSrcA, PCSrc,
	output reg [2:0] ALUSrcB, ALUOp
);

reg [5:0] present_state, next_state;

localparam state0 = 5'd0,state1= 5'd1,state2 = 5'd2, state3= 5'd3, state4= 5'd4, state5= 5'd5,
state6 = 5'd6, state7 = 5'd7, state8 = 5'd8, state9 = 5'd9, state10 = 5'd10, state11 = 5'd11, state12 = 5'd12,
state13 = 5'd13, state14 = 5'd14, state15 = 5'd15, state16 = 5'd16, state17 = 5'd17, state18 = 5'd18, state19 = 5'd19, state20 = 5'd20;

always @(posedge clk) begin
	if (reset)
		present_state = state0;
	else
		present_state = next_state;
end

wire [11:0] casecode = {opcode,funct};

always @(*) begin
	case (present_state)
		state0: begin MemReadI= 1'b1;
		PCWrite = 1'b1;
		IRWrite = 1'b1;
		RegWrite = 1'b0;
		PCSrc = 2'b01;
		ALUSrcA = 2'b00;
		ALUSrcB = 3'b000;
		ALUOp = 3'b100;
		end
		state1: begin MemReadI = 1'b0;
		PCWrite = 1'b0;
		RegWrite = 1'b0; 
		MemRead = 1'b0;
		MemWrite = 1'b0; 
		IRWrite = 1'b0;
		Branch = 1'b0;
		end
		state2: begin ALUSrcA= 2'b01;
		ALUSrcB =3'b001;
		ALUOp = 3'b100;
		ALUOutEn = 1'b1;
		MemReadI = 1'b0;
		PCWrite = 1'b0;
		RegWrite = 1'b0; 
		MemRead = 1'b0;
		MemWrite = 1'b0; 
		IRWrite = 1'b0;
		Branch = 1'b0;
		end
		state3: begin ALUOutEn = 0;
		MemtoReg = 1;
		RegDst = 1;
		RegWrite = 1;
		PCWrite = 0;
		MemReadI = 0;
		MemRead = 0;
		MemWrite = 0;
		IRWrite = 0;
		end
		state4: begin ALUSrcA= 2'b01;
		ALUSrcB =3'b001;
		ALUOp = 3'b110;
		ALUOutEn = 1'b1;
		MemReadI = 1'b0;
		PCWrite = 1'b0;
		RegWrite = 1'b0; 
		MemRead = 1'b0;
		MemWrite = 1'b0; 
		IRWrite = 1'b0;
		Branch = 1'b0;
		end
		state5: begin ALUSrcA= 2'b01;
		ALUSrcB =3'b001;
		ALUOp = 3'b011;
		ALUOutEn = 1'b1;
		MemReadI = 1'b0;
		PCWrite = 1'b0;
		RegWrite = 1'b0; 
		MemRead = 1'b0;
		MemWrite = 1'b0; 
		IRWrite = 1'b0;
		Branch = 1'b0;
		end
		state6: begin ALUSrcA= 2'b01;
		ALUSrcB =3'b001;
		ALUOp = 3'b001;
		ALUOutEn = 1'b1;
		MemReadI = 1'b0;
		PCWrite = 1'b0;
		RegWrite = 1'b0; 
		MemRead = 1'b0;
		MemWrite = 1'b0; 
		IRWrite = 1'b0;
		Branch = 1'b0;
		end
		state7: begin ALUSrcA= 2'b01;
		ALUSrcB =3'b001;
		ALUOp = 3'b101;
		ALUOutEn = 1'b1;
		MemReadI = 1'b0;
		PCWrite = 1'b0;
		RegWrite = 1'b0; 
		MemRead = 1'b0;
		MemWrite = 1'b0; 
		IRWrite = 1'b0;
		Branch = 1'b0;
		end
		state8: begin ALUSrcA= 2'b01;
		ALUSrcB =3'b001;
		ALUOp = 3'b000;
		ALUOutEn = 1'b1;
		MemReadI = 1'b0;
		PCWrite = 1'b0;
		RegWrite = 1'b0; 
		MemRead = 1'b0;
		MemWrite = 1'b0; 
		IRWrite = 1'b0;
		Branch = 1'b0;
		end
		state9: begin ALUSrcA= 2'b01;
		ALUSrcB =3'b001;
		ALUOp = 3'b010;
		ALUOutEn = 1'b1;
		MemReadI = 1'b0;
		PCWrite = 1'b0;
		RegWrite = 1'b0; 
		MemRead = 1'b0;
		MemWrite = 1'b0; 
		IRWrite = 1'b0;
		Branch = 1'b0;
		end
		state10: begin ALUSrcA= 2'b01;
		ALUSrcB =3'b010;
		ALUOp = 3'b100;
		ALUOutEn = 1'b1;
		MemReadI = 1'b0;
		PCWrite = 1'b0;
		RegWrite = 1'b0; 
		MemRead = 1'b0;
		MemWrite = 1'b0; 
		IRWrite = 1'b0;
		Branch = 1'b0;
		end
		state11: begin ALUOutEn = 1'b0;
		MemtoReg = 1'b1;
		RegDst = 1'b0;
		RegWrite = 1'b1;
		PCWrite = 1'b0;
		MemReadI = 1'b0;
		MemRead = 1'b0;
		MemWrite = 1'b0;
		IRWrite = 1'b0;
		end
		state12: begin ALUSrcA= 01;
		ALUSrcB =010;
		ALUOp = 001;
		ALUOutEn = 1;
		MemReadI = 0;
		PCWrite = 0;
		RegWrite = 0; 
		MemRead = 0;
		MemWrite = 0; 
		IRWrite = 0;
		Branch = 0;
		end
		state13: begin ALUSrcA= 01;
		ALUSrcB =010;
		ALUOp = 011;
		ALUOutEn = 1;
		MemReadI = 0;
		PCWrite = 0;
		RegWrite = 0; 
		MemRead = 0;
		MemWrite = 0; 
		IRWrite = 0;
		Branch = 0;
		end
		state14: begin ALUSrcA= 01;
		ALUSrcB =011;
		ALUOp = 100;
		ALUOutEn = 1;
		MemReadI = 0;
		PCWrite = 0;
		RegWrite = 0; 
		MemRead = 0;
		MemWrite = 0; 
		IRWrite = 0;
		Branch = 0;
		end
		state15: begin MemWrite = 0;
		MemRead = 1;
		MemReadI = 0;
		PCWrite = 0;
		IRWrite = 0;
		ALUOutEn = 0;
		RegWrite = 0;
		end
		state16: begin MemWrite = 0;
		MemRead =0;
		MemReadI =0;
		PCWrite =0;
		IRWrite =0;
		RegWrite =1;
		RegDst = 0;
		MemtoReg =0;
		end
		state17: begin ALUOutEn = 0;
		MemWrite =1;
		MemRead =0;
		MemReadI =0;
		IRWrite =0;
		RegWrite =0;
		PCWrite =0;
		end
		state18: begin PCSrc =00;
		PCWrite = 1;
		MemReadI =0;
		end
		state19: begin Branch = 1;
		ALUSrcA = 01;
		ALUSrcB =01;
		ALUOp = 110;
		PCWrite = Branch & Zero;
		PCSrc = 10;
		ALUOutEn = 0;
		MemReadI = 0;
		end
		state20: begin Branch = 1;
		ALUSrcA = 01;
		ALUSrcB =01;
		ALUOp = 110;
		PCWrite = Branch & !Zero;
		PCSrc = 10;
		ALUOutEn = 0;
		MemReadI = 0;
		end
		default: PCWrite = 1'dz;
	endcase

	case (present_state)
		state0: next_state = state1;
		state1: casez (casecode)
			12'000000100000: next_state = state2;
			12'000000100010: next_state = state4;
			12'000000100100: next_state = state5;
			12'000000100101: next_state = state6;
			12'000000100111: next_state = state7;
			12'000000000000: next_state = state8;
			12'000000000010: next_state = state9;
			12'001000??????: next_state = state10;
			12'001101??????: next_state = state12;
			12'001100??????: next_state = state13;
			12'10?011??????: next_state = state14;
			12'000010??????: next_state = state18;
			12'000100??????: next_state = state19;
			12'000101??????: next_state = state20;
			endcase
		state2: next_state = state3;
		state3: next_state = state0;
		state4: next_state = state3;
		state5: next_state = state3;
		state6: next_state = state3;
		state7: next_state = state3;
		state8: next_state = state3;
		state9: next_state = state3;
		state10: next_state = state11;
		state11: next_state = state0;
		state12: next_state = state11;
		state13: next_state = state11;
		state14: next_state = (opcode[3]==1)? state17 : state15;
		state15: next_state = state16;
		state16: next_state = state0;
		state17: next_state = state0;
		state18: next_state = state0;
		state19: next_state = state0;
		state20: next_state = state0;
		default: next_state = state1;
	endcase
endmodule
	
		












	 

	 






