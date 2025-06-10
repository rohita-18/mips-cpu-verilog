module ALU(
	input wire [2:0] ALUOp,
	input wire [31:0] ALU_A,
	input wire [31:0] ALU_B,
	output wire Zero,
	output wire carryout,
	output reg [31:0] ALU_Result
);
	wire [4:0] shamt = ALU_A[4:0];
	wire Binvert = (ALUOp == 3'd6);  // 1 if subtract
	wire [31:0] adder_result;
	wire adder_carryout;

	adder_32 a1(
		.Binvert(Binvert),
		.A(ALU_A),
		.B(ALU_B),
		.Result(adder_result),
		.carryout(adder_carryout)
	);

	always @(*) begin
		case (ALUOp)
			3'd0: ALU_Result = ALU_B << shamt;               // Shift Left
			3'd1: ALU_Result = ALU_A | ALU_B;                // OR
			3'd2: ALU_Result = ALU_B >> shamt;               // Shift Right
			3'd3: ALU_Result = ALU_A & ALU_B;                // AND
			3'd4: ALU_Result = adder_result;                 // ADD
			3'd5: ALU_Result = ~(ALU_A | ALU_B);             // NOR
			3'd6: ALU_Result = adder_result;                 // SUB
			default: ALU_Result = 32'd0;
		endcase
	end

	assign Zero = (ALU_Result == 32'd0);
	assign carryout = (ALUOp == 3'd4 || ALUOp == 3'd6) ? adder_carryout : 0;

endmodule


module adder_32(
	input wire Binvert,
	input wire [31:0] A,
	input wire [31:0] B,
	output reg [31:0] Result,
	output reg carryout
);
	always @(*) begin
		if (Binvert)
			{carryout, Result} = A + ~B + 1;
		else
			{carryout, Result} = A + B;
	end
endmodule
