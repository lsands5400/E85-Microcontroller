module controller(input  logic       clk,
                  input  logic       reset,  
                  input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       zero,


                  output logic [1:0] immsrc,
                  output logic [1:0] alusrca, alusrcb,
                  output logic [1:0] resultsrc, 
                  output logic       adrsrc,
                  output logic [2:0] alucontrol,
                  output logic       irwrite, pcwrite, 
                  output logic       regwrite, memwrite);
						
		logic [2:0] aluc;
		logic [1:0] alua, alub, rs, imm, ao;
		logic 		ir, pcup, regw, memw;

		mainFSM f(clk, 
						reset,
						op,
						
						alua, alub,
						rs,
						ao,
						ads,
						ir,
						regw, memw,
						branch, pcup);
		
		decoderALU d(clk,
							reset,  
							op,
							funct3,
							funct7b5,
							
							ao,
							aluc);
						
		instructDecode i(clk,
								reset,
								op,
								
								imm);
								
								
		assign pcwrite = (zero & branch) | pcup;
		assign resultsrc = rs;
		assign adrsrc = ads;
		assign irwrite = ir;
		assign regwrite = regw;
		assign memwrite = memw;
		assign alucontrol = aluc;
		assign immsrc = imm;
		assign alusrca = alua;
		assign alusrcb = alub;

endmodule

// This is the module for the main FSM. 
// The main FSM takes op as an input.
// It outputs Branch, PCUpdate, RegWrite, MenWrite, IRWrite, ResultSrc, 
// ALUSrcA, ALUSrcB, AdrSrc, and ALUOp.
module mainFSM(input  logic       clk,
                  input  logic       reset,  
                  input  logic [6:0] op,

                  output logic [1:0] alusrca, alusrcb,
                  output logic [1:0] resultsrc, 
                  output logic [1:0] aluop,						
                  output logic       adrsrc,
                  output logic       irwrite, 
                  output logic       regwrite, memwrite,
						output logic       branch, pcupdate);

		typedef enum logic[10:0] {S0, S1, S2, S3, S4, S5, S6
											S7, S8, S9, S10} statetype;
		statetype state, nextstate;
		
		// state register 
		always_ff@(posedge clk, posedge reset)
		if(reset) state <= S0;
			else					state <= nextstate;
			
			// next state logic
			always_comb
				case(state)
					S0:																nextstate = S1;
					
					S1: if ((op = 7'b0000011) | (op = 7'b0100011))     nextstate = S2; // lw or sw
						else if (op = 7'b0110011)								nextstate = S6; // R-Type
						else if (op = 7'b0010011)								nextstate = S8; // I-Type ALU
						else if (op = 7'b1101111)								nextstate = S9; // jal
						else if (op = 7'b1100011)								nextstate = S10; // beq
						else 															nextstate = S1;
					
					S2: if ((op = 7'b0000011))    							nextstate = S3; // lw
						else if (op = 7'b0100011)								nextstate = S5; // sw
						else 															nextstate = S2;
					
					S3:																nextstate = S4;
					
					S4:																nextstate = S0;
					
					S5:																nextstate = S0;
					
					S6:																nextstate = S7;
					
					S7:																nextstate = S0;
					
					S8:																nextstate = S7;
					
					S9:																nextstate = S7;
					
					S10:																nextstate = S0;
					
					default: 														nextstate = S0;
					
				endcase
			
			// output logic
			assign adrsrc = (state == S3) | (state == S5);
			assign irwrite = (state == S0);
			assign regwrite = (state == S4) | (state == S7);
			assign memwrite = (state == S5);
			assign branch = (state == S10);
			assign pcupdate = (state == S0) | (state == S9);
			
			assign alusrca[1] = (state == S2) | (state == S6) | (state == S8) | (state == S10);
			assign alusrca[0] = (state == S1) | (state == S9);
			
			assign alusrcb[1] = (state == S0) | (state == S9);
			assign alusrcb[0] = (state == S1) | (state == S2) | (state == S8);
			
			assign resultsrc[1] = (state == S0) | (state == S2);
			assign resultsrc[0] = (state == S4);
			
			assign aluop[1] = (state == S6) | (state == S8);
			assign aluop[0] = (state == S10);

endmodule

// This is the module for the ALU. 
// The ALU takes op, funct3, and funct7, and ALUOp as inputs
// It outputs the ALUControl
module decoderALU(input  logic       clk,
                  input  logic       reset,  
                  input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
						input  logic [1:0] aluop,
                  
                  output logic [2:0] alucontrol);

		always_comb
			case(aluop)
			
				2'b00: 					alucontrol = 3'b000; // lw, sw => add
				2'b01: 					alucontrol = 3'b001; // beq => subtract
				2'b10:
					if (((op[5] = 0) & (funct7b5 = 0) |
							(op[5] = 0) & (funct7b5 = 1) |
							(op[5] = 1) & (funct7b5 = 0)) &
							(funct3 = 3'b000))
											alucontrol = 3'b000; // add => add
								
					else if ((op[5] = 1) & (funct7b5 = 1) &
									(funct3 = 3'b000))
											alucontrol = 3'b001; // sub => subtract
											
					else if ((funct7b5 = 0) &
									(funct3 = 3'b010))
											alucontrol = 3'b101; // slt => set less than
											
					else if ((funct7b5 = 0) &
									(funct3 = 3'b110))
											alucontrol = 3'b011; // or => or
					
					else if ((funct7b5 = 0) &
									(funct3 = 3'b111))
											alucontrol = 3'b010; // and => and
					else					alucontrol = 3'b111; // nothing
					
				default:					alucontrol = 3'b111; // also nothing
					
			endcase
			
			


endmodule

// This is the module for the instruction decoder. 
// The instruction decoder takes op as an input
// It outputs ImmSrc.
module instructDecode(input  logic [6:0] op,
							output logic [1:0] immsrc);
						
		always_comb
			case(op)
			
				7'b0110011: immsrc = 2'b00; // R-type
				7'b0010011: immsrc = 2'b00; // I-type
				7'b0000011: immsrc = 2'b00; // lw
				7'b0100011: immsrc = 2'b01; // sw
				7'b1100011: immsrc = 2'b10; // beq
				7'b1101111: immsrc = 2'b11; // jal
				
			endcase


endmodule