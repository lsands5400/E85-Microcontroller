module top(input  logic        clk, reset, 

           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);
		
		logic[31:0] readdata, instr, adr
		
		memory				mem(clk, MemWrite, adr, WriteData, );
		riscv					multi(clk, reset, readdata, instr,
											adr, WriteData, MemWrite);
		assign DataAdr = adr;
endmodule


//
//
//
// Processor Module
// This is the module for the processor. 
// The processor takes clk, reset, ReadData, 
// and the instruction as an input.
// It outputs PC, ALUResult, and WriteData, and MemWrite to the memory.
module riscv(input logic					clk,
				input logic						reset,
				input logic [31:0]			readdata,
				input logic [31:0]			instr,
				
				output logic [31:0] 		adr,
				output logic [31:0]		writedata,
				output logic				memwrite);
				
		logic				regwrite, alusrca, alusrcb, pcwrite, irwrite,
								adrsrc, zero;
		logic	[1:0]		immsrc, resultsrc
		logic	[2:0]		alucontrol
		
	
		controller 				ct(clk, reset,
										instr[6:0], instr[14:12], instr[30], zero, 
										immsrc, alusrca, alusrcb,
										resultsrc, adrsrc, alucontrol,
										irwrite, pcwrite, regwrite, memwrite);
										
		datapath 				dp(clk, reset,
										regwrite, immsrc, alusrca, alusrcb,
										alucontrol, resultsrc, pcwrite, irwrite,
										adrsrc, instr, readdata, 
										zero, adr, writedata);

endmodule



//
//
//
// Datapath Modules	
module datapath(input logic			clk, reset,
					input logic				regwrite,
					input logic	[1:0]		immsrc,
					input logic				alusrca, alusrcb,
					input logic	[2:0]		alucontrol,
					input logic	[1:0]		resultsrc,
					input logic				pcwrite, irwrite,
					input logic				adrsrc,
					input logic	[31:0]	instr,
					input logic	[31:0]	readdata,
					
					output logic				zero,
					output logic [31:0]		adr,
					output logic [31:0]		writedata);
					
	  logic [31:0] pcnext, pc, oldpc;
	  logic [31:0] adr;
	  logic [31:0] immext;
	  logic [31:0] a;
	  logic [31:0] srca, srcb;
	  logic [31:0] srcatemp, writedatatemp;
	  logic [31:0] result, aluresult;
	  logic [31:0] aluout;

	  // adr logic
	  enflopr #(32) 	pcreg(clk, reset, pcwrite, pcnext, pc);
	  adrmux  			mux1(pc, result, adrsrc, adr);    
	  mux2 #(32)  		pcmux(PCPlus4, PCTarget, PCSrc, PCNext);
	  
	  // register file logic
	  enflopr #(32) 	instrreg(clk, reset, irwrite, readdata, instr);
	  regfile 			rf(clk, regwrite, instr[19:15], instr[24:20],
									instr[11:7], result, srcatemp, writedatatemp);	
	  flopr #(32)		srcareg(clk, reset, srcatemp, a);
	  flopr #(32)		writereg(clk, reset, writedatatemp, writedata);
	  extend      		ext(instr[31:7], immsrc, immext);	  
	 
	  // ALU logic
	  srcamux 			amux(pc, oldpc, a, alusrca, srca);
	  srcbmux 			bmux(writedata, immext, alusrcb, srcb);
	  alu 				alu1(srca, srcb, alucontrol, aluresult, zero);
	 
	  // result logic
	  flopr #(32) 		datareg(clk, reset, readdata, data);
	  resmux				rmux(aluout, data, aluresult, resultsrc, result);
		
	  // PC logic
	  assign 			pcnext = result;
	  enflopr #(32) 	oldpcreg(clk, reset, irwrite, pc, oldpc);
endmodule

// This is the mux that takes AdrSrc to decide whether to 
// take the PC value or the result value to put into the memory. 
module adrmux #(parameter WIDTH = 8)
					(input  logic [WIDTH-1:0] 	pc, result, 
					input  logic             	adrsrc,
					output logic [WIDTH-1:0] 	adr);

	assign adr = adrsc ? pc : result; 

endmodule

// This is the mux that takes alusrca to decide whether to 
// take the PC value, the OldPC value, or the A value to put into the ALU. 
module srcamux #(parameter WIDTH = 8)
					(input  logic [WIDTH-1:0] 			pc, oldpc, a 
					input  logic [1:0]             	alusrca, 
					output logic [WIDTH-1:0] 			srca);

	assign srca = alusrca[1] ? a : (alusrca[0] ? pc : oldpc); 

endmodule


// This is the mux that takes alusrcb to decide whether to take the write/readdata 
// value, the extended immediate, or the value 4 to put into the ALU. 
module srcbmux #(parameter WIDTH = 8)
					(input  logic [WIDTH-1:0] 			wdrd, immext, 
					input  logic [1:0]             	alusrcb, 
					output logic [WIDTH-1:0] 			srcb);

	assign srcb = alusrcb[1] ? 4 : (alusrcb[0] ? wdrd : oldpc); 
	
endmodule

// This is the mux that takes resultsrc to decide whether to take the aluout
// value, the data value, or the aluresult value to put into the result path. 
module resmux #(parameter WIDTH = 8)
					(input  logic [WIDTH-1:0] 			aluout, data, aluresult
					input  logic [1:0]             	resultsrc, 
					output logic [WIDTH-1:0] 			result);

	assign result = resultsrc[1] ? aluresult : (resultsrc[0] ? aluout : data); 
 
endmodule

// This is the ALU. It performs different operations based on the alucontrol
// input value.
module alu(input logic [31:0]				srca,
				input logic	[31:0]			srcb,
				input logic[2:0]				alucontrol,
				
				output logic [31:0]			aluresult,
				output logic					zero);
				
		logic [31:0] condinvb, sum;
		logic        sub;
  
		assign sub = (alucontrol[1:0] == 2'b01);
		assign condinvb = sub ? ~b : b; // for subtraction or slt
				
		always_comb
			case(alucontrol)
				3'b000: aluresult = sum;  			// add
				3'b001: aluresult = sum;			// subtract
				3'b010: aluresult = srca & srcb;	// and
				3'b011: aluresult = srca | srcb;	// or
				3'b101: aluresult = sum[31];		// set less than
				default:	aluresult =0;				
			endcase

		assign zero = (result == 32'b0);
		
endmodule

// This extends the immediate based on the immsrc value
module extend(input logic [31:7]			instr,
					input logic	[1:0]			immsrc,
					
					output logic [31:0]		immext);
					
		always_comb
			case(ImmSrc) 
							// I-type 
				2'b00:   ImmExt = {{20{Instr[31]}}, Instr[31:20]};  
							// S-type (Stores)
				2'b01:   ImmExt = {{20{Instr[31]}}, Instr[31:25], Instr[11:7]}; 
							// B-type (Branches)
				2'b10:   ImmExt = {{20{Instr[31]}}, Instr[7], Instr[30:25], Instr[11:8], 1'b0}; 
							// J-type (Jumps)
				2'b11:   ImmExt = {{12{Instr[31]}}, Instr[19:12], Instr[20], Instr[30:21], 1'b0}; 
				default: ImmExt = 32'bx; // undefined
			endcase             
		
endmodule

// This is the register file module. The register file writes and reads to
// registers depending on the write enable. 
module regfile(input  logic        clk, 
               input  logic        WE3, 
               input  logic [ 4:0] A1, A2, A3, 
               input  logic [31:0] WD3, 
               output logic [31:0] RD1, RD2);

		logic [31:0] rf[31:0];

		  // three ported register file
		  // read two ports combinationally (A1/RD1, A2/RD2)
		  // write third port on rising edge of clock (A3/WD3/WE3)
		  // register 0 hardwired to 0

		 always_ff @(posedge clk)
			 if (WE3) rf[A3] <= WD3;	

		 assign RD1 = (A1 != 0) ? rf[A1] : 0;
		 assign RD2 = (A2 != 0) ? rf[A2] : 0;
endmodule

// This is the flip flop module with an enable.
module enflopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
					input  logic				 en,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset & en) q <= 0;
    else       q <= d;
endmodule

// This is the flip flop module.
module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule


//
//
//
// Memory Modules

// This is the module for the joint instruction and data memory unit. 
// The memory reads or writes data based on the write enable. 
module memory(input logic 					clk,
					input logic					memwrite,
					input logic [31:0]		dataadr,
					input logic	[31:0]		writedata,
					
					output logic		readdata);
					
		logic [31:0] RAM[63:0];
		
		always_ff @(posedge clk)
			if (memwrite) RAM[A[31:2]] <= WD;

		initial $readmemh("memfile.dat",RAM);
		
		assign readdata = RAM[dataadr[31:2]];
				
endmodule
					

//
//
//
// Controller Modules

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
						
		instructDecode inst(op,
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

		typedef enum logic[10:0] {S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10} statetype;
		statetype state, nextstate;
		
		// state register 
		always_ff@(posedge clk, posedge reset)
		if(reset) state <= S0;
			else					state <= nextstate;
			
			// next state logic
			always_comb
				case(state)
					S0:																nextstate = S1;
					
					S1: 
						case(op)
							7'b0000011:						      nextstate = S2; // lw
							7'b0100011:						      nextstate = S2; // sw
							7'b0110011:								nextstate = S6; // R-Type
							7'b0010011:								nextstate = S8; // I-Type ALU
							7'b1101111:								nextstate = S9; // jal
							7'b1100011:								nextstate = S10; // beq
							default: 								nextstate = S1;
						endcase
					
					S2: 
						case(op)
							7'b0000011:    							nextstate = S3; // lw
							7'b0100011:									nextstate = S5; // sw
							default: 									nextstate = S2;
						endcase
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
			
			assign resultsrc[1] = (state == S0);
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
					if (((op[5] == 0) & (funct7b5 == 0) |
							(op[5] == 0) & (funct7b5 == 1) |
							(op[5] == 1) & (funct7b5 == 0)) &
							(funct3 == 3'b000))
											alucontrol = 3'b000; // add => add
								
					else if ((op[5] == 1) & (funct7b5 == 1) &
									(funct3 == 3'b000))
											alucontrol = 3'b001; // sub => subtract
											
					else if ((funct7b5 == 0) &
									(funct3 == 3'b010))
											alucontrol = 3'b101; // slt => set less than
											
					else if ((funct7b5 == 0) &
									(funct3 == 3'b110))
											alucontrol = 3'b011; // or => or
					
					else if ((funct7b5 == 0) &
									(funct3 == 3'b111))
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
				default:		immsrc = 2'b10; 
			endcase


endmodule