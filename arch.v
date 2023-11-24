module FetchStage(
  input Clk,
  input Rest,
  output reg [31:0] pc,
  input [31:0] pcr,
  output reg [31:0] pcn,
  input [31:0] Baddi,
  input [31:0] Jaddi,
  input [31:0] Raddi,
  input stall,
  input kill,  
  output  [31:0] Instm,
  input [1:0] pcsori
);
  wire [31:0] Inst;  
  reg [31:0] Badd, Jadd, Radd;
  reg [1:0] pcsor; 
  reg satllo;
  
  always @(posedge Clk) begin
    Badd = Baddi;
    Jadd = Jaddi;
    Radd = Raddi;
    pcsor = pcsori;	
	satllo=	 stall;
  end 
  assign NOP =32'hFFFFFFFF;

  // Instantiate modules
  ST_Mux4x1 m1(pcn, Baddi, Jaddi, Raddi, pcsori, pc); 
  ST_REG PC(pc, stall, Rest, pcr, Clk);
  ST_PLUS4 plus4(pcr, pcn,stall);
  ST_InstMem IM(pcr, Inst,stall);
  ST_Mux2x1 m2(Inst, 32'hFFFFFFFE, kill, Instm);
  
/*always @(Instm) begin
  #4;
  if (kill) begin
    $display("The Fetch Instruction after kill: Instruction Address:%d Instruction Fetch:%b PC+4:%d Stall:%b", pcr, Instm, pcn, satllo);
  end
  else if (pcsor == 0) begin
    $display("\nTHE FETCH INSTRUCTION: Normal sequence\nInstruction Address:%d Instruction Fetch:%b PC+4:%d Stall:%b\n", pcr, Instm, pcn, satllo);
  end
  else if (pcsor == 1) begin
    $display("\nTHE FETCH INSTRUCTION: Branch sequence\nInstruction Address:%d Instruction Fetch:%b PC+4:%d Stall:%b\n", pcr, Instm, pcn, satllo);
  end
  else if (pcsor == 2) begin
    $display("\nTHE FETCH INSTRUCTION: Jump sequence\nInstruction Address:%d Instruction Fetch:%b PC+4:%d Stall:%b\n", pcr, Instm, pcn, satllo);
  end
  else if (pcsor == 3) begin
    $display("\nTHE FETCH INSTRUCTION: Stop sequence\nInstruction Address:%d Instruction Fetch:%b PC+4:%d Stall:%b\n", pcr, Instm, pcn, satllo);
  end
end	*/
always@(posedge Clk) 
begin
	#5
	if (satllo == 1) 
	begin
		$display("\nTHE FETCH INSTRUCTION:Instruction Address:%d Instruction Fetch:%b PC+4:%d Stall:%b\n", pcr, Instm, pcn, satllo);
	end
	else
	begin
		$display("\nTHE FETCH INSTRUCTION: stall cycle");
	end
end
endmodule



 module ST_regFile (
  input [31:0] regfilei [31:0],
  input clk,
  input [4:0] readreg1, writereg,readreg2,	 
  input [31:0] writedata,
  input  RegWrite,
  output reg [31:0] readdata1, readdata2,
  output reg [31:0] regfileo [31:0]
);
  
  integer i;  

  /*initial begin
    for (i=0; i<32; i=i+1) begin
      regfileo[i] = regfilei[i];
    end
  end  */

  always @(posedge clk) begin
    if (RegWrite) begin 
      regfileo[writereg] = writedata;
      regfileo[0] = 0; 
      $display("Write Back Stage: Address=%d, DataIn=%d", writereg, writedata);
    end
  else
	  begin
		 $display("Write Back Stage:  not write in Register File ");  
	  end

    for (i = 0; i < 32; i = i + 1) begin
      if (regfileo[i] === 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx) begin
        regfileo[i] = 0;
      end
    end
	
	/* $display("Register File Contents:");
    for (i = 0; i < 32; i = i + 1) begin
      $display("regfileo[%d] = %d", i, regfileo[i]);
    end		*/
	
  end

  always @(negedge clk) begin		
    if (regfileo[readreg1] === 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx) begin
      readdata1 = 0;
    end
    else begin
      readdata1 = regfileo[readreg1];
    end
    
    if (regfileo[readreg2] === 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx) begin
      readdata2 = 0;
    end
    else begin
      readdata2 = regfileo[readreg2];
    end	 
	  //$display("read Back Stage: Address=%d, DataIn=%d", readreg2, readdata2);
  end

endmodule






module testPiplines;
  reg Clk, Rest, stall, kill,zero,Neg;
  wire [31:0] pcr, pcn, Badd, Jadd, Radd, buffer,DataWriteMemory,DataWriteMemoryOE,DataMemory;
  reg [1:0] pcsor, Alusoro;
  wire [31:0] pcPlus4, Intraction, Imm32, ALUres, Mres, SBi2, SBi1, SA32,PALUres,WRres;
  reg RWo, Rsor, MemRo, MemWo, DWDo, RWB4;
  wire push;
  reg [1:0] spsor;
  reg [1:0] Alusor;
  reg [2:0] op;
  wire [4:0] Rdo1, Rdo2, Rdo3, Rd,Rdm;
  wire Ex_RWo, Mem_RWo, WR_RWo, EX_MemRo,memorywriteo,Datao,Datao2;
  reg [31:0] regfileo [31:0];	 
  reg [31:0] regfilei [31:0];
  FetchStage fetchStage(
    .Clk(Clk),
    .Rest(Rest),
    .pc(pcn), // Update with the correct signal
    .pcr(pcPlus4),
    .Instm(Intraction),
    .pcn(pcn),
    .Baddi(Badd),
    .Jaddi(Jadd),
    .Raddi(Radd),
    .stall(stall),
    .kill(kill),
    .pcsori(pcsor)
  );

 DecodeStage S1 (
 	.regfile(regfileo),
    .Clk(Clk),
    .Ex_RWi(Ex_RWo),
	.DataWriteMemory(DataWriteMemory),
    .Mem_RWi(Mem_RWo),
    .WR_RWi(WR_RWo),
    .EX_MemRi(EX_MemRo),
    .Insti(Intraction),
    .PcPlus4i(pcPlus4),
    .ALUresi(ALUres),
    .Mresi(DataMemory),
    .Rdo1i(Rdo1),
    .Rdo2i(Rdo2),
    .Rdo3i(Rdo3),
    .Badd(Badd),
    .Jadd(Jadd),
    .Radd(Radd),
    .pcsor(pcsor),
    .kill(kill),
    .stall(stall),
    .spsor(spsor),
    .SBi1(SBi1),
    .SBi2(SBi2),
    .SA32(SA32),
    .Alusoro(Alusoro),
    .op(op),
    .Rd(Rd),
    .Imm32(Imm32),
    .RWo(RWo),
    .Rsor(Rsor),
    .MemRo(MemRo),
    .MemWo(MemWo),
    .DWDo(DWDo),
    .RWB4(RWB4),
    .WRresi(WRres),
	.zero(zero)
  );  

   ALU alu_inst(
    .Clk(Clk),
    .Imm32i(Imm32),
    .SA32i(SA32),
    .SBo2i(SBi2),
    .SBo1i(SBi1),
    .Rdi(Rd),
    .opi(op),
    .AlusorBi(Alusoro),
    .zero(zero),
    .Cout(Cout),
    .Neg(Neg),
    .ALUres(ALUres),
    .Rdm(Rdo1),
    .memoryreado(EX_MemRo),
    .memorywriteo(memorywriteo),
    .regwriteo(Ex_RWo),
    .Datao(Datao),
    .memoryreadii(MemRo),
    .memorywriteii(MemWo),
    .regwriteii(RWo),
    .Dataii(DWDo),
	.DataWriteMemoryIE(DataWriteMemory),
	.DataWriteMemoryOE(DataWriteMemoryOE)
  ); 
  //DaINMi'

  Stage_Memory SM (
  .Clk(Clk),
  .MemRB2i(EX_MemRo),
  .MemWB2i(memorywriteo),
  .regWB2ii(Ex_RWo),
  .DWDB2ii(Datao),
  .DaINMi(DataWriteMemoryOE),// what the data Rd,
  .address(ALUres),
  .ALUresii(ALUres),
  .RegWL(Rdo1),
  .DaOuM(DataMemory),
  .ALUreso(PALUres), 
  .RegWLo(Rdo2),
  .regWB2o(Mem_RWo),
  .DWDB2o(Datao2)
); 
	 
	//input [31:0] ResBo, DaOuM, 
WriteBackStage WR (
   .regfilei(regfilei),
  .Clk(Clk),
  .DWDB3i(Datao2),
  .regWB3i(Mem_RWo),
  .ResBo(PALUres),
  .DaOuM(DataMemory),
  .RDi(Rdo2),
  .RDo(Rdo3),
  .regWB3o(WR_RWo),
  .regfileo(regfileo),
  .Mres(WRres)
);	
wire[31:0] s11tt,s222 ;
  reg [7:0] counter;
  initial begin
    // Initialize inputs
    Rest = 0;
    Clk = 0; 
	 counter = 0;
  end 

  
  always @(posedge Clk) begin	
    $display("\n___ Clock Cycle %0d ___\n", counter);	 
	 counter = counter + 1;
  end
  initial begin
    repeat (50) begin
      // $display("\n____ Clock Cycle %0d _____\n", $time);
      #5 Clk = ~Clk;
      #5; // Wait for some time before the next clock cycle	 
    end
  end 
  
integer i;

always @(posedge Clk) begin
  //$display("Register File Contents:");
 /*for (i = 0; i < 32; i = i + 1) begin
    $display("regfileo[%d] = %d", i, regfileo[i]);
  end  */

  for (i = 0; i < 32; i = i + 1) begin
    regfilei[i] = regfileo[i];
  end
end

  
endmodule


// Rest of the code...


  //ST_REG PC(pc, stall, Rest, pcr, Clk);
module ST_REG(IN, EN, Rest, OUT, Clk);
  input EN, Clk, Rest;
  input [31:0] IN; 
  
  initial begin	  
    OUT = 0; 
  end
  
  output reg [31:0] OUT;  
  
  always @(posedge Clk) begin
    if (Rest == 1) begin
      OUT = 0;
    end else if (EN == 0) begin
      OUT = IN;
    end else begin	
      OUT = IN;
    end  
  end
endmodule

module ST_PLUS4 (
  input [31:0] in,
  output reg [31:0] out,
  input stall
);
  
  always @(in or stall) begin
    if (stall) begin
      out = in + 4; // Increment by 4 when stall is 1
    end else begin
      out = in; // No change if stall is 0
    end
  end
  
endmodule



module ST_Mux4x1(in1, in2, in3, in4, sel, out);
  input [31:0] in1, in2, in3, in4;
  input [1:0] sel;
  output reg [31:0] out;
  
  always @ (in1, in2, in3, in4, sel) begin
    if (sel === 2'bx) // Check for unknown sel (x)
      out = in1; 
    else if (sel == 2'b00)
      out = in1;
    else if (sel == 2'b01)
      out = in2;
    else if (sel == 2'b10)
      out = in3;
    else if (sel == 2'b11)
      out = in4;
	else out = in1;  	   
	//	$display("=%b =%b =%b =%b =%b  ",in1,in2,in3,in4,out,);
  //   $display("Mux4x1 : %b", in1);
  end
endmodule


module ST_InstMem(
  input [31:0] address,
  output reg [31:0] Inst,input stall
);
reg [31:0] Mem [0:9] = '{
32'b00000000000000000000000000000000,
32'b00010000000001000000000000001100,  
32'b00001000000000000000000001100010, 
32'b00010000000001100000000000001100, 
32'b00001000010001000011000010100000,
32'b00000111111111111111111111100010, 
32'b00001000010000100000000010100100,
32'b00011000010000100010000000000000,
32'b00100000010001011111111110100100,
32'b00010000010000100010000000000001
                          };


  always @(*) begin	 
	  if (stall == 0) begin
        Inst = Mem[(address - 4) >> 2];
    //    $display("Fetch instruction: Current address: %b, Instruction: %h \n\n", address, Inst);
      end 
    else if (address[31] == 0) begin
      Inst = Mem[address >> 2];
   //   $display("Fetch instruction: Current address: %b, Instruction: %h \n\n", address, Inst);
    end 
  else begin
      Inst= 32'b00000000000000000000000000000000 ;
   //   $display("FETCH INSTRACTION: Current address: %b, Instruction: %b", address, Inst);
    end
  end
endmodule

module ST_Mux2x1(in1, in2, sel, out);
  input [31:0] in1, in2;
  input sel;
  output reg [31:0] out;

  initial begin
    #1;
    out <= in1;
  end

  always @(in1, in2, sel) begin
    case (sel)
      1'b0: out <= in1;
      1'b1: out <= in2;
      default: out <= in1;  // If sel is unknown ('x'), choose in1
    endcase
    //
	//$display("Instruction = %b", in1);
  end
endmodule

//////////////////////////////////////////////////////////////////// first step fetch ^_^  


module DecodeStage(
  input [31:0] regfile [31:0],
  input Clk,Ex_RWi,Mem_RWi,WR_RWi,EX_MemRi,zero,
  input [31:0] Insti,
  input	[31:0] PcPlus4i,ALUresi,Mresi,WRresi,
  input [4:0]Rdo1i,Rdo2i,Rdo3i,  
  output [31:0]Badd,Jadd,Radd,
  output reg [31:0]DataWriteMemory,
  output reg [1:0] pcsor,
  output reg kill,stall,
  output reg [1:0] spsor,
  output [31:0] SBi1,SBi2,
  output  [31:0] SA32,
  output [1:0] Alusoro,
  output [2:0] op,
  output [4:0] Rd,
  output [31:0]  Imm32,
  output RWo,Rsor,MemRo,MemWo,DWDo,RWB4
);	
reg [31:0] Insto;
reg Ex_RWo,Mem_RWo,WR_RWo,EX_MemRo;
reg	[31:0] PcPlus4,ALUres,Mres,WRres; 
reg [4:0]Rdo1,Rdo2,Rdo3;
  always @(posedge Clk) begin
    Insto = Insti;
    Ex_RWo= Ex_RWi;
	Mem_RWo= Mem_RWi;
	WR_RWo=WR_RWi	 ;
	EX_MemRo=EX_MemRi;
	PcPlus4=PcPlus4i;
	ALUres=ALUresi;
	WRres=WRresi;
	Rdo1=Rdo1i;
	Rdo2=Rdo2i;
	Rdo3=Rdo3i;	   
  end
//ST_Mux2x1Stall m7(RW,Alusor,MemR,MemW,DWD,stall,RWo,Alusoro,MemRo,MemWo,DWDo);
    wire [4:0] func,Rs1,Rs2,sa;
	wire [1:0] Type;
	wire [13:0] Imm;
	wire [23:0] JImm;
	wire stop,EXT1,push;
	wire [1:0] Alusor;
 	wire RW,MemR,DWD,MemW;
	assign func = Insto[31:27];
	assign Rs1 = Insto[26:22];
	assign Rd = Insto[21:17];
	assign Rs2 = Insto[16:12];
	assign sa = Insto[11:7];
	assign Type = Insto[2:1];
	assign Imm = Insto[16:3];
	assign JImm = Insto[26:3];
	assign stop = Insto[0];
	/// control signils   
	wire pop;
 	ST_MainControl MC(Type,func,RW,Rsor,Alusor,MemR,MemW,DWD,EXT1);
	ST_AluControl AC(Type,func,op);
	ST_SPControl SC(Type,func,stop,spsor,push,pop);	  // in this spsor,push is output.
	ST_PCControl PRC(Type,func,stop,zero,pcsor,kill);	
	 integer i; 
 /* always @(posedge Clk) begin
  $display("Register File Contents:");
  for (i = 0; i < 32; i = i + 1) begin
    $display("regfileo[%d] = %d", i, regfile[i]);
  end 	*/
 // end
always @(posedge Clk) begin	
	#6	
    $display("DECODE STAGE :Insto=%b,func=%b ,Rs1=%b,Rd = %b,Rs2 =%b,sa =%b,Type = %b,Imm = %b,JImm = %b,stop = %b",Insto, func, Rs1, Rd, Rs2, sa, Type, Imm, JImm, stop);
end	
	wire [31:0] regfileo [31:0];
	wire [4:0] S2;	  
	ST_Mux2x1x5bit m3(Rs2,Rd,Rsor,S2); 
	reg [31:0]	So1,So2,So3;
	always@(negedge Clk) 
		begin
			#3
	So2= regfile[S2];
	So1= regfile[Rs1] ;	
	DataWriteMemory=regfile[Rd] ;
	//$display("sou1=%b.sour=%b,DataWriteMemory=%b",	So2,Mresi,DataWriteMemory);
	end
	//ST_regFile RF(regfile,Clk,Rs1,0,S2,0,0,So1,So2,regfileo); 
	//ST_regFile tee(regfile,Clk, Rs1, 0, Rd,  0, 0, So3,DataWriteMemory,regfileo);

	wire [31:0] JImm32;
    ST_EX1 E1(Imm32,Imm,EXT1);
	ST_EX2 E2(SA32,sa);
	ST_EX3 E3(JImm32,JImm);
		
							   
	ST_ADDER AD1(Imm32,PcPlus4i,Badd);
	ST_ADDER AD2(JImm32,PcPlus4i,Jadd);									   
	///////////////////////////////////////////////////////
	
	wire [31:0] spin,spout,spp1,spm1; 
	wire [1:0]Forward1,Forward2;
	ST_Stack ST(spout,PcPlus4i,push,pop,Clk,Radd);
	ST_Forwarding FR(Type,Rs1,S2,Rdo1i,Rdo2i,Rdo3i,Ex_RWi,Mem_RWi,WR_RWi,Forward1,Forward2);
	ST_DATAHAZERD DH(EX_MemRi,Forward1,Forward2,stall);
	
	//ST_Mux4x1(in1, in2, in3, in4, sel, out);
	ST_Mux4x1 m5(So1,ALUresi,Mresi,WRresi,Forward1,SBi1);
	ST_Mux4x1 m6(So2,ALUresi,Mresi,WRresi,Forward2,SBi2);  
	
	ST_Mux2x1Stall m7(RW,Alusor,MemR,MemW,DWD,stall,RWo,Alusoro,MemRo,MemWo,DWDo);	
	
endmodule

module ST_EX1(out,in,ExtendSign);

    
     output reg [31:0] out;
    
     input  [13:0] in;
	 input ExtendSign;
 
    always@(in,ExtendSign)
	 begin
		if (ExtendSign == 0) begin
			out <= {16'h0000,2'b00,in};
		end else begin
			if (in[13]==1)begin
			  out <= {16'hffff,2'b11 , in};
			end
			else begin
			  out <= {16'h0000,2'b00 , in};
			end
		end

	 end
endmodule









module ST_EX2(out,in);

    
    output reg [31:0] out;
    input  [4:0] in;
 
    always@(in)
	 begin
		 out <= {24'h000000,3'b000, in};
		 
	 end
	 
endmodule






module ST_EX3(out,in);

    
     output reg [31:0] out;
    
     input  [23:0] in;
 
    always@(in)
	begin
		
		
		if (in[23]==1)begin
		   out <= {8'hff , in};
		end
		else begin
			out <= {8'h00 , in};
			end

	 end
endmodule
	

module ST_ADDER(in1,in2,out);
	
	input signed [31:0] in1,in2;
	
	output signed [31:0] out;
	always@(*)
		begin
			//s$display("%d   %d       ccccccccccc%d ",in1,in2,out);
		end
	assign out = in1+in2;

endmodule



module ST_Forwarding(
  input  [1:0] Type,
  input [4:0] Rs1, Rs2, Rd1, Rd2, Rd3,
  input Ex_RW, Mem_RW, WR_RW,
  output reg [1:0] Forward1, Forward2
);
always @* begin
    #6
    //$display("Rs1=%d, Rs2=%d, Rd1=%d, Rd2=%d, Rd3=%d, Ex_RW=%d, Mem_RW=%d, WR_RW=%d", Rs1, Rs2, Rd1, Rd2, Rd3, Ex_RW, Mem_RW, WR_RW);
    

    if (Rs1 == Rd1 && Ex_RW == 1) begin
      Forward1 = 1;
    end
    else if (Rs1 == Rd2 && Mem_RW == 1) begin
      Forward1 = 2;
    end
    else if (Rs1 == Rd3 && WR_RW == 1) begin
      Forward1 = 0;
    end
    else if (Rd1 == 5'bxxxxx || Rd2 == 5'bxxxxx || Rd3 == 5'bxxxxx) begin
      Forward1 = 0; // Set Forward1 to 0 if any Rd value is 'x'
    end
    else begin
      Forward1 = 0;
    end

   if (Type == 2'b00 || Type == 2'b11) begin
	  if (Rs2 == 0) begin
      Forward2 = 0;
    end
    else if (Rs2 == Rd1 && Ex_RW == 1) begin
      Forward2 = 1;
    end
    else if (Rs2 == Rd2 && Mem_RW == 1) begin
      Forward2 = 2;
    end
    else if (Rs2 == Rd3 && WR_RW == 1) begin
      Forward2 = 0;
    end
    else if (Rd1 == 5'bxxxxx || Rd2 == 5'bxxxxx || Rd3 == 5'bxxxxx) begin
      Forward2 = 0; // Set Forward2 to 0 if any Rd value is 'x'
    end	 
	end
    else begin
      Forward2 = 0;
    end
    
  $display("+---------------------------------+");
    $display("\n| Forward1: %b   Forward2: %b |\n", Forward1, Forward2);
    $display("+---------------------------------+");	

  end
endmodule



   
 module ST_Mux2x1Stall(RWi,Alusori,MemRi,MemWi,DWDi,sel,RWo,Alusoro,MemRo,MemWo,DWDo);
	
	input RWi,MemRi,MemWi,DWDi;
	input [1:0] Alusori;
	input  sel;
	output reg RWo,MemRo,MemWo,DWDo;
	output reg [1:0] Alusoro;

	always @(RWi,Alusori,MemRi,MemWi,DWDi,sel)
	begin
	//	$display("The Select equla =%d",sel);
		case(sel)
			1'b0: begin
				RWo = 0;
				Alusoro = 0;
				MemRo = 0;
				MemWo = 0;
				DWDo = 0;
			end
			default 	
			    begin
				RWo = RWi;
				Alusoro = Alusori;
				MemRo = MemRi;
				MemWo = MemWi;
				DWDo = DWDi;
			end
			
		endcase
		
	end
	
endmodule









module ST_DATAHAZERD(EX_MemR, For1, For2, Stall);
    input EX_MemR;
    input [1:0] For1, For2;
    output reg Stall;
    
    always @(EX_MemR, For1, For2) begin	 
		#10
		//$display("EX_MemR=%d , For1=%d, For2=%d",EX_MemR, For1, For2);
        if (EX_MemR == 1 && (For1 == 1 || For2 == 1)) begin
            Stall = 0;
           $display("There is a hazard - Stall!");
        end
        else begin
            Stall = 1;
        end
    end
endmodule
module ST_Stack(
  input [31:0] address,
  input [31:0] DataIn,
  input push,
  input pop,
  input Clk,
  output reg [31:0] DataOut
);
  reg [31:0] Mem [255:0];
  reg [31:0] stack_address;

  initial begin
    stack_address = 0; // Initialize stack_address to zero
  end

  always @(push,pop) begin 
    if (push) begin
      Mem[stack_address] = DataIn;
      stack_address = stack_address + 4; // Increment stack_address by 4 on push 
	  
    end
    else if (pop) begin
      stack_address = stack_address - 4; // Decrement stack_address by 4 on pop
    end
    DataOut = Mem[stack_address];
	//$display(" Hi there pop and push=%d =%d",stack_address,DataOut);
  end
endmodule

module ST_MainControl(Type,func,RW,Rsor,Alusor,MemR,MemW,DWD,EXT1);
	
	input [1:0] Type;
	input [4:0] func;
	output reg RW,Rsor,MemR,MemW,DWD,EXT1;
	output reg [1:0] Alusor;
	
	reg [6:0] TAF;
	
	
	assign TAF = {func,Type};
	
	
	always @(*)
	begin
		
		casex(TAF)
			
			7'b0000000: begin
				
				RW = 1'b1;
				Rsor = 1'b0;
				Alusor = 2'b10;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'bx;
			end
			
			7'b0000100: begin
				
				RW = 1'b1;
				Rsor = 1'b0;
				Alusor = 2'b10;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'bx;
			end
			
				7'b0001000: begin
				
				RW = 1'b1;
				Rsor = 1'b0;
				Alusor = 2'b10;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'bx;
			end
			
			
				7'b0001100: begin
				
				RW = 1'b0;
				Rsor = 1'b0;
				Alusor = 2'b10;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'bx;
			end
			
				7'b0000010: begin
				
				RW = 1'b1;
				Rsor = 1'bx;
				Alusor = 2'b00;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'b0;
			end
			
				7'b0000110: begin
				
				RW = 1'b1;
				Rsor = 1'bx;
				Alusor = 2'b00;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'b1;
			end
			
				7'b0001010: begin
				
				RW = 1'b1;
				Rsor = 1'bx;
				Alusor = 2'b00;
				MemR = 1'b1;
				MemW = 1'b0;
				DWD = 1'b1;
				EXT1 = 1'b1;
			end
			
				7'b0001110: begin
				
				RW = 1'b0;
				Rsor = 1'b1;
				Alusor = 2'b00;
				MemR = 1'b0;
				MemW = 1'b1;
				DWD = 1'bx;
				EXT1 = 1'b1;
			end
			
				7'b0010010: begin
				
				RW = 1'b0;
				Rsor = 1'b1;
				Alusor = 2'b10;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'bx;
				EXT1 = 1'b1;
			end
			
				7'b0000001: begin
				
				RW = 1'b0;
				Rsor = 1'bx;
				Alusor = 2'bxx;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'bx;
				EXT1 = 1'bx;
			end
			
			  	7'b0000101: begin
				
				RW = 1'b0;
				Rsor = 1'bx;
				Alusor = 2'bxx;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'bx;
				EXT1 = 1'bx;
			end
			
				7'b0000011: begin
				
				RW = 1'b1;
				Rsor = 1'bx;
				Alusor = 2'b01;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'bx;
			end
			
				7'b0000111: begin
				
				RW = 1'b1;
				Rsor = 1'bx;
				Alusor = 2'b01;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'bx;
			end
			
				7'b0001011: begin
				
				RW = 1'b1;
				Rsor = 1'b0;
				Alusor = 2'b10;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'bx;
			end
			
				7'b0001111: begin
				
				RW = 1'b1;
				Rsor = 1'b0;
				Alusor = 2'b10;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'bx;
			end
			
			default: begin
				RW = 1'b0;
				Rsor = 1'b0;
				Alusor = 2'b00;
				MemR = 1'b0;
				MemW = 1'b0;
				DWD = 1'b0;
				EXT1 = 1'b0;
				
			end
			
			
		endcase
		
	end
		
endmodule

module ST_AluControl(Type,func,op);
	input [1:0] Type;
	input [4:0] func;
	output reg [2:0] op;
	
	reg [6:0] TAF;
	
	assign TAF = {func,Type};
	
	
	always @(*)
	begin
		
		case(TAF)
			7'b0000100,7'b0000110,7'b0001010,7'b0001110:
			op = 0;
			
			7'b0001000,7'b0001100:
			op = 1;
			
			7'b0000000,7'b0000010:
			op = 2;
			
			7'b0000011,7'b0001011:
			op = 5;
			
			7'b0000111,7'b0001111:
			op = 6;
		endcase
	end
endmodule	 
module ST_SPControl(Type,func,Stop,spsor,push,pop);
	input [4:0] func;
	input [1:0] Type;
	input Stop;
	output reg [1:0] spsor;
	output reg push,pop;
	
	
	always @(*)
	begin
		if(func == 5'b00001 && Type == 2'b01) begin
			spsor = 2'b00;
			push = 1'b1;
			pop = 1'b0;
		end
		else if(Stop == 1'b1) begin
			spsor = 2'b01;
			pop = 1'b1;
			push = 1'b0;
		end
		else begin
			spsor = 2'b10;
			push = 1'b0;
			pop = 1'b0;
		end	
		//$display("pop=%d ,push=%d",pop,push);
	
	end
	
endmodule
module ST_PCControl(Type,func,Stop,zero,pcsor,Kill);
	input [4:0] func;
	input [1:0] Type;
	input Stop,zero;
	output reg [1:0] pcsor;
	output reg Kill;
	always @(*)	 
	begin 
		#4
	//	$display("zero=%d",zero);
		if(Type == 2'b01) begin
			pcsor <= 2'b10;
			Kill <= 1;
		end
		else if(func == 5'b00100 && Type == 2'b10 && zero == 1) begin
			pcsor <= 2'b01;
			Kill <= 1;
		end
		else if(Stop == 1) begin
			pcsor <= 2'b11;
			Kill <= 1;
		end
		else begin
			pcsor <= 2'b00;
			Kill <= 0;
		end	
		//$display("zero=%d,pcsor=%d,Kill=%d",zero,pcsor,Kill);
	end
endmodule	
module ST_Mux2x1x5bit(in1,in2,sel,out);
	
	input [4:0] in1,in2;
	input  sel;
	output reg [4:0] out;

	always @(in1,in2,sel)
	begin
		
		case(sel)
			
			1'b0: out <= in1;
			1'b1: out <= in2;
			default   out <= in1;
		endcase
		
	end
	
endmodule  

///////////////////////////////////////////////////////////
module ALU(
  input Clk,
  input [31:0] Imm32i, SA32i, SBo2i, SBo1i, DataWriteMemoryIE,
  input [2:0] opi,
  input [1:0] AlusorBi,
  output reg zero, Cout, Neg,
  output reg [31:0] ALUres, DataWriteMemoryOE,
  input [4:0] Rdi,
  output reg [4:0] Rdm,
  output reg memoryreado, memorywriteo, regwriteo, Datao,
  input memoryreadii, memorywriteii, regwriteii, Dataii
);

  reg [31:0] Imm32o, SA32o, SBo2, SBo1;
  reg [2:0] opo;
  reg [1:0] AlusorBo;
  reg [4:0] Rd;
  reg memoryreadi, memorywritei, regwritei, Datai;
  reg [31:0] val2;
  reg aa_completed;

  always @(posedge Clk) begin
    regwritei = regwriteii;
    Datai = Dataii;
    memorywritei = memorywriteii;
    memoryreadi = memoryreadii;
    DataWriteMemoryOE = DataWriteMemoryIE;
    Rd = Rdi;
    Imm32o = Imm32i;
    SA32o = SA32i;
    SBo2 = SBo2i;
    SBo1 = SBo1i;
    opo = opi;
    AlusorBo = AlusorBi;
    Rdm = Rd;
    memoryreado = memoryreadi;
    memorywriteo = memorywritei;
    regwriteo = regwritei;
    Datao = Datai; 
  end
  
  always @(posedge Clk) begin  
	#2
    $display("\nEXUSION STAGE : OPcode=%b, DATA1=%b, DATA2=%b, RESULT=%b,Zero=%b,regwritei=%b\n", opo, SBo1, val2, ALUres,zero,regwritei);
  end 
  
  always @(Imm32o or SA32o or SBo2 or AlusorBo) begin
    case (AlusorBo)
      2'b00: val2 = Imm32o;
      2'b01: val2 = SA32o;
      2'b10: val2 = SBo2;
      default: val2 = Imm32o;
    endcase	 
   //	$display("tttttttttttttttttttttttttttttttttttttttttttttttt %b ,%b ,%b,%b %b ",	Imm32i, SA32i, SBo2i, SBo1i,AlusorBo);
  end

  ST_ALU aa(opo, SBo1, val2, ALUres, zero, Cout, Neg, aa_completed);	 
endmodule


module ST_ALU(opi, SBo1, SBo2, ALUres, zero, Cout, Neg, aa_completed);
  input [2:0] opi;
  input [31:0] SBo1, SBo2;
  output reg [31:0] ALUres;
  output reg zero, Cout, Neg;
  output reg aa_completed;
  
  always @(opi or SBo1 or SBo2) begin
    case (opi)
      3'b000: ALUres = SBo1 + SBo2;
      3'b001: ALUres = SBo1 - SBo2;
      3'b010: ALUres = SBo1 & SBo2;
      3'b011: ALUres = SBo1 | SBo2;
      3'b100: ALUres = SBo1 ^ SBo2;
      3'b101: ALUres = SBo1 << SBo2;
      3'b110: ALUres = SBo1 >> SBo2;
      default: ALUres = SBo1;
    endcase
    
    if (ALUres == 0) begin
      zero = 1;
      Cout = 0;
      Neg = 0;
    end
    else if (ALUres[31] == 1) begin
      zero = 0;
      Cout = 0;
      Neg = 1;
    end
    else begin
      zero = 0;
      Cout = 1;
      Neg = 0;
    end
    
    aa_completed = 1;  
	// $display("\nEXUSION STAGE : OPcode=%b, DATA1=%b, DATA2=%b, RESULT=%b,Zero=%b\n", opi, SBo1, SBo2, ALUres,zero);
  end
endmodule


module ST_Mux3x1(in1, in2, in3, sel, out);
  input [31:0] in1, in2, in3;
  input [1:0] sel;
  output reg [31:0] out;

  always @(in1 or in2 or in3 or sel) begin
    case (sel)
      2'b00: out = in1;
      2'b01: out = in2;
      2'b10: out = in3;
      default: out = in1;
    endcase
  end
endmodule


//////////////////////////////////////////////////////////// 

module Stage_Memory(
  input Clk, MemRB2i, MemWB2i, regWB2ii, DWDB2ii,
  input [31:0] address, DaINMi, ALUresii, 
  input [4:0] RegWL,
  output [31:0] DaOuM,
  output reg [31:0] ALUreso,
  output reg  [4:0]RegWLo,
  output reg regWB2o, DWDB2o
);
  reg MemRB2o, MemWB2o, regWB2i, DWDB2i;
  reg [31:0] ResBo, DaINM,  ALUresi;
  reg[4:0] 	RDi;

  always @(posedge Clk) begin
    MemRB2o = MemRB2i;
    MemWB2o = MemWB2i;
    regWB2i = regWB2ii;
    DWDB2i = DWDB2ii;
    DaINM = DaINMi;
    RDi = RegWL;
    ALUreso =ALUresii;
    RegWLo = RDi;
    regWB2o = regWB2i;
	DWDB2o=DWDB2i;
end


  reg [31:0] DataOut;

  ST_DataMem DM(
    .address(address),
    .DataIn(DaINM),
    .R(MemRB2i),
    .W(MemWB2i),
    .DataOut(DaOuM),
    .Clk(Clk)
  );

endmodule



module ST_DataMem(
  input [31:0] address, DataIn,
  input R, W, Clk,
  output reg [31:0] DataOut
);
  reg [31:0] Mem [1023:0];
  integer i;
  
  initial begin
    for (i = 0; i < 1024; i = i + 1) begin
      Mem[i] = 32'd20;// Set initial value to all zeros
    end
  end
   
  always @(posedge Clk) begin	
    if (W == 1) begin
      Mem[address] = DataIn; // Use non-blocking assignment for synchronous write 	 
      $display("MEMORY STAGE: Write In Memory Address=%b, DataIn=%b\n ", address, DataIn);
    end
    else if (R == 1) begin
      DataOut = Mem[address]; // Use non-blocking assignment for synchronous read
     $display("MEMORY STAGE: Read From Memory, Address=%b, DataOut=%b\n", address, DataOut);
    end
   else begin 
	   DataOut=address;
      $display("MEMORY STAGE: NOT Write or Read From Memory\n");
    end	 
  end
endmodule



module WriteBackStage2 (
  input [31:0] regfilei [31:0],
  input Clk, DWDB3i, regWB3i,
  input [31:0] ResBo, DaOuM, 
  input [4:0] RDi,
  output reg [4:0] RDo,
  output reg regWB3o,
  output reg [31:0] Mres,
  output [31:0] regfileo [31:0]
);

  wire [31:0] s1, s2;
  ST_Mux2x1 m9(ResBo, DaOuM, DWDB3i, s1);
  ST_regFile mq(regfilei,Clk, 0, RDi, 0, s1, regWB3o, s1, s2,regfileo);

  always @(posedge Clk) begin
    RDo <= RDi;
    regWB3o <= regWB3i;
    Mres <= s1;	
	$display(" ooooooooutttttt = %d",s1);
  end

endmodule 

module WriteBackStage (
	 input [31:0] regfilei [31:0],
  input Clk, DWDB3i, regWB3i, input [31:0] ResBo, DaOuM, 
  input [4:0] RDi, output reg [31:0] Mres,
  output reg [4:0] RDo, output reg regWB3o, output [31:0] regfileo [31:0]
);

  wire [31:0] s1, s2;
  //module ST_regFile (Rs1, Rd, Rs2, Rw, DataIn, S1, S2, Clk);
  ST_Mux2x1 m9(ResBo, DaOuM, DWDB3i, Mres);
  ST_regFile mq(regfilei, Clk,0, RDi, 0,Mres, regWB3i, s1, s2,regfileo);

  always @(posedge Clk) begin
    RDo = RDi;
    regWB3o = regWB3i;
//	$display("RDi=%d",ResBo);
  end
endmodule