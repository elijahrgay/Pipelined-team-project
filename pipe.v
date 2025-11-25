// Testbench options
`define RUNTIME 380	// How long simulator can run
`define CLKDEL  1	// CLocK transition delay
`define	TRACE	1	// enable simulation trace

// Types
`define WORD	[31:0]	// size of a data word
`define ADDR	[31:0]	// size of a memory address
`define INST	[31:0]	// size of an instruction
`define REG	[4:0]	// size of a register number
`define	REGCNT	[31:0]	// register count
`define	MEMCNT	[511:0] // memory count implemented
`define	OPCODE	[5:0]	// 6-bit opcodes
`define	ALUOP	[3:0]	// Simplified ALU codes

// Fields
`define OP	[31:26]	// opcode field
`define RS	[25:21]	// rs field
`define RT	[20:16]	// rt field
`define RD	[15:11]	// rd field
`define IMM	[15:0]	// immediate/offset field
`define SHAMT	[10:6]	// shift ammount
`define FUNCT	[5:0]	// function code (opcode extension)
`define JADDR	[25:0]	// jump address field
`define	JPACK(R,O,J)		begin R`OP=0; R`JADDR=J; end
`define	IPACK(R,O,S,T,I)	begin R`OP=O; R`RS=S; R`RT=T; R`IMM=I; end
`define	RPACK(R,S,T,D,SH,FU)	begin R`OP=`RTYPE; R`RS=S; R`RT=T; R`RD=D; R`SHAMT=SH; R`FUNCT=FU; end
`define NOP	`OR	// Null OPeration is or $0,$0,$0

// Instruction encoding
`define	RTYPE	6'h00	// OP field for all RTYPE instructions
`define BEQ	6'h04	// OP field
`define	ADDIU	6'h09	// OP field
`define	SLTIU	6'h0b	// OP field
`define	ANDI	6'h0c	// OP field
`define	ORI	6'h0d	// OP field
`define	XORI	6'h0e	// OP field
`define	LUI	6'h0f	// OP field
`define	LW	6'h23	// OP field
`define	SW	6'h2b	// OP field
`define SRLV 6'h06 // FUNCT field for SRLV
`define SRAV 6'h07 // FUNCT field for SRLV
`define	ADDU	6'h21	// FUNCT field
`define	SUBU	6'h23	// FUNCT field
`define	AND	6'h24	// FUNCT field
`define	OR	6'h25	// FUNCT field
`define	XOR	6'h26	// FUNCT field
`define NOR 6'h27 // FUNCT field for NOR
`define	SLTU	6'h2b	// FUNCT field

// Simplified ALU codes, default to lui
`define	ALUAND	4'b0000
`define	ALUOR	4'b0001
`define	ALUADD	4'b0010
`define	ALUSUB	4'b0110
`define	ALUSLT	4'b0111
`define	ALULUI	4'b1000
`define ALUNOR 4'b1001  // Define ALU code for NOR
`define ALUSRLV 4'b1011 // Define ALU code for SRLV
`define ALUSRAV 4'b1100 // Define ALU code for SRAV
`define	ALUXOR	4'b1111

// Generic multi-cycle processor
module processor(halt, reset, clk);
output reg halt;
input reset, clk;
reg `WORD m `MEMCNT;
reg `WORD r `REGCNT;

// Initialize register file and memory
initial begin
    r[1] = 22; r[2] = 1; r[3] = 42;
    r[4] = 601;	r[5] = 11811; r[6] = -1;
    `RPACK(m[0], 3, 4, 3, 0, `SUBU)
    `RPACK(m[1], 3, 2, 1, 0, `SRAV)
    `RPACK(m[0], 2, 3, 1, 0, `ADDU)
    `RPACK(m[1], 2, 3, 1, 0, `SLTU)
    `RPACK(m[2], 3, 5, 1, 0, `AND)
    `RPACK(m[3], 5, 3, 1, 0, `OR)
    `RPACK(m[4], 3, 5, 1, 0, `XOR)
    `RPACK(m[5], 4, 3, 1, 0, `SUBU)
    `IPACK(m[6], `ADDIU, 3, 1, -1)
    `IPACK(m[7], `SLTIU, 5, 1, 12345)
    `IPACK(m[8], `ANDI, 3, 1, 3)
    `IPACK(m[9], `ORI, 3, 1, 3)
    `IPACK(m[10], `XORI, 3, 1, 3)
    `IPACK(m[11], `LUI, 0, 1, 1)
    `IPACK(m[12], `LW, 2, 1, 1023)
    `IPACK(m[13], `SW, 0, 2, 1024)
    `IPACK(m[14], `ADDIU, 6, 6, 1)
    
    m[15] = 0;
    m[256] = 22;
end

// IF registers
reg `ADDR IF_pc;
reg `INST IF_ir;

// ID registers
reg RegDst, Branch, MemRead, MemWrite, ALUSrc, RegWrite, Bad, ID_MemRead, ID_MemWrite, ID_Bad;
reg `ALUOP ALUOp, ID_ALUOp;
reg `WORD s, t, imm, ID_s, ID_t, ID_src;
reg `REG ID_rd;
reg `ADDR target;
reg squash;

// EX registers
reg EX_MemRead, EX_MemWrite, EX_Bad;
reg `WORD alu, EX_alu, EX_t;
reg `REG EX_rd;

// MEM registers
reg MEM_Bad;
reg `WORD v, MEM_v;
reg `REG MEM_rd;

// Running state?
wire running;
assign running = ((!halt) && (!reset));

// Squash instruction fetched on a mispredicted branch
wire `INST squashed;
assign squashed = (squash ? `NOP : IF_ir);

// Are we blocked by a dependence on rs or rt?
// Also handles all forwarding
wire EX_deps, EX_dept, MEM_deps, MEM_dept, WB_deps, WB_dept, canfwds, canfwdt, dep, blocked;
wire `WORD fwds, fwdt;
assign ID_needs = (IF_ir `RS != 0);
assign ID_needt = (IF_ir `RT != 0);
assign EX_deps = (IF_ir `RS == ID_rd);
assign EX_dept = (IF_ir `RT == ID_rd);
assign MEM_deps = (IF_ir `RS == EX_rd);
assign MEM_dept = (IF_ir `RT == EX_rd);
assign WB_deps = (IF_ir `RS == MEM_rd);
assign WB_dept = (IF_ir `RT == MEM_rd);
assign canfwds = ID_needs && ((EX_deps && !MemRead) || ((!EX_deps) && (MEM_deps || WB_deps)));
assign canfwdt = ID_needt && ((EX_dept && !MemRead) || ((!EX_dept) && (MEM_dept || WB_dept)));
assign fwds = (EX_deps ? alu : (MEM_deps ? v : MEM_v));
assign fwdt = (EX_dept ? alu : (MEM_dept ? v : MEM_v));
assign dep = ((ID_needs && (EX_deps || MEM_deps || WB_deps)) ||
	      (ID_needt && (EX_dept || MEM_dept || WB_dept)));
assign blocked = (dep && ((ID_needs && !canfwds) || (ID_needt && !canfwdt)));

// IF: Instruction Fetch stage
always @(posedge clk) if (running && !blocked) begin
  IF_ir <= m[(squash ? target : IF_pc) >> 2];
  IF_pc <= (squash ? target : IF_pc) + 4;
end

// ID: Instruction Decode stage
always @(posedge clk) if (running && !ID_Bad) begin
  if (blocked) begin
    ID_s <= 0;
    ID_t <= 0;
    ID_src <= 0;
    ID_rd <= 0; // not writing
    ID_MemRead <= 0; // not reading
    ID_ALUOp <= 0;
    ID_MemWrite <= 0; // not storing
    ID_Bad <= 0;
  end else begin
    // Brute force decode of instruction
    case (squashed `OP)
      `RTYPE: begin
        case (squashed `FUNCT)
          `ADDU:   begin RegDst=1; Branch=0; MemRead=0; ALUOp=`ALUADD; MemWrite=0; ALUSrc=0; RegWrite=1; Bad=0; end
          `SUBU:   begin RegDst=1; Branch=0; MemRead=0; ALUOp=`ALUSUB; MemWrite=0; ALUSrc=0; RegWrite=1; Bad=0; end
	  `AND:    begin RegDst=1; Branch=0; MemRead=0; ALUOp=`ALUAND; MemWrite=0; ALUSrc=0; RegWrite=1; Bad=0; end
	  `OR:     begin RegDst=1; Branch=0; MemRead=0; ALUOp=`ALUOR;  MemWrite=0; ALUSrc=0; RegWrite=1; Bad=0; end
	  `XOR:    begin RegDst=1; Branch=0; MemRead=0; ALUOp=`ALUXOR; MemWrite=0; ALUSrc=0; RegWrite=1; Bad=0; end
    `NOR:    begin RegDst=1; Branch=0; MemRead=0; ALUOp=`ALUNOR; MemWrite=0; ALUSrc=0; RegWrite=1; Bad=0; end // Decode NOR
	  `SLTU:   begin RegDst=1; Branch=0; MemRead=0; ALUOp=`ALUSLT; MemWrite=0; ALUSrc=0; RegWrite=1; Bad=0; end
    `SRLV:   begin RegDst=1; Branch=0; MemRead=0; ALUOp=`ALUSRLV; MemWrite=0; ALUSrc=0; RegWrite=1; Bad=0; end // Devode SRLV
    `SRAV:   begin RegDst=1; Branch=0; MemRead=0; ALUOp=`ALUSRAV; MemWrite=0; ALUSrc=0; RegWrite=1; Bad=0; end // Devode SRAV
	  default: begin RegDst=0; Branch=0; MemRead=0; ALUOp=`ALUOR;  MemWrite=0; ALUSrc=0; RegWrite=0; Bad=1; end
        endcase
      end
      `BEQ:    begin RegDst=0; Branch=1; MemRead=0; ALUOp=`ALUSUB; MemWrite=0; ALUSrc=0; RegWrite=0; Bad=0; end
      `ADDIU:  begin RegDst=0; Branch=0; MemRead=0; ALUOp=`ALUADD; MemWrite=0; ALUSrc=1; RegWrite=1; Bad=0; end
      `SLTIU:  begin RegDst=0; Branch=0; MemRead=0; ALUOp=`ALUSLT; MemWrite=0; ALUSrc=1; RegWrite=1; Bad=0; end
      `ANDI:   begin RegDst=0; Branch=0; MemRead=0; ALUOp=`ALUAND; MemWrite=0; ALUSrc=1; RegWrite=1; Bad=0; end
      `ORI:    begin RegDst=0; Branch=0; MemRead=0; ALUOp=`ALUOR;  MemWrite=0; ALUSrc=1; RegWrite=1; Bad=0; end
      `XORI:   begin RegDst=0; Branch=0; MemRead=0; ALUOp=`ALUXOR; MemWrite=0; ALUSrc=1; RegWrite=1; Bad=0; end
      `LUI:    begin RegDst=0; Branch=0; MemRead=0; ALUOp=`ALULUI; MemWrite=0; ALUSrc=1; RegWrite=1; Bad=0; end
      `LW:     begin RegDst=0; Branch=0; MemRead=1; ALUOp=`ALUADD; MemWrite=0; ALUSrc=1; RegWrite=1; Bad=0; end
      `SW:     begin RegDst=0; Branch=0; MemRead=0; ALUOp=`ALUADD; MemWrite=1; ALUSrc=1; RegWrite=0; Bad=0; end
      default: begin RegDst=0; Branch=0; MemRead=0; ALUOp=`ALUOR;  MemWrite=0; ALUSrc=0; RegWrite=0; Bad=1; end
    endcase

    s = (canfwds ? fwds : r[squashed `RS]);
    t = (canfwdt ? fwdt : r[squashed `RT]);
    imm = {{16{squashed[15]}}, squashed `IMM};
    target <= IF_pc + {imm[29:0], 2'b00};

    squash <= (Branch && (s == t));
    ID_s <= s;
    ID_t <= t;
    ID_src <= (ALUSrc ? imm : t);
    ID_rd <= (RegWrite ? (RegDst ? squashed `RD : squashed `RT) : 0); // $0 if not writing
    ID_MemRead <= MemRead;
    ID_ALUOp <= ALUOp;
    ID_MemWrite <= MemWrite;
    ID_Bad <= Bad;
  end
end

// EX: EXecute stage
always @(posedge clk) if (running) begin
  case (ID_ALUOp)
    `ALUAND: alu = ID_s & ID_src;
    `ALUOR:  alu = ID_s | ID_src;
    `ALUADD: alu = ID_s + ID_src;
    `ALUSUB: alu = ID_s - ID_src;
    `ALUSLT: alu = ID_s < ID_src;
    `ALUXOR: alu = ID_s ^ ID_src;
    `ALUNOR: alu = ~(ID_s | ID_src); // NOR added as ALU operation
    `ALUSRLV: alu = ID_s >> ID_src;  // SRLV added as ALU operation
    `ALUSRAV: alu = ID_s[31] ? ~(32'hffffffff >> ID_src[4:0]) | (ID_s >> ID_src) : ID_s >> ID_src; // SRAV added as ALU operation
    default: alu = (ID_src << 16);
  endcase

  EX_alu <= alu;
  EX_t <= ID_t;
  EX_rd <= ID_rd;
  EX_MemRead <= ID_MemRead;
  EX_MemWrite <= ID_MemWrite;
  EX_Bad <= ID_Bad;
end

// MEM: data MEMory access stage
always @(posedge clk) if (running) begin
  if (EX_MemRead) v = m[EX_alu >> 2]; else v = EX_alu;
  if (EX_MemWrite) m[EX_alu >> 2] <= EX_t;

  MEM_v <= v;
  MEM_rd <= EX_rd;
  MEM_Bad <= EX_Bad;
end

// WB: register Write Back stage
always @(posedge clk) if (running) begin
  if (MEM_rd) r[MEM_rd] <= MEM_v;
  if (MEM_Bad) halt <= 1;
end

// Reset
always @(posedge clk) if (reset) begin
  halt<=0;
  r[0]<=0;
  squash <= 0; target <= 0;
  IF_ir<=`NOP; IF_pc<=0;
  ID_s<=0; ID_t<=0; ID_src<=0; ID_rd<=0; ID_MemRead<=0; ID_ALUOp<=0; ID_MemWrite<=0; ID_Bad<=0;
  EX_alu<=0; EX_t<=0; EX_rd<=0; EX_MemRead<=0; EX_MemWrite<=0; EX_Bad<=0;
  MEM_v<=0; MEM_rd<=0; MEM_Bad<=0;
end

// State-by-state trace
`ifdef	TRACE
always @(posedge clk) if (running) begin
`define	Fs	(canfwds ? (EX_deps ? "EX_" : (MEM_deps ? "MEM_" : "WB_")) : "")
`define	Ft	(canfwdt ? (EX_dept ? "EX_" : (MEM_dept ? "MEM_" : "WB_")) : "")
  $display("\n    running=%1d blocked=%1d s=%s$%1d t=%s$%1d squash=%1d", running, blocked, `Fs, IF_ir `RS, `Ft, IF_ir `RT, squash);
  $display("IF  ir=%x pc=%1d", IF_ir, IF_pc);
  case (IF_ir `OP)
    `RTYPE: begin
      case (IF_ir `FUNCT)
        `ADDU:   $display("IF  addu $%1d,$%1d,$%1d", IF_ir `RD, IF_ir `RS, IF_ir `RT);
        `SUBU:   $display("IF  subu $%1d,$%1d,$%1d", IF_ir `RD, IF_ir `RS, IF_ir `RT);
	`AND:    $display("IF  and $%1d,$%1d,$%1d", IF_ir `RD, IF_ir `RS, IF_ir `RT);
	`OR:     $display("IF  or $%1d,$%1d,$%1d", IF_ir `RD, IF_ir `RS, IF_ir `RT);
	`XOR:    $display("IF  xor $%1d,$%1d,$%1d", IF_ir `RD, IF_ir `RS, IF_ir `RT);
	`SLTU:   $display("IF  sltu $%1d,$%1d,$%1d", IF_ir `RD, IF_ir `RS, IF_ir `RT);
  `NOR:    $display("IF  nor $%1d,$%1d,$%1d", IF_ir `RD, IF_ir `RS, IF_ir `RT);             // NOR output added to trace
  `SRLV:   $display("IF  srlv $%1d,$%1d,$%1d", IF_ir `RD, IF_ir `RS, IF_ir `RT);            // SRLV output added to trace
  `SRAV:   $display("IF  srav $%1d,$%1d,$%1d", IF_ir `RD, IF_ir `RS, IF_ir `RT);            // SRAV output added to trace
	default: $display("IF  OP=%x RS=%1d RT=%1d RD=%1d SHAMT=%1d FUNCT=%x", IF_ir `OP, IF_ir `RS, IF_ir `RT, IF_ir `RD, IF_ir `SHAMT, IF_ir `FUNCT);
      endcase
    end
    `BEQ:    $display("IF  beq $%1d,$%1d,offset=$%1d", IF_ir `RT, IF_ir `RS, IF_ir `IMM);
    `ADDIU:  $display("IF  addiu $%1d,$%1d,$%1d", IF_ir `RT, IF_ir `RS, IF_ir `IMM);
    `SLTIU:  $display("IF  sltiu $%1d,$%1d,$%1d", IF_ir `RT, IF_ir `RS, IF_ir `IMM);
    `ANDI:   $display("IF  andi $%1d,$%1d,$%1d", IF_ir `RT, IF_ir `RS, IF_ir `IMM);
    `ORI:    $display("IF  ori $%1d,$%1d,$%1d", IF_ir `RT, IF_ir `RS, IF_ir `IMM);
    `XORI:   $display("IF  xori $%1d,$%1d,$%1d", IF_ir `RT, IF_ir `RS, IF_ir `IMM);
    `LUI:    $display("IF  lui $%1d,$%1d", IF_ir `RS, IF_ir `IMM);
    `LW:     $display("IF  lw $%1d,$%1d($%1d)", IF_ir `RT, IF_ir `IMM, IF_ir `RS);
    `SW:     $display("IF  sw $%1d,$%1d($%1d)", IF_ir `RT, IF_ir `IMM, IF_ir `RS);
    default: $display("IF  OP=%x RS=%1d RT=%1d IMM=%x", IF_ir `OP, IF_ir `RS, IF_ir `RT, IF_ir `IMM);
  endcase
  if (ID_Bad) $display("ID  illegal instruction");
    else $display("ID  s=%1d t=%1d src=%1d rd=%1d MemRead=%b ALUOp=%b MemWrite=%b", ID_s, ID_t, ID_src, ID_rd, ID_MemRead, ID_ALUOp, ID_MemWrite);
  if (EX_Bad) $display("EX  illegal instruction");
    else $display("EX  alu=%1d t=%1d rd=%1d MemRead=%b MemWrite=%b", EX_alu, EX_t, EX_rd, EX_MemRead, EX_MemWrite);
  if (MEM_Bad) $display("MEM illegal instruction");
    else begin
      if (EX_MemWrite) $display("MEM m[%1d]=%1d v=%1d rd=%1d", EX_alu, EX_t, MEM_v, MEM_rd);
        else $display("MEM v=%1d rd=%1d", MEM_v, MEM_rd);
    end
  if (MEM_rd) $display("WB  r[%1d]=%1d", MEM_rd, MEM_v);
end
`endif
endmodule

// Testbench
module bench;
wire halt;
reg reset = 1;
reg clk = 0;

processor PE(halt, reset, clk);

initial begin
  #`CLKDEL clk = 1;
  #`CLKDEL clk = 0;
  reset = 0;
  while (($time < `RUNTIME) && !halt) begin
    #`CLKDEL clk = 1;
    #`CLKDEL clk = 0;
  end
end
endmodule
