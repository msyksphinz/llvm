//===- MYRISCVXInstrInfo.td - Target Description for MYRISCVX Target -*- tablegen -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Instructions specific format
//===----------------------------------------------------------------------===//

// Compressed Instruciton CR (Compressed Register)
class MYRISCVX_CR<bits<2> op, bits<4> funct4, bits<2> op,
                  dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin> :
  MYRISCVXCInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5> rd;
  bits<5> rs2;

  let Inst{15-12} = funct4;
  let Inst{11-7}  = rd;
  let Inst{6-2}   = rs2;
}


// Compressed Instruciton CI (Compressed Immediate)
class MYRISCVX_CI<bits<2> op, bits<3> funct3, bits<2> op,
                  dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin> :
  MYRISCVXCInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5> rd;
  bits<6> imm;

  let Inst{15-13} = funct3;
  let Inst{12}    = imm{5};
  let Inst{11-7}  = rd;
  let Inst{6-2}   = imm{4-0};
}


// Compressed Instruciton CSS (Compressed Stack-relative Store)
class MYRISCVX_CSS<bits<2> op, bits<3> funct3,
                  dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin> :
  MYRISCVXCInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<6> imm;
  bits<5> rs2;

  let Inst{15-13} = funct3;
  let Inst{12-7}  = imm;
  let Inst{6-2}   = rs2;
}


// Compressed Instruciton CIW (Compressed Wide Immediate)
class MYRISCVX_CIW<bits<2> op, bits<3> funct3,
                  dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin> :
  MYRISCVXCInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<8> imm;
  bits<3> rddash;

  let Inst{15-13} = funct3;
  let Inst{12-5}  = imm;
  let Inst{4-2}   = rddash;
}


// Compressed Instruciton CL (Compressed Load)
class MYRISCVX_CL<bits<2> op, bits<3> funct3,
                  dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin> :
  MYRISCVXCInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5> imm;
  bits<3> rs1dash;
  bits<3> rddash;

  let Inst{15-13} = funct3;
  let Inst{12-10} = imm{4-2};
  let Inst{9-7}   = rs1dash;
  let Inst{6-5}   = imm{1-0};
  let Inst{4-2}   = rddash;
}


// Compressed Instruciton CS (Compressed Store)
class MYRISCVX_CS<bits<2> op, bits<3> funct3,
                  dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin> :
  MYRISCVXCInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5> imm;
  bits<3> rs1dash;
  bits<3> rs2dash;

  let Inst{15-13} = funct3;
  let Inst{12-10} = imm{4-2};
  let Inst{9-7}   = rs1dash;
  let Inst{6-5}   = imm{1-0};
  let Inst{4-2}   = rs2dash;
}


// Compressed Instruciton CA (Compressed Arithmetic)
class MYRISCVX_CA<bits<2> op, bits<6> funct6, bits<2> funct2,
                  dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin> :
  MYRISCVXCInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<3> rddash;
  bits<3> rs2dash;

  let Inst{15-10} = funct6;
  let Inst{9-7}   = rddash;
  let Inst{6-5}   = funct2;
  let Inst{4-2}   = rs2dash;
}


// Compressed Instruciton CB (Compressed Branch)
class MYRISCVX_CB<bits<2> op, bits<3> funct3,
                  dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin> :
  MYRISCVXCInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<8> offset;
  bits<3> rs1dash;

  let Inst{15-13} = funct3;
  let Inst{12-10} = offset{7-5};
  let Inst{9-7}   = rs1dash;
  let Inst{6-2}   = offset{4-0};
}


// Compressed Instruciton CJ (Compressed Jump)
class MYRISCVX_CJ<bits<2> op, bits<3> funct3,
                  dag outs, dag ins, string asmstr,
                  list<dag> pattern, InstrItinClass itin> :
  MYRISCVXCInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<11> target;

  let Inst{15-13} = funct3;
  let Inst{12-2}  = target;
}


//@ 32-bit load.
let canFoldAsLoad = 1 in
class CLoad<bits<2> op, bits<3> funct3, string instr_asm, PatFrag OpNode, RegisterClass RC,
            bit Pseudo>:
  MYRISCVX_CL<opcode, funct3, (outs RC:$rd), (ins GPR:$rs1, simm12:$simm12),
     !strconcat(instr_asm, "\t$rd, ${simm12}(${rs1})"),
     [], IILoad> {
  let isPseudo = Pseudo;
}


def C_ADDI4SPN : MYRISCVX_CIW<0b00, 0b000>

def C_LW       : CLoad<0b00, 0b010, "c.lw",  load_a>
def C_LD       : CLoad<0b00, 0b011, "c.ld",  load_a> // (RV64/128)
def C_FLD      : CLoad<0b00, 0b001, "c.fld", load_a> // (RV32/64)
def C_FLW      : CLoad<0b00, 0b011, "c.fld", load_a> // (RV32)

def C_FSD      : MYRISCVX_CS<0b00, 0b101> // (RV32/64)
def C_FSW      : MYRISCVX_CS<0b00, 0b111> // (RV32)
def C_SW       : MYRISCVX_CS<0b00, 0b110>
def C_SD       : MYRISCVX_CS<0b00, 0b111> // (RV64/128)

def C_NOP      : MYRISCVX_CI<0b01, 0b000>  // (HINT, nzimm̸=0)
def C_ADDI     : MYRISCVX_CI<0b01, 0b000>  // (HINT, nzimm=0)
def C_JAL      : MYRISCVX_CJ<0b01, 0b001>  // (RV32)
def C_ADDIW    : MYRISCVX_CI<0b01, 0b001>  // (RV64/128; RES, rd=0)
def C_LI       : MYRISCVX_CI<0b01, 0b010>  // (HINT, rd=0)
def C_ADDI16SP : MYRISCVX_CI<0b01, 0b011>  // (RES, nzimm=0)
def C_LUI      : MYRISCVX_CI<0b01, 0b011>  // (RES, nzimm=0; HINT, rd=0)
def C_SRLI     : MYRISCVX_CB<0b01, 0b100>  // (RV32 NSE, nzuimm[5]=1)
def C_SRAI     : MYRISCVX_CB<0b01, 0b100>  // (RV32 NSE, nzuimm[5]=1)
def C_ANDI     : MYRISCVX_CB<0b01, 0b100>
def C_SUB      : MYRISCVX_CA<0b01, 0b100>
def C_XOR      : MYRISCVX_CA<0b01, 0b100>
def C_OR       : MYRISCVX_CA<0b01, 0b100>
def C_AND      : MYRISCVX_CA<0b01, 0b100>
def C_SUBW     : MYRISCVX_CA<0b01, 0b100>  // (RV64/128; RV32 RES)
def C_ADDW     : MYRISCVX_CA<0b01, 0b100>  // (RV64/128; RV32 RES)
def C_J        : MYRISCVX_CJ<0b01, 0b101>
def C_BEQZ     : MYRISCVX_CB<0b01, 0b110>
def C_BNEZ     : MYRISCVX_CB<0b01, 0b111>
def C_SLLI     : MYRISCVX_CI<0b10, 0b000>  // (HINT, rd=0; RV32 NSE, nzuimm[5]=1)
def C_FLDSP    : MYRISCVX_CI<0b10, 0b001>  // (RV32/64)
def C_LWSP     : MYRISCVX_CI<0b10, 0b010>  // (RES, rd=0)
def C_FLWSP    : MYRISCVX_CI<0b10, 0b011>  // (RV32)
def C_LDSP     : MYRISCVX_CI<0b10, 0b011>  // (RV64/128; RES, rd=0)
def C_JR       : MYRISCVX_CR<0b10, 0b100>  // (RES, rs1=0)
def C_MV       : MYRISCVX_CR<0b10, 0b100>  // (HINT, rd=0)
def C_EBREAK   : MYRISCVX_CR<0b10, 0b100>
def C_JALR     : MYRISCVX_CR<0b10, 0b100>
def C_ADD      : MYRISCVX_CR<0b10, 0b100>  // (HINT, rd=0)
def C_FSDSP    : MYRISCVX_CSS<0b10, 0b101>  // (RV32/64)
def C_SWSP     : MYRISCVX_CSS<0b10, 0b110>
def C_FSWSP    : MYRISCVX_CSS<0b10, 0b111>  // (RV32)
def C_SDSP     : MYRISCVX_CSS<0b10, 0b111>  // (RV64/128)
