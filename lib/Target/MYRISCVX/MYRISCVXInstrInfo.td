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
// MYRISCVX profiles and nodes
//===----------------------------------------------------------------------===//

def SDT_MYRISCVXRet : SDTypeProfile<0, 1, [SDTCisInt<0>]>;

// Return
def MYRISCVXRet : SDNode<"MYRISCVXISD::Ret", SDTNone,
                         [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

include "MYRISCVXInstrFormats.td"

//===----------------------------------------------------------------------===//
// MYRISCVX Operand, Complex Patterns and Transformations Definitions.
//===----------------------------------------------------------------------===//

// Transformation Function - get the lower 12 bits.
def LO12 : SDNodeXForm<imm, [{
  return getImm(N, N->getZExtValue() & 0xfff);
}]>;

// Transformation Function - get the higher 20 bits.
def HI20 : SDNodeXForm<imm, [{
  return getImm(N, (N->getZExtValue() >> 12) & 0xfffff);
}]>;

// Hi and Lo nodes are used to handle global addresses. Used on
// MYRISCVXISelLowering to lower stuff like GlobalAddress, ExternalSymbol
// static model. (nothing to do with MYRISCVX Registers Hi and Lo)
def MYRISCVXHi    : SDNode<"MYRISCVXISD::Hi", SDTIntUnaryOp>;
def MYRISCVXLo    : SDNode<"MYRISCVXISD::Lo", SDTIntUnaryOp>;
def MYRISCVXGPRel : SDNode<"MYRISCVXISD::GPRel", SDTIntUnaryOp>;

// Signed Operand
def simm12 : Operand<i32> {
  let DecoderMethod= "DecodeSimm12";
}
def simm20 : Operand<i32> {
  let DecoderMethod= "DecodeSimm20";
}
// Unsigned Operand
def uimm12 : Operand<i32> {
  let PrintMethod = "printUnsignedImm";
}

def immSExt12 : PatLeaf<(imm), [{ return isInt<12>(N->getSExtValue()); }]>;
def immSExt20 : PatLeaf<(imm), [{ return isInt<20>(N->getSExtValue()); }]>;
def immZExt12 : PatLeaf<(imm), [{
  if (N->getValueType(0) == MVT::i32)
    return (uint32_t)N->getZExtValue() == (unsigned short)N->getZExtValue();
  else
    return (uint64_t)N->getZExtValue() == (unsigned short)N->getZExtValue();
}], LO12>;

// Immediate can be loaded with LUi (32-bit int with lower 16-bit cleared).
def immLow12Zero : PatLeaf<(imm), [{
  int64_t Val = N->getSExtValue();
  return isInt<32>(Val) && !(Val & 0x0fff);
}]>;

// shamt field must fit in 5 bits.
def immZExt5 : ImmLeaf<i32, [{return Imm == (Imm & 0x1f);}]>;

def shamt : Operand<i32>;

// JAL
def brtarget20    : Operand<OtherVT> {
  let EncoderMethod = "getBranch20TargetOpValue";
  let OperandType   = "OPERAND_PCREL";
  let DecoderMethod = "DecodeBranch20Target";
}


// BEQ, BNE
def brtarget12    : Operand<OtherVT> {
  let EncoderMethod = "getBranch12TargetOpValue";
  let OperandType   = "OPERAND_PCREL";
  let DecoderMethod = "DecodeBranch12Target";
}


def MYRISCVXMemAsmOperand : AsmOperandClass {
  let Name = "Mem";
  let ParserMethod = "parseMemOperand";
}

// Address operand
def mem : Operand<iPTR> {
  let PrintMethod   = "printMemOperand";
  let MIOperandInfo = (ops GPR, simm12);
  let EncoderMethod = "getMemEncoding";
  let ParserMatchClass = MYRISCVXMemAsmOperand;
}

def mem_ea : Operand<iPTR> {
  let PrintMethod = "printMemOperandEA";
  let MIOperandInfo = (ops GPR, simm12);
  let EncoderMethod = "getMemEncoding";
}

// MYRISCVX Address Mode! SDNode frameindex could possibily be a match
// since load and store instructions from stack used it.
def addr :
  ComplexPattern<iPTR, 2, "SelectAddr", [frameindex]>;

def MYRISCVXWrapper    : SDNode<"MYRISCVXISD::Wrapper", SDTIntBinOp>;

def RelocPIC : Predicate<"TM.getRelocationModel() == Reloc::PIC_">;

//===----------------------------------------------------------------------===//
// Pattern fragment for load/store
//===----------------------------------------------------------------------===//

class AlignedLoad<PatFrag Node> :
  PatFrag<(ops node:$ptr), (Node node:$ptr), [{
  LoadSDNode *LD = cast<LoadSDNode>(N);
  return LD->getMemoryVT().getSizeInBits()/8 <= LD->getAlignment();
}]>;

class AlignedStore<PatFrag Node> :
  PatFrag<(ops node:$val, node:$ptr), (Node node:$val, node:$ptr), [{
  StoreSDNode *SD = cast<StoreSDNode>(N);
  return SD->getMemoryVT().getSizeInBits()/8 <= SD->getAlignment();
}]>;

// Load/Store PatFrags.
def load_a          : AlignedLoad<load>;
def sextloadi8_a    : AlignedLoad<sextloadi8>;
def zextloadi8_a    : AlignedLoad<zextloadi8>;
def sextloadi16_a   : AlignedLoad<sextloadi16>;
def zextloadi16_a   : AlignedLoad<zextloadi16>;
def extloadi16_a    : AlignedLoad<extloadi16>;

def store_a         : AlignedStore<store>;
def truncstorei8_a  : AlignedStore<truncstorei8>;
def truncstorei16_a : AlignedStore<truncstorei16>;

//===----------------------------------------------------------------------===//
// Instructions specific format
//===----------------------------------------------------------------------===//

// Memory Load/Store
let canFoldAsLoad = 1 in
class LoadM<bits<7> opcode, bits<3> funct3, string instr_asm, PatFrag OpNode, RegisterClass RC,
            Operand MemOpnd, bit Pseudo>:
  MYRISCVX_I<opcode, funct3, (outs RC:$ra), (ins MemOpnd:$addr),
     !strconcat(instr_asm, "\t$ra, $addr"),
     [(set RC:$ra, (OpNode addr:$addr))], IILoad> {
  let isPseudo = Pseudo;
}

class StoreM<bits<7> opcode, bits<3> funct3, string instr_asm, PatFrag OpNode, RegisterClass RC,
             Operand MemOpnd, bit Pseudo>:
  MYRISCVX_S<opcode, funct3, (outs), (ins RC:$ra, MemOpnd:$addr),
     !strconcat(instr_asm, "\t$ra, $addr"),
     [(OpNode RC:$ra, addr:$addr)], IIStore> {
  let isPseudo = Pseudo;
}


class EffectiveAddress<string instr_asm, RegisterClass RC, Operand Mem> :
  MYRISCVX_I<0b0010011, 0b000, (outs RC:$ra), (ins Mem:$addr),
     instr_asm, [(set RC:$ra, addr:$addr)], IIAlu>;

// FrameIndexes are legalized when they are operands from load/store
// instructions. The same not happens for stack address copies, so an
// add op with mem ComplexPattern is used and the stack address copy
// can be matched. It's similar to Sparc LEA_ADDRi
def LEA_ADDI : EffectiveAddress<"addi\t$ra, $addr", GPR, mem_ea> {
  let isCodeGenOnly = 1;
}


//@ 32-bit load.
multiclass LoadM32<bits<7> opcode, bits<3> funct3, string instr_asm, PatFrag OpNode,
                   bit Pseudo = 0> {
  def #NAME# : LoadM<opcode, funct3, instr_asm, OpNode, GPR, mem, Pseudo>;
}

// 32-bit store.
multiclass StoreM32<bits<7> opcode, bits<3> funct3, string instr_asm, PatFrag OpNode,
                    bit Pseudo = 0> {
  def #NAME# : StoreM<opcode, funct3, instr_asm, OpNode, GPR, mem, Pseudo>;
}

// Arithmetic and logical instructions with 2 register operands.
class ArithLogicR<bits<7> opcode, bits<3> funct3, bits<7>funct7,
                  string instr_asm, SDNode OpNode,
                  RegisterClass RC> :
  MYRISCVX_R<opcode, funct3, funct7, (outs RC:$rd), (ins RC:$rs1, RC:$rs2),
  !strconcat(instr_asm, "\t$rd, $rs1, $rs2"),
  [(set RC:$rd, (OpNode RC:$rs1, RC:$rs2))], IIAlu> {
    let isReMaterializable = 1;
}


// Arithmetic and logical instructions with 2 register operands.
class ArithLogicI<bits<7> opcode, bits<3> funct3,
                  string instr_asm, SDNode OpNode,
                  Operand Od, PatLeaf imm_type, RegisterClass RC> :
  MYRISCVX_I<opcode, funct3, (outs RC:$rd), (ins RC:$rs1, Od:$imm12),
  !strconcat(instr_asm, "\t$rd, $rs1, $imm12"),
  [(set RC:$rd, (OpNode RC:$rs1, imm_type:$imm12))], IIAlu> {
    let isReMaterializable = 1;
}

// Arithmetic and logical instructions with 2 register and one immediate.
class ArithLogicU<bits<7> opcode,
                  string instr_asm, RegisterClass RC, Operand Od, PatLeaf imm_type> :
  MYRISCVX_U<opcode, (outs RC:$rd), (ins Od:$imm20),
    !strconcat(instr_asm, "\t$rd, $imm20"), [], IIAlu> {
      let isReMaterializable = 1;
}


// Branch instructions with 2 register operands.
class CondBranch12<bits<7> opcode, bits<3> funct3,
                string instr_asm,
                PatFrag cond_op, RegisterClass RC> :
  MYRISCVX_B<opcode, funct3, (outs), (ins RC:$rs1, RC:$rs2, brtarget12:$imm12),
  !strconcat(instr_asm, "\t$rs1, $rs2, $imm12"),
  [], IIAlu> {
    let isBranch = 1;
    let isTerminator = 1;
}


// Unconditional Branch
class JumpOffset<bits<7> opcode, string instr_asm, RegisterClass RC>:
  MYRISCVX_J<opcode, (outs RC:$rd), (ins brtarget20:$addr),
  !strconcat(instr_asm, "\t$rd, $addr"), [], IIAlu> {
    let isBranch = 1;
    let isTerminator = 1;
    let isBarrier = 1;
    let hasDelaySlot = 0;
}


class JumpOffsetX0<bits<7> opcode, string instr_asm>:
  JumpOffset<opcode, instr_asm, GPR> {
    let rd = 0;
}


// Return instruction
class RetGPR<bits<7> opcode, bits<3> funct3, string instr_asm, RegisterClass RC>:
  MYRISCVX_I<opcode, funct3, (outs), (ins RC:$rs1),
             !strconcat(instr_asm, "\t$rs1"), [(brind RC:$rs1)], IIBranch> {
  let rd = 0;
  let isReturn = 1;
}

// Shifts
class ShiftImm<bits<7> opcode, bits<3> funct3, bit arithshift, string instr_asm,
               SDNode OpNode, PatFrag PF, Operand ImmOpnd,
               RegisterClass RC>:
  MYRISCVX_ISHIFT<opcode, funct3, arithshift, (outs RC:$rd), (ins RC:$rs1, ImmOpnd:$shamt),
                  !strconcat(instr_asm, "\t$rd, $rs1, $shamt"),
                  [(set GPR:$rd, (OpNode RC:$rs1, PF:$shamt))], IIAlu> {
}


// 32-bit shift instructions.
class ShiftImm32<bits<7> opcode, bits<3> funct3, bit arithshift, string instr_asm,
                         SDNode OpNode>:
  ShiftImm<opcode, funct3, arithshift, instr_asm, OpNode, immZExt5, shamt, GPR>;


class ShiftR<bits<7> opcode, bits<7> funct7, bits<3> funct3,
             bits<4> isRotate, string instr_asm,
             SDNode OpNode, RegisterClass RC>:
  MYRISCVX_R<opcode, funct3, funct7, (outs RC:$ra), (ins RC:$rb, RC:$rc),
          !strconcat(instr_asm, "\t$ra, $rb, $rc"),
          [(set GPR:$ra, (OpNode RC:$rb, RC:$rc))], IIAlu> {
}


// SetCC
class SetCC_R<bits<7> opcode, bits<3> funct3, bits<7> funct7,
              string instr_asm, PatFrag cond_op,
              RegisterClass RC> :
  MYRISCVX_R<opcode, funct3, funct7, (outs RC:$ra), (ins RC:$rb, RC:$rc),
             !strconcat(instr_asm, "\t$ra, $rb, $rc"),
             [(set GPR:$ra, (cond_op RC:$rb, RC:$rc))], IIAlu> {
               let isReMaterializable = 1;
}


class SetCC_I<bits<7> opcode, bits<3> funct3,
              string instr_asm, PatFrag cond_op,
              Operand imm, PatLeaf imm_type, RegisterClass RC> :
  MYRISCVX_I<opcode, funct3, (outs RC:$ra), (ins RC:$rb, imm:$imm12),
             !strconcat(instr_asm, "\t$ra, $rb, $imm12"),
             [(set GPR:$ra, (cond_op RC:$rb, imm_type:$imm12))], IIAlu> {
               let isReMaterializable = 1;
}


//===----------------------------------------------------------------------===//
// Instruction definition
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// MYRISCVX Instructions
//===----------------------------------------------------------------------===//


def ADDI : ArithLogicI<0b0010011, 0b000, "addi", add, simm12, immSExt12, GPR>;
def XORI : ArithLogicI<0b0010011, 0b100, "xori", xor, uimm12, immZExt12, GPR>;
def ORI  : ArithLogicI<0b0010011, 0b110, "ori",  or,  uimm12, immZExt12, GPR>;
def ANDI : ArithLogicI<0b0010011, 0b111, "andi", and, uimm12, immZExt12, GPR>;

def LUI  : ArithLogicU<0b0110111, "lui", GPR, simm20, immSExt20>;
def ADD  : ArithLogicR<0b0110011, 0b000, 0b0000000, "add", add, GPR>;
def SUB  : ArithLogicR<0b0110011, 0b000, 0b0100000, "sub", sub, GPR>;

def SLL  : ShiftR<0b0110011, 0b0000000, 0b001, 0x0, "sll", shl, GPR>;
def AND  : ArithLogicR<0b0110011, 0b111, 0b0000000, "and", and, GPR>;
def SRL  : ShiftR<0b0110011, 0b0000000, 0b101, 0x0, "srl", srl, GPR>;
def SRA  : ShiftR<0b0110011, 0b0100000, 0b101, 0x0, "sra", sra, GPR>;
def OR   : ArithLogicR<0b0110011, 0b110, 0b0000000, "or",  or,  GPR>;
def XOR  : ArithLogicR<0b0110011, 0b100, 0b0000000, "xor", xor, GPR>;

def SRLI : ShiftImm32<0b0010011, 0b101, 0, "srli", srl>;
def SLLI : ShiftImm32<0b0010011, 0b001, 0, "slli", shl>;
def SRAI : ShiftImm32<0b0010011, 0b101, 1, "srai", sra>;

def MUL   : ArithLogicR<0b0110011, 0b000, 0b0000001, "mul",    mul,   GPR>;
def MULH  : ArithLogicR<0b0110011, 0b001, 0b0000001, "mulh",   mulhs, GPR>;
def MULHSU: ArithLogicR<0b0110011, 0b010, 0b0000001, "mulhsu", mulhs, GPR>;
def MULHU : ArithLogicR<0b0110011, 0b011, 0b0000001, "mulhu",  mulhu, GPR>;
def DIV   : ArithLogicR<0b0110011, 0b100, 0b0000001, "div",    sdiv,  GPR>;
def DIVU  : ArithLogicR<0b0110011, 0b101, 0b0000001, "divu",   udiv,  GPR>;
def REM   : ArithLogicR<0b0110011, 0b110, 0b0000001, "rem",    srem,  GPR>;
def REMU  : ArithLogicR<0b0110011, 0b111, 0b0000001, "remu",   urem,  GPR>;

def SLTI  : SetCC_I<0b0010011, 0b010, "slti",  setlt,  simm12, immSExt12, GPR>;
def SLTIU : SetCC_I<0b0010011, 0b011, "sltiu", setult, simm12, immSExt12, GPR>;
def SLT   : SetCC_R<0b0110011, 0b010, 0b0000000, "slt",  setlt,  GPR>;
def SLTU  : SetCC_R<0b0110011, 0b011, 0b0000000, "sltu", setult, GPR>;

def JAL : JumpOffset<0b1101111, "jal", GPR>;
def J   : JumpOffsetX0<0b1101111, "j">;
def JR  : ArithLogicI<0b1100111, 0b000, "jr", add, simm12, immSExt12, GPR>;
def RET : RetGPR<0b1100111, 0b000, "ret", GPR>;

let isReturn=1, isTerminator=1, hasDelaySlot=0, isBarrier=1, hasCtrlDep=1 in
  def RetRA : MYRISCVXPseudo<(outs), (ins), "", [(MYRISCVXRet)]>;


/// Load and Store Instructions
///  aligned
defm LW     : LoadM32 <0b0000011, 0b010, "lw",  load_a>;
defm SW     : StoreM32<0b0100011, 0b010, "sw",  store_a>;
defm LB     : LoadM32 <0b0000011, 0b000, "lb",  sextloadi8_a>;
defm LBU    : LoadM32 <0b0000011, 0b100, "lbu", zextloadi8_a>;
defm SB     : StoreM32<0b0100011, 0b000, "sb",  truncstorei8_a>;
defm LH     : LoadM32 <0b0000011, 0b001, "lh",  sextloadi16_a>;
defm LHU    : LoadM32 <0b0000011, 0b101, "lhu", zextloadi16_a>;
defm SH     : StoreM32<0b0100011, 0b001, "sh",  truncstorei16_a>;

def BEQ     : CondBranch12<0b1100011, 0b000, "beq" , seteq,  GPR>;
def BNE     : CondBranch12<0b1100011, 0b001, "bne" , setne,  GPR>;
def BLT     : CondBranch12<0b1100011, 0b100, "blt" , setlt,  GPR>;
def BGE     : CondBranch12<0b1100011, 0b101, "bge" , setge,  GPR>;
def BLTU    : CondBranch12<0b1100011, 0b110, "bltu", setult, GPR>;
def BGEU    : CondBranch12<0b1100011, 0b111, "bgeu", setuge, GPR>;

//===----------------------------------------------------------------------===//
//  Arbitrary patterns that map to one or more instructions
//===----------------------------------------------------------------------===//

// Small immediates
def : Pat<(i32 immSExt12:$in),
          (ADDI ZERO, imm:$in)>;
def : Pat<(i32 immZExt12:$in),
          (ORI ZERO, imm:$in)>;

def : Pat<(i32 immLow12Zero:$in),
          (LUI (HI20 imm:$in))>;

def : Pat<(i32 imm:$imm),
          (ORI (LUI (HI20 imm:$imm)), (LO12 imm:$imm))>;

// hi/lo relocs
def : Pat<(MYRISCVXHi tglobaladdr:$in), (LUI tglobaladdr:$in)>;
def : Pat<(MYRISCVXLo tglobaladdr:$in), (ORI ZERO, tglobaladdr:$in)>;
def : Pat<(add GPR:$hi, (MYRISCVXLo tglobaladdr:$lo)),
          (ORI GPR:$hi, tglobaladdr:$lo)>;
// gp_rel relocs
def : Pat<(add GPR:$gp, (MYRISCVXGPRel tglobaladdr:$in)),
          (ORI GPR:$gp, tglobaladdr:$in)>;

//@ wrapper_pic
class WrapperPat<SDNode node, Instruction ORiOp, RegisterClass RC>:
      Pat<(MYRISCVXWrapper RC:$gp, node:$in),
          (ORiOp RC:$gp, node:$in)>;

def : WrapperPat<tglobaladdr, ORI, GPR>;
def : Pat<(i32 (zextloadi1  addr:$src)), (LBU addr:$src)>;
def : Pat<(i32 (zextloadi8  addr:$src)), (LBU addr:$src)>;
def : Pat<(i32 (zextloadi16 addr:$src)), (LHU addr:$src)>;

def : Pat<(i32 (sextloadi1  addr:$src)), (LB addr:$src)>;
def : Pat<(i32 (sextloadi8  addr:$src)), (LB addr:$src)>;
def : Pat<(i32 (sextloadi16 addr:$src)), (LH addr:$src)>;

def : Pat<(i32 (extloadi1  addr:$src)), (LBU addr:$src)>;
def : Pat<(i32 (extloadi8  addr:$src)), (LBU addr:$src)>;
def : Pat<(i32 (extloadi16 addr:$src)), (LHU addr:$src)>;

def : Pat<(sext_inreg GPR:$ra, i8),  (SRAI (SLLI GPR:$ra, 24), 24)>;
def : Pat<(sext_inreg GPR:$ra, i16), (SRAI (SLLI GPR:$ra, 16), 16)>;

def : Pat<(br bb:$simm20), (J bb:$simm20)>;

// Define pattern expansions for setcc operations that aren't directly
// handled by a RISC-V instruction.
def : Pat<(seteq  GPR:$rs1, GPR:$rs2), (SLTIU (XOR GPR:$rs1, GPR:$rs2), 1)>;
def : Pat<(setne  GPR:$rs1, GPR:$rs2), (SLTU ZERO, (XOR GPR:$rs1, GPR:$rs2))>;
def : Pat<(setugt GPR:$rs1, GPR:$rs2), (SLTU GPR:$rs2, GPR:$rs1)>;
def : Pat<(setuge GPR:$rs1, GPR:$rs2), (XORI (SLTU GPR:$rs1, GPR:$rs2), 1)>;
def : Pat<(setule GPR:$rs1, GPR:$rs2), (XORI (SLTU GPR:$rs2, GPR:$rs1), 1)>;
def : Pat<(setgt  GPR:$rs1, GPR:$rs2), (SLT GPR:$rs2, GPR:$rs1)>;
def : Pat<(setge  GPR:$rs1, GPR:$rs2), (XORI (SLT GPR:$rs1, GPR:$rs2), 1)>;
def : Pat<(setle  GPR:$rs1, GPR:$rs2), (XORI (SLT GPR:$rs2, GPR:$rs1), 1)>;

// brcond for slt instruction
multiclass BrcondPatsSlt<RegisterClass RC, Instruction BEQOp, Instruction BNEOp,
                      Instruction SLTOp, Instruction SLTuOp, Instruction SLTiOp,
                      Instruction SLTiuOp, Register ZEROReg> {
def : Pat<(brcond (i32 (setne RC:$lhs, 0)), bb:$dst),
              (BNEOp RC:$lhs, ZEROReg, bb:$dst)>;
def : Pat<(brcond (i32 (seteq RC:$lhs, 0)), bb:$dst),
              (BEQOp RC:$lhs, ZEROReg, bb:$dst)>;

def : Pat<(brcond (i32 (seteq RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQOp RC:$lhs, RC:$rhs, bb:$dst)>;
def : Pat<(brcond (i32 (setueq RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQOp RC:$lhs, RC:$rhs, bb:$dst)>;
def : Pat<(brcond (i32 (setne RC:$lhs, RC:$rhs)), bb:$dst),
              (BNEOp RC:$lhs, RC:$rhs, bb:$dst)>;
def : Pat<(brcond (i32 (setune RC:$lhs, RC:$rhs)), bb:$dst),
              (BNEOp RC:$lhs, RC:$rhs, bb:$dst)>;
def : Pat<(brcond (i32 (setlt RC:$lhs, RC:$rhs)), bb:$dst),
              (BNE (SLTOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setult RC:$lhs, RC:$rhs)), bb:$dst),
              (BNE (SLTuOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setgt RC:$lhs, RC:$rhs)), bb:$dst),
              (BNE (SLTOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setugt RC:$lhs, RC:$rhs)), bb:$dst),
              (BNE (SLTuOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setle RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQ (SLTOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setule RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQ (SLTuOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setge RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQ (SLTOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
def : Pat<(brcond (i32 (setuge RC:$lhs, RC:$rhs)), bb:$dst),
              (BEQ (SLTuOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
def : Pat<(brcond RC:$cond, bb:$dst),
              (BNEOp RC:$cond, ZEROReg, bb:$dst)>;
}

defm : BrcondPatsSlt<GPR, BEQ, BNE, SLT, SLTU, SLTI, SLTIU, ZERO>;

// // peepholes
// def : Pat<(store (i32 0), addr:$dst), (SW ZERO, addr:$dst)>;
