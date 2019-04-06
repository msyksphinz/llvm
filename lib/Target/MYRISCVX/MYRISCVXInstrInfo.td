//===- MYRISCVXInstrInfo.td - Target Description for MYRISCVX Target -*- tablegen -*-=//
//
// The LLVM Compiler Infrastructure
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

def SDT_MYRISCVXSelectCC   : SDTypeProfile<1, 5, [SDTCisSameAs<1, 2>,
                                               SDTCisSameAs<0, 4>,
                                               SDTCisSameAs<4, 5>]>;
def SDT_MYRISCVXJmpLink : SDTypeProfile<0, 1, [SDTCisVT<0, iPTR>]>;

def SDT_MYRISCVXCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i32>]>;
def SDT_MYRISCVXCallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;

// Exception handling related node and instructions.
// The conversion sequence is:
// ISD::EH_RETURN -> MYRISCVXISD::EH_RETURN ->
// MYRISCVXeh_return -> (stack change + indirect branch)
//
// MYRISCVXeh_return takes the place of regular return instruction
// but takes two arguments (V1, V0) which are used for storing
// the offset and return address respectively.
def SDT_MYRISCVXEHRET : SDTypeProfile<0, 2, [SDTCisInt<0>, SDTCisPtrTy<1>]>;

def MYRISCVXehret : SDNode<"MYRISCVXISD::EH_RETURN", SDT_MYRISCVXEHRET,
                           [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

// Tail call
def MYRISCVXTailCall : SDNode<"MYRISCVXISD::TailCall", SDT_MYRISCVXJmpLink,
                   [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

// Return
def MYRISCVXRet : SDNode<"MYRISCVXISD::Ret", SDTNone,
                         [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

def SelectCC : SDNode<"MYRISCVXISD::SELECT_CC", SDT_MYRISCVXSelectCC,
                      [SDNPInGlue]>;

// Hi and Lo nodes are used to handle global addresses. Used on
// MYRISCVXISelLowering to lower stuff like GlobalAddress, ExternalSymbol
// static model. (nothing to do with MYRISCVX Registers Hi and Lo)
def MYRISCVXHi : SDNode<"MYRISCVXISD::Hi", SDTIntUnaryOp>;
def MYRISCVXLo : SDNode<"MYRISCVXISD::Lo", SDTIntUnaryOp>;
def MYRISCVXGPRel : SDNode<"MYRISCVXISD::GPRel", SDTIntUnaryOp>;

def MYRISCVXWrapper : SDNode<"MYRISCVXISD::Wrapper", SDTIntBinOp>;
def RelocPIC : Predicate<"TM.getRelocationModel() == Reloc::PIC_">;

// Call
def MYRISCVXJmpLink : SDNode<"MYRISCVXISD::JmpLink",SDT_MYRISCVXJmpLink,
                         [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue,
                         SDNPVariadic]>;

//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

include "MYRISCVXInstrFormats.td"

//===----------------------------------------------------------------------===//
// MYRISCVX Operand, Complex Patterns and Transformations Definitions.
//===----------------------------------------------------------------------===//

// BEQ, BNE
def brtarget12    : Operand<OtherVT> {
  let EncoderMethod = "getBranch12TargetOpValue";
  let OperandType   = "OPERAND_PCREL";
  let DecoderMethod = "DecodeBranch12Target";
}

// JAL
def brtarget20    : Operand<OtherVT> {
  let EncoderMethod = "getBranch20TargetOpValue";
  let OperandType   = "OPERAND_PCREL";
  let DecoderMethod = "DecodeBranch20Target";
}

// JMP
def jmptarget    : Operand<OtherVT> {
  let EncoderMethod = "getJumpTargetOpValue";
  let OperandType = "OPERAND_PCREL";
}


// Instruction operand types
// Signed Operand
def simm12 : Operand<i32> {
  let DecoderMethod= "DecodeSimm12";
}

// Signed Operand
def simm20 : Operand<i32> {
  let DecoderMethod= "DecodeSimm20";
}

// Unsigned Operand
def uimm12 : Operand<i32> {
  let PrintMethod = "printUnsignedImm";
}


// Address operand
def mem : Operand<iPTR> {
  let PrintMethod   = "printMemOperand";
  let MIOperandInfo = (ops GPR, simm12);
  let EncoderMethod = "getMemEncoding";
}

def mem_ea : Operand<iPTR> {
  let PrintMethod   = "printMemOperandEA";
  let MIOperandInfo = (ops GPR, simm12);
  let EncoderMethod = "getMemEncoding";
}

// Transformation Function - get the lower 12 bits.
def LO12 : SDNodeXForm<imm, [{
  return getImm(N, N->getZExtValue() & 0xfff);
}]>;

// Transformation Function - get the higher 20 bits.
def HI20 : SDNodeXForm<imm, [{
  return getImm(N, (N->getZExtValue() >> 12) & 0xfffff);
}]>;

// Node immediate fits as 12-bit sign extended on target immediate.
// e.g. addi, andi
def immSExt12 : PatLeaf<(imm), [{ return isInt<12>(N->getSExtValue()); }]>;
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

// MYRISCVX Address Mode! SDNode frameindex could possibily be a match
// since load and store instructions from stack used it.
def addr :
  ComplexPattern<iPTR, 2, "SelectAddr", [frameindex], [SDNPWantParent]>;

// We need these two pseudo instructions to avoid offset calculation for long
// branches. See the comment in file MYRISCVXLongBranch.cpp for detailed
// explanation.
// Expands to: lui $dst, %hi($tgt - $baltgt)
def LONG_BRANCH_LUI : MYRISCVXPseudo<(outs GPR:$dst),
    (ins jmptarget:$tgt, jmptarget:$baltgt), "", []>;

// Expands to: addiu $dst, $src, %lo($tgt - $baltgt)
def LONG_BRANCH_ADDI : MYRISCVXPseudo<(outs GPR:$dst),
    (ins GPR:$src, jmptarget:$tgt, jmptarget:$baltgt), "", []>;

// These are target-independent nodes, but have target-specific formats.
def callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_MYRISCVXCallSeqStart,
                           [SDNPHasChain, SDNPOutGlue]>;
def callseq_end : SDNode<"ISD::CALLSEQ_END", SDT_MYRISCVXCallSeqEnd,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;


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
def load_a        : AlignedLoad<load>;
def sextloadi8_a  : AlignedLoad<sextloadi8>;
def zextloadi8_a  : AlignedLoad<zextloadi8>;
def sextloadi16_a : AlignedLoad<sextloadi16>;
def zextloadi16_a : AlignedLoad<zextloadi16>;
def extloadi16_a  : AlignedLoad<extloadi16>;

def store_a         : AlignedStore<store>;
def truncstorei8_a  : AlignedStore<truncstorei8>;
def truncstorei16_a : AlignedStore<truncstorei16>;

//===----------------------------------------------------------------------===//
// Instructions specific format
//===----------------------------------------------------------------------===//

class EffectiveAddress<string instr_asm, RegisterClass RC, Operand Mem> :
  FI<0b0000011, 0b010, (outs RC:$ra), (ins Mem:$addr),
     instr_asm, [(set RC:$ra, addr:$addr)], IIAlu> {
}


// Arithmetic and logical instructions with 2 register operands.
class ArithLogicR<bits<7> opcode, bits<3> funct3,
                  string instr_asm, SDNode OpNode,
                  RegisterClass RC> :
  FI<opcode, funct3, (outs GPR:$ra), (ins RC:$rb, RC:$rc),
  !strconcat(instr_asm, "\t$ra, $rb, $rc"),
  [(set GPR:$ra, (OpNode RC:$rb, RC:$rc))], IIAlu> {
    let isReMaterializable = 1;
}


// Arithmetic and logical instructions with 2 register operands.
class ArithLogicI<bits<7> opcode, bits<3> funct3,
                  string instr_asm, SDNode OpNode,
                  Operand Od, PatLeaf imm_type, RegisterClass RC> :
  FI<opcode, funct3, (outs GPR:$ra), (ins RC:$rb, Od:$imm12),
  !strconcat(instr_asm, "\t$ra, $rb, $imm12"),
  [(set GPR:$ra, (OpNode RC:$rb, imm_type:$imm12))], IIAlu> {
    let isReMaterializable = 1;
}


// Branch instructions with 2 register operands.
class CBranch12<bits<7> opcode, bits<3> funct3,
                string instr_asm,
                PatFrag cond_op, RegisterClass RC> :
  FB<opcode, funct3, (outs), (ins RC:$ra, RC:$rb, brtarget12:$imm12),
  !strconcat(instr_asm, "\t$ra, $rb, $imm12"),
  [], IIAlu> {
    let isBranch = 1;
    let isTerminator = 1;
}

class UncondBranch<bits<7> opcode, string instr_asm>:
  FJ<opcode, (outs), (ins brtarget20:$addr),
  !strconcat(instr_asm, "\t$addr"), [(br bb:$addr)], IIAlu> {
    let isBranch = 1;
    let isTerminator = 1;
    let isBarrier = 1;
    let hasDelaySlot = 0;
}


// Arithmetic and logical instructions with 3 register operands.
class ArithLogicU<bits<7> opcode,
                  string instr_asm, Operand Od, PatLeaf imm_type> :
  FU<opcode, (outs GPR:$ra), (ins Od:$imm20),
    !strconcat(instr_asm, "\t$ra, $imm20"), [], IIAlu> {
      let isReMaterializable = 1;
}

// Memory Load/Store
let canFoldAsLoad = 1 in
class LoadM<bits<7> opcode, bits<3> funct3, string instr_asm, PatFrag OpNode, RegisterClass RC,
            Operand MemOpnd, bit Pseudo>:
  FI<opcode, funct3, (outs RC:$ra), (ins MemOpnd:$addr),
     !strconcat(instr_asm, "\t$ra, $addr"),
     [(set RC:$ra, (OpNode addr:$addr))], IILoad> {
  let isPseudo = Pseudo;
}


class StoreM<bits<7> opcode, bits<3> funct3, string instr_asm, PatFrag OpNode, RegisterClass RC,
             Operand MemOpnd, bit Pseudo>:
  FI<opcode, funct3, (outs), (ins RC:$ra, MemOpnd:$addr),
     !strconcat(instr_asm, "\t$ra, $addr"),
     [(OpNode RC:$ra, addr:$addr)], IIStore> {
  let isPseudo = Pseudo;
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


//@JumpFR {
let isBranch=1, isTerminator=1, isBarrier=1, imm12=0, hasDelaySlot = 0,
isIndirectBranch = 1 in
class JumpFR<bits<7> opcode, bits<3> funct3, string instr_asm, RegisterClass RC>:
  FI<opcode, funct3, (outs), (ins RC:$ra),
    !strconcat(instr_asm, "\t$ra"), [], IIBranch> {
  let rd    = 0;  // Zero
  let rs1   = 1;  // RA
  let imm12 = 0;
/*     !strconcat(instr_asm, "\t$ra"), [(brind RC:$ra)], IIBranch> { */
}
//@JumpFR }


// Return instruction
class RetBase<RegisterClass RC>: JumpFR<0b1100111, 0b000, "ret", RC> {
  let isReturn               = 1;
  let isCodeGenOnly          = 1;
  let hasCtrlDep             = 1;
  let hasExtraSrcRegAllocReq = 1;
}


// Shifts
class shift_rotate_imm<bits<7> opcode, bits<3> funct3, bits<4> isRotate, string instr_asm,
                       SDNode OpNode, PatFrag PF, Operand ImmOpnd,
                       RegisterClass RC>:
  FI<opcode, funct3, (outs GPR:$ra), (ins RC:$rb, ImmOpnd:$shamt),
    !strconcat(instr_asm, "\t$ra, $rb, $shamt"),
    [(set GPR:$ra, (OpNode RC:$rb, PF:$shamt))], IIAlu> {
}

// 32-bit shift instructions.
class shift_rotate_imm32<bits<7> opcode, bits<3> funct3, bits<4> isRotate, string instr_asm,
                         SDNode OpNode>:
  shift_rotate_imm<opcode, funct3, isRotate, instr_asm, OpNode, immZExt5, shamt, GPR>;


class shift_rotate_reg<bits<7> opcode, bits<7> funct7, bits<3> funct3,
                       bits<4> isRotate, string instr_asm,
                       SDNode OpNode, RegisterClass RC>:
  FR<opcode, funct7, funct3, (outs GPR:$ra), (ins RC:$rb, RC:$rc),
     !strconcat(instr_asm, "\t$ra, $rb, $rc"),
     [(set GPR:$ra, (OpNode RC:$rb, RC:$rc))], IIAlu> {
}


// SetCC
class SetCC_R<bits<7> opcode, bits<3> funct3,
              string instr_asm, PatFrag cond_op,
              RegisterClass RC> :
  FI<opcode, funct3, (outs GPR:$ra), (ins RC:$rb, RC:$rc),
  !strconcat(instr_asm, "\t$ra, $rb, $rc"),
  [(set GPR:$ra, (cond_op RC:$rb, RC:$rc))], IIAlu> {
    let isReMaterializable = 1;
}


class SetCC_I<bits<7> opcode, bits<3> funct3,
              string instr_asm, PatFrag cond_op,
              Operand imm, PatLeaf imm_type, RegisterClass RC> :
  FI<opcode, funct3, (outs GPR:$ra), (ins RC:$rb, imm:$imm12),
  !strconcat(instr_asm, "\t$ra, $rb, $imm12"),
  [(set GPR:$ra, (cond_op RC:$rb, imm_type:$imm12))], IIAlu> {
    let isReMaterializable = 1;
}


let isReturn=1, isTerminator=1, hasDelaySlot=0, isBarrier=1, hasCtrlDep=1 in
  def RetRA : MYRISCVXPseudo<(outs), (ins), "", [(MYRISCVXRet)]>;



class IsTailCall {
  bit isCall = 1;
  bit isTerminator = 1;
  bit isReturn = 1;
  bit isBarrier = 1;
  bit hasExtraSrcRegAllocReq = 1;
  bit isCodeGenOnly = 1;
}

def calltarget : Operand<iPTR> {
  let EncoderMethod = "getJumpTargetOpValue";
}


// As stack alignment is always done with addiu, we need a 16-bit immediate
// let Defs = [SP], Uses = [SP] in {
def ADJCALLSTACKDOWN : MYRISCVXPseudo<(outs), (ins simm12:$amt1, simm12:$amt2), "",
                                      [(callseq_start timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP   : MYRISCVXPseudo<(outs), (ins simm12:$amt1, simm12:$amt2), "",
                                      [(callseq_end timm:$amt1, timm:$amt2)]>;
//}

//@def CPRESTORE {
// When handling PIC code the assembler needs .cpload and .cprestore
// directives. If the real instructions corresponding these directives
// are used, we have the same behavior, but get also a bunch of warnings
// from the assembler.
let hasSideEffects = 0 in
def CPRESTORE : MYRISCVXPseudo<(outs), (ins simm12:$loc, GPR:$gp),
                            ".cprestore\t$loc", []>;

let Uses = [A0, A1], isTerminator = 1, isReturn = 1, isBarrier = 1 in {
  def MYRISCVXeh_return32 : MYRISCVXPseudo<(outs), (ins GPR:$spoff, GPR:$dst), "",
                                [(MYRISCVXehret GPR:$spoff, GPR:$dst)]>;
}

//===----------------------------------------------------------------------===//
// Instruction definition
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// MYRISCVX Instructions
//===----------------------------------------------------------------------===//

/// Load and Store Instructions
/// aligned
defm LB  : LoadM32<0b0000011, 0b000, "lb" , sextloadi8_a >;
defm LH  : LoadM32<0b0000011, 0b001, "lh" , sextloadi16_a>;
defm LW  : LoadM32<0b0000011, 0b010, "lw" , load_a       >;
defm LBU : LoadM32<0b0000011, 0b100, "lbu", zextloadi8_a >;
defm LHU : LoadM32<0b0000011, 0b101, "lhu", zextloadi16_a>;

defm SB  : StoreM32<0b0100011, 0b000, "sb", truncstorei8_a >;
defm SH  : StoreM32<0b0100011, 0b001, "sh", truncstorei16_a>;
defm SW  : StoreM32<0b0100011, 0b010, "sw", store_a        >;

/// Arithmetic Instructions (ALU Immediate)
// IR "add" defined in include/llvm/Target/TargetSelectionDAG.td, line 315 (def add).
def ADDI : ArithLogicI<0b0010011, 0b000, "addi", add, simm12, immSExt12, GPR>;

/// Arithmetic Instructions (3-Operand, R-Type)
def JALR : JumpFR<0b1100111, 0b000, "jalr", GPR>;
def : Pat<(brind GPR:$rs1), (JALR GPR:$rs1)>;

// let isCall = 1 in
// def JAL : UncondBranch<0b1101111, "jal">;

def XORI : ArithLogicI<0b0010011, 0b110, "xori", xor, uimm12, immZExt12, GPR>;
def ORI  : ArithLogicI<0b0010011, 0b110, "ori",  or,  uimm12, immZExt12, GPR>;
def ANDI : ArithLogicI<0b0010011, 0b111, "andi", and, uimm12, immZExt12, GPR>;

def LUI  : ArithLogicU<0b0110111, "lui", simm20, immSExt12>;
def ADD  : ArithLogicR<0b0110011, 0b000, "add", add, GPR>;
def SUB  : ArithLogicR<0b0110011, 0b000, "sub", sub, GPR>;

def SLL  : shift_rotate_reg<0b0110011, 0b0000000, 0b001, 0x0, "sll", shl, GPR>;
def AND  : ArithLogicR<0b0110011, 0b111, "and", and, GPR>;
def SRL  : shift_rotate_reg<0b0110011, 0b0000000, 0b101, 0x0, "srl", srl, GPR>;
def SRA  : shift_rotate_reg<0b0110011, 0b0100000, 0b101, 0x0, "sra", sra, GPR>;
def OR   : ArithLogicR<0b0110011, 0b110, "or",  or,  GPR>;
def XOR  : ArithLogicR<0b0110011, 0b100, "xor", xor, GPR>;

def SRLI : shift_rotate_imm32<0b0010011, 0b101, 0x00, "shli", srl>;
def SLLI : shift_rotate_imm32<0b0010011, 0b001, 0x00, "slli", shl>;
def SRAI : shift_rotate_imm32<0b0010011, 0b101, 0x00, "srai", sra>;

def SLTI  : SetCC_I<0b0010011, 0b010, "slti",  setlt,  simm12, immSExt12, GPR>;
def SLTIU : SetCC_I<0b0010011, 0b011, "sltiu", setult, simm12, immSExt12, GPR>;
def SLT   : SetCC_R<0b0110011, 0b010, "slt",  setlt,  GPR>;
def SLTU  : SetCC_R<0b0110011, 0b011, "sltu", setult, GPR>;

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

def BEQ     : CBranch12<0b1100011, 0b000, "beq" , seteq,  GPR>;
def BNE     : CBranch12<0b1100011, 0b001, "bne" , setne,  GPR>;
def BLT     : CBranch12<0b1100011, 0b100, "blt" , setlt,  GPR>;
def BGE     : CBranch12<0b1100011, 0b101, "bge" , setge,  GPR>;
def BLTU    : CBranch12<0b1100011, 0b110, "bltu", setult, GPR>;
def BGEU    : CBranch12<0b1100011, 0b111, "bgeu", setuge, GPR>;

let usesCustomInserter = 1 in
class SelectCC_rrirr<RegisterClass RC, RegisterClass cmpty>
    : MYRISCVXPseudo<(outs RC:$dst),
             (ins cmpty:$lhs, cmpty:$rhs, simm12:$imm,
              RC:$truev, RC:$falsev),
              "",
             [(set RC:$dst,
               (SelectCC cmpty:$lhs,
                         cmpty:$rhs,
                         (i32 imm:$imm),
                         RC:$truev,
                         RC:$falsev))]>;

def Select_GPR_Using_CC_GPR : SelectCC_rrirr<GPR, GPR>;

// Only op DAG can be disabled by ch4_1, data DAG cannot.
def SDT_MYRISCVXDivRem : SDTypeProfile<0, 2,
                                  [SDTCisInt<0>,
                                  SDTCisSameAs<0, 1>]>;

def MUL   : ArithLogicR<0b0110011, 0b000, "mul",    mul,   GPR>;
def MULH  : ArithLogicR<0b0110011, 0b001, "mulh",   mulhs, GPR>;
def MULHSU: ArithLogicR<0b0110011, 0b010, "mulhsu", mulhs, GPR>;
def MULHU : ArithLogicR<0b0110011, 0b011, "mulhu",  mulhu, GPR>;
def DIV   : ArithLogicR<0b0110011, 0b100, "div",    sdiv,  GPR>;
def DIVU  : ArithLogicR<0b0110011, 0b101, "divu",   udiv,  GPR>;
def REM   : ArithLogicR<0b0110011, 0b110, "rem",    srem,  GPR>;
def REMU  : ArithLogicR<0b0110011, 0b111, "remu",   urem,  GPR>;

// FrameIndexes are legalized when they are operands from load/store
// instructions. The same not happens for stack address copies, so an
// add op with mem ComplexPattern is used and the stack address copy
// can be matched. It's similar to Sparc LEA_ADDRI
def LEA_ADDI : EffectiveAddress<"addi\t$ra, $addr", GPR, mem_ea> {
  let isCodeGenOnly = 1;
}

// Jump and Link (Call)
let isCall=1, hasDelaySlot=0 in {
  //@JumpLink {
  class JumpLink<bits<7> opcode, string instr_asm, DAGOperand opnd>:
    FJ<opcode, (outs), (ins opnd:$target),
       !strconcat(instr_asm, "\t$target"), [(MYRISCVXJmpLink tglobaladdr:$target)],
       IIBranch> {
    }

  //@JumpLink }
  class JumpLinkReg<bits<7> opcode, bits<3> funct3, string instr_asm,
                    RegisterClass RC>:
    FI<opcode, funct3, (outs RC:$rd), (ins RC:$rb, variable_ops),
       !strconcat(instr_asm, "\t$rb"), [(MYRISCVXJmpLink RC:$rd, RC:$rb)], IIBranch> {
  }
}

def JAL : JumpLink<0b1101111, "jal", calltarget>;

def : Pat<(MYRISCVXJmpLink (i32 tglobaladdr:$dst)),
          (JAL tglobaladdr:$dst)>;
def : Pat<(MYRISCVXJmpLink (i32 texternalsym:$dst)),
          (JAL texternalsym:$dst)>;
def : Pat<(brind GPR:$rs1), (JALR GPR:$rs1)>;
def : Pat<(MYRISCVXJmpLink GPR:$rs1), (JALR GPR:$rs1)>;

let isCall = 1, isTerminator = 1, isReturn = 1, isBarrier = 1, hasDelaySlot = 1,
    hasExtraSrcRegAllocReq = 1 in {
  class TailCall<Instruction JumpInst> :
    MYRISCVXPseudo<(outs), (ins calltarget:$target), "", []>,
    PseudoInstExpansion<(JumpInst calltarget:$target)>;
  class TailCallReg<RegisterClass RO, Instruction JRInst,
    RegisterClass ResRO = RO> :
    MYRISCVXPseudo<(outs), (ins RO:$rs), "", [(MYRISCVXTailCall RO:$rs)]>,
    PseudoInstExpansion<(JRInst ResRO:$rs)>;
}


def TAILCALL   : TailCall<JAL>;
def TAILCALL_R : TailCallReg<GPR, JALR>;

def : Pat<(MYRISCVXTailCall (iPTR tglobaladdr:$dst)),
          (TAILCALL tglobaladdr:$dst)>;
def : Pat<(MYRISCVXTailCall (iPTR texternalsym:$dst)),
          (TAILCALL texternalsym:$dst)>;

//===----------------------------------------------------------------------===//
// Instruction aliases
//===----------------------------------------------------------------------===//
class MYRISCVXInstAlias<string Asm, dag Result, bit Emit = 0b1> :
  InstAlias<Asm, Result, Emit>;

def : MYRISCVXInstAlias<"move $dst, $src",
  (ADD GPR:$dst, GPR:$src,ZERO), 1>;

/// No operation
def : InstAlias<"nop",  (ADDI ZERO, ZERO, 0)>;

//===----------------------------------------------------------------------===//
// Arbitrary patterns that map to one or more instructions
//===----------------------------------------------------------------------===//
// Small immediates
def : Pat<(i32 immSExt12:$in),
          (ADDI ZERO, imm:$in)>;

def : Pat<(i32 immZExt12:$in),
          (ORI ZERO, imm:$in)>;

def : Pat<(i32 immLow12Zero:$in),
          (LUI (HI20 imm:$in))>;

// Arbitrary immediates
def : Pat<(i32 imm:$imm),
           (ORI (LUI (HI20 imm:$imm)), (LO12 imm:$imm))>;


// hi/lo relocs
def : Pat<(MYRISCVXHi tglobaladdr:$in), (LUI tglobaladdr:$in)>;
def : Pat<(MYRISCVXHi tblockaddress:$in), (LUI tblockaddress:$in)>;
def : Pat<(MYRISCVXHi tjumptable:$in), (LUI tjumptable:$in)>;

def : Pat<(MYRISCVXLo tglobaladdr:$in), (ORI ZERO, tglobaladdr:$in)>;
def : Pat<(add GPR:$hi, (MYRISCVXLo tglobaladdr:$lo)),
          (ORI GPR:$hi, tglobaladdr:$lo)>;
def : Pat<(MYRISCVXLo tblockaddress:$in), (ORI ZERO, tblockaddress:$in)>;
def : Pat<(MYRISCVXLo tjumptable:$in), (ORI ZERO, tjumptable:$in)>;


def : Pat<(add GPR:$hi, (MYRISCVXLo tglobaladdr:$lo)),
          (ORI GPR:$hi, tglobaladdr:$lo)>;
def : Pat<(add GPR:$hi, (MYRISCVXLo tblockaddress:$lo)),
          (ORI GPR:$hi, tblockaddress:$lo)>;
def : Pat<(add GPR:$hi, (MYRISCVXLo tjumptable:$lo)),
          (ORI GPR:$hi, tjumptable:$lo)>;

// gp_rel relocs
def : Pat<(add GPR:$gp, (MYRISCVXGPRel tglobaladdr:$in)),
  (ORI GPR:$gp, tglobaladdr:$in)>;

//@ wrapper_pic
class WrapperPat<SDNode node, Instruction ORiOp, RegisterClass RC>:
      Pat<(MYRISCVXWrapper RC:$gp, node:$in),
          (ORiOp RC:$gp, node:$in)>;

def : WrapperPat<tglobaladdr    , ORI, GPR>;
def : WrapperPat<tconstpool     , ORI, GPR>;
def : WrapperPat<texternalsym   , ORI, GPR>;
def : WrapperPat<tblockaddress  , ORI, GPR>;
def : WrapperPat<tjumptable     , ORI, GPR>;
def : WrapperPat<tglobaltlsaddr , ORI, GPR>;

def : Pat<(i32 (extloadi1  addr:$src)), (LBU addr:$src)>;
def : Pat<(i32 (extloadi8  addr:$src)), (LBU addr:$src)>;
def : Pat<(i32 (extloadi16 addr:$src)), (LHU addr:$src)>;
// peepholes
def : Pat<(store (i32 0), addr:$dst), (SW ZERO, addr:$dst)>;


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

let isBarrier = 1, isBranch = 1, isTerminator = 1 in
def PseudoBR : MYRISCVXPseudo<(outs), (ins calltarget:$simm20), "", [(br bb:$simm20)]>,
               PseudoInstExpansion<(JAL calltarget:$simm20)>;
