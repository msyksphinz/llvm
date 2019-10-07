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

def SDT_MYRISCVXCall       : SDTypeProfile<0, 1, [SDTCisVT<0, iPTR>]>;
def SDT_MYRISCVXSelectCC   : SDTypeProfile<1, 5, [SDTCisSameAs<1, 2>,
                                               SDTCisSameAs<0, 4>,
                                               SDTCisSameAs<4, 5>]>;

def SDT_MYRISCVXRet : SDTypeProfile<0, 1, [SDTCisInt<0>]>;

def SDT_MYRISCVXCallSeqStart : SDCallSeqStart<[SDTCisVT<0, XLenVT>, SDTCisVT<1, XLenVT>]>;
def SDT_MYRISCVXCallSeqEnd   : SDCallSeqEnd  <[SDTCisVT<0, XLenVT>, SDTCisVT<1, XLenVT>]>;

// Return
def MYRISCVXRet : SDNode<"MYRISCVXISD::Ret", SDTNone,
                         [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

def SelectCC : SDNode<"MYRISCVXISD::SELECT_CC", SDT_MYRISCVXSelectCC,
                      [SDNPInGlue]>;

def MYRISCVXCall : SDNode<"MYRISCVXISD::CALL", SDT_MYRISCVXCall,
                          [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                           SDNPVariadic]>;
// These are target-independent nodes, but have target-specific formats.
def MYRISCVXCallSeqStart : SDNode<"ISD::CALLSEQ_START", SDT_MYRISCVXCallSeqStart,
                                  [SDNPHasChain, SDNPOutGlue]>;
def MYRISCVXCallSeqEnd : SDNode<"ISD::CALLSEQ_END", SDT_MYRISCVXCallSeqEnd,
                                [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

include "MYRISCVXInstrFormats.td"

//===----------------------------------------------------------------------===//
// MYRISCVX Operand, Complex Patterns and Transformations Definitions.
//===----------------------------------------------------------------------===//

// Transformation Function - get the lower 12 bits.
def LO12 : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(SignExtend64<12>(N->getZExtValue()),
                                   SDLoc(N), N->getValueType(0));
}]>;

// Transformation Function - get the higher 20 bits.
def HI20 : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(((N->getZExtValue()+0x800) >> 12) & 0xfffff,
                                   SDLoc(N), N->getValueType(0));
}]>;

// Signed Operand
def simm12 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<12>(Imm);}]> {
  // let ParserMatchClass = SImmAsmOperand<12>;
  //   let EncoderMethod = "getImmOpValue";
  let DecoderMethod= "DecodeSimm12";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (MCOp.evaluateAsConstantImm(Imm))
      return isInt<12>(Imm);
    return MCOp.isBareSymbolRef();
  }];
}
def simm20 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<20>(Imm);}]> {
  let DecoderMethod= "DecodeSimm20";
}
def simm32 : ImmLeaf<XLenVT, [{return isInt<32>(Imm);}]>;

def immSExt12 : PatLeaf<(imm), [{ return isInt<12>(N->getSExtValue()); }]>;
def immSExt20 : PatLeaf<(imm), [{ return isInt<20>(N->getSExtValue()); }]>;
// Immediate can be loaded with LUi (32-bit int with lower 16-bit cleared).
def immLow12Zero : ImmLeaf<XLenVT, [{return isShiftedInt<20, 12>(Imm);}]>;

// shamt field must fit in 5 bits.
def immZExt5 : ImmLeaf<XLenVT, [{return Imm == (Imm & 0x1f);}]>;
def shamt : Operand<i32>;

// def calltarget : Operand<iPTR> {
//   let EncoderMethod = "getJumpTargetOpValue";
// }

def calltarget : Operand<iPTR> {
  let EncoderMethod = "getJumpTargetOpValue";
}

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


// MYRISCVX Address Mode! SDNode frameindex could possibily be a match
// since load and store instructions from stack used it.
def addr_fi : ComplexPattern<iPTR, 1, "SelectAddrFI", [frameindex], []>;
def addr    : ComplexPattern<iPTR, 2, "SelectAddr"  , [frameindex]>;

def IsOrAdd: PatFrag<(ops node:$A, node:$B), (or node:$A, node:$B), [{
  return isOrEquivalentToAdd(N);
}]>;


//===----------------------------------------------------------------------===//
// Instructions specific format
//===----------------------------------------------------------------------===//

// Arithmetic and logical instructions with 2 register operands.
class ArithLogicR<bits<7> opcode, bits<3> funct3, bits<7>funct7,
                  string instr_asm, SDNode OpNode,
                  RegisterClass RC> :
  MYRISCVX_R<opcode, funct3, funct7, (outs RC:$rd), (ins RC:$rs1, RC:$rs2),
  !strconcat(instr_asm, "\t$rd, $rs1, $rs2"),
  [(set RC:$rd, (OpNode RC:$rs1, RC:$rs2))], IIAlu> {
    let isReMaterializable = 1;
}

// Hi and Lo nodes are used to handle global addresses. Used on
// MYRISCVXISelLowering to lower stuff like GlobalAddress, ExternalSymbol
// static model. (nothing to do with MYRISCVX Registers Hi and Lo)
def MYRISCVXHi    : SDNode<"MYRISCVXISD::Hi", SDTIntUnaryOp>;
def MYRISCVXLo    : SDNode<"MYRISCVXISD::Lo", SDTIntUnaryOp>;
def MYRISCVXGPRel : SDNode<"MYRISCVXISD::GPRel", SDTIntUnaryOp>;

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

// Shifts
class ShiftRotateI<bits<7> opcode, bits<3> funct3, bit arithshift, string instr_asm,
                   SDNode OpNode, PatFrag PF, Operand ImmOpnd,
                   RegisterClass RC>:
  MYRISCVX_ISHIFT<opcode, funct3, arithshift, (outs RC:$rd), (ins RC:$rs1, ImmOpnd:$shamt),
                  !strconcat(instr_asm, "\t$rd, $rs1, $shamt"),
                  [(set GPR:$rd, (OpNode RC:$rs1, PF:$shamt))], IIAlu> {
}


// 32-bit shift instructions.
class ShiftRotateI32<bits<7> opcode, bits<3> funct3, bit arithshift, string instr_asm,
                     SDNode OpNode>:
  ShiftRotateI<opcode, funct3, arithshift, instr_asm, OpNode, immZExt5, shamt, GPR>;


class ShiftRotateR<bits<7> opcode, bits<7> funct7, bits<3> funct3,
                   bits<4> isRotate, string instr_asm,
                   SDNode OpNode, RegisterClass RC>:
  MYRISCVX_R<opcode, funct3, funct7, (outs RC:$rd), (ins RC:$rs1, RC:$rs2),
          !strconcat(instr_asm, "\t$rd, $rs1, $rs2"),
          [(set GPR:$rd, (OpNode RC:$rs1, RC:$rs2))], IIAlu> {
}


// Memory Load/Store
let canFoldAsLoad = 1 in
class LoadMemory<bits<7> opcode, bits<3> funct3, string instr_asm, RegisterClass RC, PatFrag OpNode,
                 bit Pseudo>:
  MYRISCVX_I<opcode, funct3, (outs RC:$rd), (ins GPR:$rs1, simm12:$simm12),
     !strconcat(instr_asm, "\t$rd, ${simm12}(${rs1})"),
     [], IILoad> {
  let isPseudo = Pseudo;
}

class StoreMemory<bits<7> opcode, bits<3> funct3, string instr_asm, RegisterClass RC, PatFrag OpNode,
                  bit Pseudo>:
  MYRISCVX_S<opcode, funct3, (outs), (ins RC:$rs2, GPR:$rs1, simm12:$simm12),
     !strconcat(instr_asm, "\t$rs2, ${simm12}(${rs1})"),
     [], IIStore> {
  let isPseudo = Pseudo;
}

// Branch instructions with 2 register operands.
class CBranch12<bits<7> opcode, bits<3> funct3,
                string instr_asm,
                PatFrag cond_op, RegisterClass RC> :
  MYRISCVX_B<opcode, funct3, (outs), (ins RC:$rs1, RC:$rs2, brtarget12:$imm12),
  !strconcat(instr_asm, "\t$rs1, $rs2, $imm12"),
  [], IIAlu> {
    let isBranch = 1;
    let isTerminator = 1;
}


// Jump and Link (Call)
let isCall = 1 in
class JumpLink<bits<7> opcode, string opstr, DAGOperand opnd> :
  MYRISCVX_J<opcode, (outs GPR:$rd), (ins opnd:$imm20), !strconcat(opstr, "\t$rd, $imm20"),
                 [], IIAlu> {
  let DecoderMethod = "DecodeJumpTarget";
}


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
def sextloadi32_a   : AlignedLoad<sextloadi32>;
def zextloadi32_a   : AlignedLoad<zextloadi32>;
def extloadi32_a    : AlignedLoad<extloadi32>;

def store_a         : AlignedStore<store>;
def truncstorei8_a  : AlignedStore<truncstorei8>;
def truncstorei16_a : AlignedStore<truncstorei16>;
def truncstorei32_a : AlignedStore<truncstorei32>;

def truncstoref32_a : AlignedStore<truncstoref32>;
def truncstoref64_a : AlignedStore<truncstoref64>;


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
def XORI : ArithLogicI<0b0010011, 0b100, "xori", xor, simm12, immSExt12, GPR>;
def ORI  : ArithLogicI<0b0010011, 0b110, "ori",  or,  simm12, immSExt12, GPR>;
def ANDI : ArithLogicI<0b0010011, 0b111, "andi", and, simm12, immSExt12, GPR>;

def LUI  : ArithLogicU<0b0110111, "lui", GPR, simm20, immSExt20>;
def ADD  : ArithLogicR<0b0110011, 0b000, 0b0000000, "add", add, GPR>;
def SUB  : ArithLogicR<0b0110011, 0b000, 0b0100000, "sub", sub, GPR>;

def SLL  : ShiftRotateR<0b0110011, 0b0000000, 0b001, 0x0, "sll", shl, GPR>;
def AND  : ArithLogicR     <0b0110011, 0b111, 0b0000000, "and", and, GPR>;
def SRL  : ShiftRotateR<0b0110011, 0b0000000, 0b101, 0x0, "srl", srl, GPR>;
def SRA  : ShiftRotateR<0b0110011, 0b0100000, 0b101, 0x0, "sra", sra, GPR>;
def OR   : ArithLogicR     <0b0110011, 0b110, 0b0000000, "or",  or,  GPR>;
def XOR  : ArithLogicR     <0b0110011, 0b100, 0b0000000, "xor", xor, GPR>;

def SRLI : ShiftRotateI32 <0b0010011, 0b101, 0, "srli", srl>;
def SLLI : ShiftRotateI32 <0b0010011, 0b001, 0, "slli", shl>;
def SRAI : ShiftRotateI32 <0b0010011, 0b101, 1, "srai", sra>;

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

def JALR : ArithLogicI<0b1100111, 0b000, "jalr", add, simm12, immSExt12, GPR>;
def : InstAlias<"jr $rs",      (JALR ZERO, GPR:$rs, 0)>;
def : InstAlias<"jalr $rs",    (JALR RA,   GPR:$rs, 0)>;
def : InstAlias<"ret",         (JALR ZERO,      RA, 0), 2>;

def JAL : JumpLink<0b1101111, "jal", brtarget20>;
def : InstAlias<"j $offset",   (JAL  ZERO, brtarget20:$offset)>;
def : InstAlias<"jal $offset", (JAL  RA,   brtarget20:$offset)>;

/// Load and Store Instructions
///  aligned
def LW  : LoadMemory <0b0000011, 0b010, "lw",  GPR, sextloadi32_a  , 0>;
def SW  : StoreMemory<0b0100011, 0b010, "sw",  GPR, truncstorei32_a, 0>;
def LB  : LoadMemory <0b0000011, 0b000, "lb",  GPR, sextloadi8_a   , 0>;
def LBU : LoadMemory <0b0000011, 0b100, "lbu", GPR, zextloadi8_a   , 0>;
def SB  : StoreMemory<0b0100011, 0b000, "sb",  GPR, truncstorei8_a , 0>;
def LH  : LoadMemory <0b0000011, 0b001, "lh",  GPR, sextloadi16_a  , 0>;
def LHU : LoadMemory <0b0000011, 0b101, "lhu", GPR, zextloadi16_a  , 0>;
def SH  : StoreMemory<0b0100011, 0b001, "sh",  GPR, truncstorei16_a, 0>;

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
                         (XLenVT imm:$imm),
                         RC:$truev,
                         RC:$falsev))]>;

def Select_GPR_Using_CC_GPR : SelectCC_rrirr<GPR, GPR>;

let Defs = [SP], Uses = [SP], hasSideEffects = 1 in {
  // As stack alignment is always done with addi, we need a 16-bit immediate
  def ADJCALLSTACKDOWN : MYRISCVXPseudo<(outs), (ins simm12:$amt1, simm12:$amt2), "",
                                        [(MYRISCVXCallSeqStart timm:$amt1, timm:$amt2)]>;
  def ADJCALLSTACKUP   : MYRISCVXPseudo<(outs), (ins simm12:$amt1, simm12:$amt2), "",
                                        [(MYRISCVXCallSeqEnd timm:$amt1, timm:$amt2)]>;
}


//}
//@def CPRESTORE {
// When handling PIC code the assembler needs .cpload and .cprestore
// directives. If the real instructions corresponding these directives
// are used, we have the same behavior, but get also a bunch of warnings
// from the assembler.
let hasSideEffects = 0 in
def CPRESTORE : MYRISCVXPseudo<(outs), (ins simm12:$loc, GPR:$gp),
                            ".cprestore\t$loc", []>;

//===----------------------------------------------------------------------===//
//  Arbitrary patterns that map to one or more instructions
//===----------------------------------------------------------------------===//

let isReturn=1, isTerminator=1, hasDelaySlot=0, isBarrier=1, hasCtrlDep=1 in
  def RetRA : MYRISCVXPseudo<(outs), (ins), "", [(MYRISCVXRet)]>;

// Small immediates
def : Pat<(immSExt12:$in), (ADDI ZERO, imm:$in)>;
def : Pat<(immLow12Zero:$in), (LUI (HI20 imm:$in))>;
def : Pat<(simm32:$in), (ORI (LUI (HI20 imm:$in)), (LO12 imm:$in))>,
      Requires<[IsRV32]>;

// Addr for Frame
def : Pat<(add (i32 addr_fi:$rs), simm12:$simm12),
          (ADDI (i32 addr_fi:$rs), simm12:$simm12)>;
def : Pat<(IsOrAdd (i32 addr_fi:$rs), simm12:$simm12),
          (ADDI (i32 addr_fi:$rs), simm12:$simm12)>;

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
def : WrapperPat<texternalsym, ORI, GPR>;


// Memory Acccess
multiclass LoadPattern<PatFrag LoadOp, MYRISCVXInst Inst> {
  def : Pat<(LoadOp GPR:$rs1)                              , (Inst GPR:$rs1, 0)           >;
  def : Pat<(LoadOp addr_fi:$rs1)                          , (Inst addr_fi:$rs1, 0)       >;
  def : Pat<(LoadOp (add GPR:$rs1, simm12:$simm12))        , (Inst GPR:$rs1, $simm12)     >;
  def : Pat<(LoadOp (add addr_fi:$rs1, simm12:$simm12))    , (Inst addr_fi:$rs1, $simm12) >;
  def : Pat<(LoadOp (IsOrAdd addr_fi:$rs1, simm12:$simm12)), (Inst addr_fi:$rs1, $simm12) >;
}

multiclass StorePattern<PatFrag StoreOp, MYRISCVXInst Inst> {
  def : Pat<(StoreOp GPR:$rs2, GPR:$rs1)                              , (Inst GPR:$rs2, GPR:$rs1, 0)                         >;
  def : Pat<(StoreOp GPR:$rs2, addr_fi:$rs1)                          , (Inst GPR:$rs2, addr_fi:$rs1, 0)                     >;
  def : Pat<(StoreOp GPR:$rs2, (add GPR:$rs1, simm12:$simm12))        , (Inst GPR:$rs2, GPR:$rs1, simm12:$simm12)            >;
  def : Pat<(StoreOp GPR:$rs2, (add addr_fi:$rs1, simm12:$simm12))    , (Inst GPR:$rs2, addr_fi:$rs1, simm12:$simm12)        >;
  def : Pat<(StoreOp GPR:$rs2, (IsOrAdd addr_fi:$rs1, simm12:$simm12)), (Inst GPR:$rs2, addr_fi:$rs1, simm12:$simm12)        >;
}

defm : LoadPattern<sextloadi8  , LB>;
defm : LoadPattern<extloadi8   , LB>;
defm : LoadPattern<sextloadi16 , LH>;
defm : LoadPattern<extloadi16  , LH>;
defm : LoadPattern<load        , LW>, Requires<[IsRV32]>;
defm : LoadPattern<zextloadi8  , LBU>;
defm : LoadPattern<zextloadi16 , LHU>;

defm : StorePattern<truncstorei8  , SB>;
defm : StorePattern<truncstorei16 , SH>;
defm : StorePattern<store         , SW>, Requires<[IsRV32]>;


let Predicates = [IsRV64] in {
  def LD  : LoadMemory <0b0000011, 0b011, "ld",  GPR, load_a         , 0>;
  def SD  : StoreMemory<0b0100011, 0b011, "sd",  GPR, store_a        , 0>;
  def LWU : LoadMemory <0b0000011, 0b110, "lwu",  GPR, sextloadi32_a  , 0>;

  /// Loads
  defm : LoadPattern<sextloadi32 , LW>;
  defm : LoadPattern<extloadi32  , LW>;
  defm : LoadPattern<zextloadi32 , LWU>;
  defm : LoadPattern<load        , LD>;

  /// Stores
  defm : StorePattern<truncstorei32 , SW>;
  defm : StorePattern<store         , SD>;

  def ADDIW : ArithLogicI<0b0011011, 0b000, "addiw", add, simm12, immSExt12, GPR>;

  /// sext
  def : Pat<(sext_inreg GPR:$rs1, i32), (ADDIW GPR:$rs1, 0)>;

} // Predicates = [IsRV64]

let isBarrier = 1, isBranch = 1, isTerminator = 1 in
  def PseudoBR : MYRISCVXPseudo<(outs), (ins brtarget20:$simm20), "", [(br bb:$simm20)]>,
                 PseudoInstExpansion<(JAL ZERO, brtarget20:$simm20)>;

let isBarrier = 1, isBranch = 1, isIndirectBranch = 1, isTerminator = 1 in
def PseudoBRIND : MYRISCVXPseudo<(outs), (ins GPR:$rs1, simm12:$simm12), "", []>,
                  PseudoInstExpansion<(JALR ZERO, GPR:$rs1, simm12:$simm12)>;

def : Pat<(brind GPR:$rs1), (PseudoBRIND GPR:$rs1, 0)>;
def : Pat<(brind (add GPR:$rs1, simm12:$simm12)),
          (PseudoBRIND GPR:$rs1, simm12:$simm12)>;

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
  def : Pat<(brcond (XLenVT (setne RC:$lhs, 0)), bb:$dst),
                (BNEOp RC:$lhs, ZEROReg, bb:$dst)>;
  def : Pat<(brcond (XLenVT (seteq RC:$lhs, 0)), bb:$dst),
                (BEQOp RC:$lhs, ZEROReg, bb:$dst)>;

  def : Pat<(brcond (XLenVT (seteq RC:$lhs, RC:$rhs)), bb:$dst),
                (BEQOp RC:$lhs, RC:$rhs, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setueq RC:$lhs, RC:$rhs)), bb:$dst),
                (BEQOp RC:$lhs, RC:$rhs, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setne RC:$lhs, RC:$rhs)), bb:$dst),
                (BNEOp RC:$lhs, RC:$rhs, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setune RC:$lhs, RC:$rhs)), bb:$dst),
                (BNEOp RC:$lhs, RC:$rhs, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setlt RC:$lhs, RC:$rhs)), bb:$dst),
                (BNE (SLTOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setult RC:$lhs, RC:$rhs)), bb:$dst),
                (BNE (SLTuOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setgt RC:$lhs, RC:$rhs)), bb:$dst),
                (BNE (SLTOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setugt RC:$lhs, RC:$rhs)), bb:$dst),
                (BNE (SLTuOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setle RC:$lhs, RC:$rhs)), bb:$dst),
                (BEQ (SLTOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setule RC:$lhs, RC:$rhs)), bb:$dst),
                (BEQ (SLTuOp RC:$rhs, RC:$lhs), ZERO, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setge RC:$lhs, RC:$rhs)), bb:$dst),
                (BEQ (SLTOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
  def : Pat<(brcond (XLenVT (setuge RC:$lhs, RC:$rhs)), bb:$dst),
                (BEQ (SLTuOp RC:$lhs, RC:$rhs), ZERO, bb:$dst)>;
  def : Pat<(brcond RC:$cond, bb:$dst),
                (BNEOp RC:$cond, ZEROReg, bb:$dst)>;
}

defm : BrcondPatsSlt<GPR, BEQ, BNE, SLT, SLTU, SLTI, SLTIU, ZERO>;

let isCall = 1, Defs = [RA], isCodeGenOnly = 0 in
def PseudoCALL : MYRISCVXPseudo<(outs), (ins GPR:$rs1), "", [(MYRISCVXCall GPR:$rs1)]>,
                 PseudoInstExpansion<(JALR RA, GPR:$rs1, 0)>;


// def : Pat<(MYRISCVXCall (i32 tglobaladdr:$dst)),  (JAL tglobaladdr:$dst)>;
def : Pat<(MYRISCVXCall texternalsym:$func), (PseudoCALL texternalsym:$func)>;
// def : Pat<(MYRISCVXCall (XLenVT tglobaladdr:$dst)),  (JALR RA, tglobaladdr:$dst)>;
// def : Pat<(MYRISCVXCall (XLenVT texternalsym:$dst)), (JALR RA, texternalsym:$dst)>;
