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
def load_a  : AlignedLoad<load>;
def store_a : AlignedStore<store>;

//===----------------------------------------------------------------------===//
// Instructions specific format
//===----------------------------------------------------------------------===//

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
     instr_asm, [(brind RC:$ra)], IIBranch> {
  let rd    = 0;  // Zero
  let rs1   = 1;  // RA
  let imm12 = 0;
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


//===----------------------------------------------------------------------===//
// Instruction definition
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// MYRISCVX Instructions
//===----------------------------------------------------------------------===//

/// Load and Store Instructions
/// aligned
defm LW : LoadM32 <0b0000011, 0b010, "lw", load_a >;
defm SW : StoreM32<0b0100011, 0b010, "sw", store_a>;

/// Arithmetic Instructions (ALU Immediate)
// IR "add" defined in include/llvm/Target/TargetSelectionDAG.td, line 315 (def add).
def ADDI : ArithLogicI<0b0010011, 0b000, "addi", add, simm12, immSExt12, GPR>;

/// Arithmetic Instructions (3-Operand, R-Type)
def JALR : JumpFR<0b1100111, 0b000, "jalr", GPR>;
// def : InstAlias<"jr $rs1", (JALR ZERO, GPR:$rs1, 0)>;
def RET  : RetBase<GPR>;

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
