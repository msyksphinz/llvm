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

// Signed Operand
def simm12 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<5>(Imm);}]> {
  let DecoderMethod= "DecodeSimm12";
}
def simm20 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<5>(Imm);}]> {
  let DecoderMethod= "DecodeSimm20";
}
// Unsigned Operand
def uimm12 : Operand<XLenVT> {
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
def immZExt5 : ImmLeaf<XLenVT, [{return Imm == (Imm & 0x1f);}]>;

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

def JALR : ArithLogicI<0b1100111, 0b000, "jalr", add, simm12, immSExt12, GPR>;
def: InstAlias<"jr $rs",      (JALR ZERO, GPR:$rs, 0)>;
def: InstAlias<"jalr $rs",    (JALR RA,   GPR:$rs, 0)>;
def: InstAlias<"ret",         (JALR ZERO,      RA, 0), 2>;


//===----------------------------------------------------------------------===//
//  Arbitrary patterns that map to one or more instructions
//===----------------------------------------------------------------------===//

let isReturn=1, isTerminator=1, hasDelaySlot=0, isBarrier=1, hasCtrlDep=1 in
  def RetRA : MYRISCVXPseudo<(outs), (ins), "", [(MYRISCVXRet)]>;

// Small immediates
def : Pat<(XLenVT immSExt12:$in),
          (ADDI ZERO, imm:$in)>;
def : Pat<(XLenVT immZExt12:$in),
          (ORI ZERO, imm:$in)>;

def : Pat<(XLenVT immLow12Zero:$in),
          (LUI (HI20 imm:$in))>;

def : Pat<(XLenVT imm:$imm),
          (ORI (LUI (HI20 imm:$imm)), (LO12 imm:$imm))>;
