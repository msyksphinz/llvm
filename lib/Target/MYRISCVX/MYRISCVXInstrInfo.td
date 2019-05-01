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

//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

include "MYRISCVXInstrFormats.td"

//===----------------------------------------------------------------------===//
// MYRISCVX Operand, Complex Patterns and Transformations Definitions.
//===----------------------------------------------------------------------===//

// Signed Operand
def simm12 : Operand<i32> {
  let DecoderMethod= "DecodeSimm12";
}
def simm20 : Operand<i32> {
  let DecoderMethod= "DecodeSimm20";
}

def immSExt12 : PatLeaf<(imm), [{ return isInt<12>(N->getSExtValue()); }]>;
def immSExt20 : PatLeaf<(imm), [{ return isInt<20>(N->getSExtValue()); }]>;

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

def LUI  : ArithLogicU<0b0110111, "lui", GPR, simm20, immSExt20>;
def ADD  : ArithLogicR<0b0110011, 0b000, 0b0000000, "add", add, GPR>;
def SUB  : ArithLogicR<0b0110011, 0b000, 0b0100000, "sub", sub, GPR>;

//===----------------------------------------------------------------------===//
//  Arbitrary patterns that map to one or more instructions
//===----------------------------------------------------------------------===//
