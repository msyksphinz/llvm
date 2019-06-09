//===-- MYRISCVXInstPrinter.cpp - Convert MYRISCVX MCInst to assembly syntax ------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===------------------------------------------------------------------------------===//
//
// This class prints an MYRISCVX MCInst to a .s file.
//
//===------------------------------------------------------------------------------===//

#include "MYRISCVXInstPrinter.h"
#include "MCTargetDesc/MYRISCVXMCExpr.h"

#include "MYRISCVXInstrInfo.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"

#define PRINT_ALIAS_INSTR
#include "MYRISCVXGenAsmWriter.inc"

void MYRISCVXInstPrinter::printRegName(raw_ostream &OS, unsigned RegNo) const {
  //- getRegisterName(RegNo) defined in MYRISCVXGenAsmWriter.inc which indicate in
  //   MYRISCVX.td.
  OS << StringRef(getRegisterName(RegNo)).lower();
}

//@1 {
void MYRISCVXInstPrinter::printInst(const MCInst *MI, raw_ostream &O,
                                    StringRef Annot, const MCSubtargetInfo &STI) {
  // Try to print any aliases first.
  if (!printAliasInstr(MI, O))
    //@1 }
    //- printInstruction(MI, O) defined in MYRISCVXGenAsmWriter.inc which came from
    //   MYRISCVX.td indicate.
    printInstruction(MI, O);
  printAnnotation(O, Annot);
}

void MYRISCVXInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                       raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isReg()) {
    printRegName(O, Op.getReg());
    return;
  }

  if (Op.isImm()) {
    O << Op.getImm();
    return;
  }

  assert(Op.isExpr() && "unknown operand kind in printOperand");
  Op.getExpr()->print(O, &MAI, true);
}

void MYRISCVXInstPrinter::printUnsignedImm(const MCInst *MI, int opNum,
                                           raw_ostream &O) {
  const MCOperand &MO = MI->getOperand(opNum);
  if (MO.isImm())
    O << (unsigned short int)MO.getImm();
  else
    printOperand(MI, opNum, O);
}

void MYRISCVXInstPrinter::
printMemOperand(const MCInst *MI, int opNum, raw_ostream &O) {
  // Load/Store memory operands -- imm($reg)
  // If PIC target the target is loaded as the
  // pattern ld $t9,%call16($gp)
  printOperand(MI, opNum+1, O);
  O << "(";
  printOperand(MI, opNum, O);
  O << ")";
}

// The DAG data node, mem_ea of MYRISCVXInstrInfo.td, cannot be disabled by
// ch7_1, only opcode node can be disabled.
void MYRISCVXInstPrinter::
printMemOperandEA(const MCInst *MI, int opNum, raw_ostream &O) {
  // when using stack locations for not load/store instructions
  // print the same way as all normal 3 operand instructions.
  printOperand(MI, opNum, O);
  O << ", ";
  printOperand(MI, opNum+1, O);
  return;
}
