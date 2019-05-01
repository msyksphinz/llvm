//===-- MYRISCVX.td - Describe the MYRISCVX Target Machine -*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This is the top level entry point for the MYRISCVX target.
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Target-dependent interfaces
//===----------------------------------------------------------------------===//

include "MYRISCVXRegisterInfo.td"
include "MYRISCVXSchedule.td"
include "MYRISCVXInstrInfo.td"

def MYRISCVXInstrInfo : InstrInfo;

// Will generate MYRISCVXGenAsmWrite.inc included by MYRISCVXInstPrinter.cpp, contents
//  as follows,
// void MYRISCVXInstPrinter::printInstruction(const MCInst *MI, raw_ostream &O) {...}
// const char *MYRISCVXInstPrinter::getRegisterName(unsigned RegNo) {...}
def MYRISCVX : Target {
// def MYRISCVXInstrInfo : InstrInfo as before.
  let InstructionSet = MYRISCVXInstrInfo;
}
