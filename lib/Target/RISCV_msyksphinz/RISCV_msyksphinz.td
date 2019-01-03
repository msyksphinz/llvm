//===-- RISCV_msyksphinz.td - Describe the RISCV_msyksphinz Target Machine ---------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// This is the top level entry point for the RISCV_msyksphinz target.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//


include "llvm/Target/Target.td"

include "RISCVRegisterInfo.td"

include "RISCVInstrInfo.td"

def RISCV_msyksphinzInstrInfo : InstrInfo;


// Will generate RISCV_msyksphinzGenAsmWrite.inc included by RISCV_msyksphinzInstPrinter.cpp, contents
//  as follows,
// void RISCV_msyksphinzInstPrinter::printInstruction(const MCInst *MI, raw_ostream &O) {...}
// const char *RISCV_msyksphinzInstPrinter::getRegisterName(unsigned RegNo) {...}
def RISCV_msyksphinz : Target {
// def RISCV_msyksphinzInstrInfo : InstrInfo as before.
  let InstructionSet = RISCV_msyksphinzInstrInfo;
}
