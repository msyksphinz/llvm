//===-- MYRISCVX.td - Describe the MYRISCVX Target Machine ---------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// This is the top level entry point for the MYRISCVX target.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//


include "llvm/Target/Target.td"

include "MYRISCVXRegisterInfo.td"

include "MYRISCVXInstrInfo.td"
include "MYRISCVXCallingConv.td"

def FeatureMYRISCVX64 : SubtargetFeature<"myriscvx64", "HasArchX64", "true", "MYRISCVX64 ISA Support">;

def FeatureMYRISCVX64_CPU : SubtargetFeature<"myriscvx64_impl", "MYRISCVXArchVersion",
                                "MYRISCVX64", "MYRISCVX64 ISA Support",
                                [FeatureMYRISCVX64]>;

def : ProcessorModel<"myriscvx64_impl", NoSchedModel, [FeatureMYRISCVX64_CPU]>;

def MYRISCVXInstrInfo : InstrInfo;

// Will generate MYRISCVXGenAsmWrite.inc included by MYRISCVXInstPrinter.cpp, contents
//  as follows,
// void MYRISCVXInstPrinter::printInstruction(const MCInst *MI, raw_ostream &O) {...}
// const char *MYRISCVXInstPrinter::getRegisterName(unsigned RegNo) {...}

def MYRISCVX : Target {
  let InstructionSet = MYRISCVXInstrInfo;
}
