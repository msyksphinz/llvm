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
// MYRISCVX subtarget features and instruction predicates.
//===----------------------------------------------------------------------===//

def FeatureMExt : SubtargetFeature<"m", "is_enable_M", "true",
    "'M' (Integer Multiplication and Division) support">;

def FeatureAExt : SubtargetFeature<"a", "is_enable_A", "true",
    "'A' (Atomic) support">;

def FeatureFExt : SubtargetFeature<"f", "is_enable_F", "true",
    "'F' (Singple Precision Floating Point) support">;

def FeatureDExt : SubtargetFeature<"d", "is_enable_D", "true",
    "'D' (Double Precision Floating Point) support">;

def FeatureCExt : SubtargetFeature<"c", "is_enable_C", "true",
    "'C' (Compressed Instruction) support">;

def FeatureRV64 : SubtargetFeature<"64bit", "HasRV64", "true", "RV64 support">;

def : ProcessorModel<"simple32", NoSchedModel, []>;
def : ProcessorModel<"rocket64", NoSchedModel, [FeatureRV64,
                                                FeatureMExt,
                                                FeatureAExt, FeatureFExt, FeatureDExt, FeatureCExt]>;

//===----------------------------------------------------------------------===//
// Target-dependent interfaces
//===----------------------------------------------------------------------===//

include "MYRISCVXRegisterInfo.td"
include "MYRISCVXSchedule.td"
include "MYRISCVXInstrInfo.td"
include "MYRISCVXCallingConv.td"

def MYRISCVXInstrInfo : InstrInfo;

// Will generate MYRISCVXGenAsmWrite.inc included by MYRISCVXInstPrinter.cpp, contents
//  as follows,
// void MYRISCVXInstPrinter::printInstruction(const MCInst *MI, raw_ostream &O) {...}
// const char *MYRISCVXInstPrinter::getRegisterName(unsigned RegNo) {...}
def MYRISCVX : Target {
// def MYRISCVXInstrInfo : InstrInfo as before.
  let InstructionSet = MYRISCVXInstrInfo;
}

def MYRISCVXAsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let AllowDuplicateRegisterNames = 1;
}
