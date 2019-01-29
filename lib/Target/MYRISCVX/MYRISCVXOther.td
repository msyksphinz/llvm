//===-- MYRISCVXOther.td - Describe the MYRISCVX Target Machine ----*- tablegen -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//

//===----------------------------------------------------------------------===//
// This is the top level entry point for the MYRISCVX target.
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Target-dependent interfaces
//===----------------------------------------------------------------------===//

// def XLenVT : ValueTypeByHwMode<[RV32, RV64, DefaultMode],
//                                [i32,  i64,  i32]>;

def RV64           : HwMode<"+64bit">;
def RV32           : HwMode<"-64bit">;

def FeatureMYRISCVX       : SubtargetFeature<"MYRISCVX", "MYRISCVXArchVersion", "MYRISCVX",
                                             "MYRISCVX Support [highly experimental]">;

class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, NoItineraries, Features>;

def : Proc<"MYRISCVX", [FeatureMYRISCVX]>;

include "MYRISCVXRegisterInfo.td"
// include "MYRISCVXRegisterInfoGPROutForOther.td" // except AsmParser
include "MYRISCVX.td"
