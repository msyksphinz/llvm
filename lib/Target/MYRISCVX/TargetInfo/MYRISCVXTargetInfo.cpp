//===-- MYRISCVXTargetInfo.cpp - MYRISCVX Target Implementation -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVX.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

namespace llvm {
Target &getTheMYRISCVX32Target() {
  static Target TheMYRISCVX32Target;
  return TheMYRISCVX32Target;
}

Target &getTheMYRISCVX64Target() {
  static Target TheMYRISCVX64Target;
  return TheMYRISCVX64Target;
}
}

extern "C" void LLVMInitializeMYRISCVXTargetInfo() {
  RegisterTarget<Triple::myriscvx32,
        /*HasJIT=*/true>
      X(getTheMYRISCVX32Target(), "myriscvx32", "MYRISCVX (32-bit)", "MYRISCVX");

  RegisterTarget<Triple::myriscvx64,
        /*HasJIT=*/true>
      Y(getTheMYRISCVX64Target(), "myriscvx64", "MYRISCVX (64-bit)", "MYRISCVX");
}
