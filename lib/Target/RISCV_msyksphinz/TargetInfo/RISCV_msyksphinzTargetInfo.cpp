//===-- RISCV_msyksphinzTargetInfo.cpp - RISCV_msyksphinz Target Implementation -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "RISCV_msyksphinz.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target llvm::TheRISCV_msyksphinzTarget, llvm::TheRISCV_msyksphinzelTarget;

extern "C" void LLVMInitializeRISCV_msyksphinzTargetInfo() {
  RegisterTarget<Triple::cpu0,
        /*HasJIT=*/true> X(TheRISCV_msyksphinzTarget, "cpu0", "RISCV_msyksphinz");

  RegisterTarget<Triple::cpu0el,
        /*HasJIT=*/true> Y(TheRISCV_msyksphinzelTarget, "cpu0el", "RISCV_msyksphinzel");
}
