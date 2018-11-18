//===-- RISCV64TargetInfo.cpp - RISCV64 Target Implementation -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "RISCV64.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target &llvm::getTheRISCV64Target() {
  static Target TheRISCV64Target;
  return TheRISCV64Target;
}
Target &llvm::getTheRISCV64elTarget() {
  static Target TheRISCV64elTarget;
  return TheRISCV64elTarget;
}
Target &llvm::getTheRISCV6464Target() {
  static Target TheRISCV6464Target;
  return TheRISCV6464Target;
}
Target &llvm::getTheRISCV6464elTarget() {
  static Target TheRISCV6464elTarget;
  return TheRISCV6464elTarget;
}

extern "C" void LLVMInitializeRISCV64TargetInfo() {
  RegisterTarget<Triple::mips,
                 /*HasJIT=*/true>
      X(getTheRISCV64Target(), "mips", "MIPS (32-bit big endian)", "RISCV64");

  RegisterTarget<Triple::mipsel,
                 /*HasJIT=*/true>
      Y(getTheRISCV64elTarget(), "mipsel", "MIPS (32-bit little endian)", "RISCV64");

  RegisterTarget<Triple::mips64,
                 /*HasJIT=*/true>
      A(getTheRISCV6464Target(), "mips64", "MIPS (64-bit big endian)", "RISCV64");

  RegisterTarget<Triple::mips64el,
                 /*HasJIT=*/true>
      B(getTheRISCV6464elTarget(), "mips64el", "MIPS (64-bit little endian)",
        "RISCV64");
}
