//===-- MYRISCVXTargetInfo.cpp - MYRISCVX Target Implementation -------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#include "MYRISCVX.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target llvm::TheMYRISCVXTarget, llvm::TheMYRISCVXelTarget;

extern "C" void LLVMInitializeMYRISCVXTargetInfo() {
  RegisterTarget<Triple::myriscvx32,
                 /*HasJIT=*/true> X(TheMYRISCVXTarget, "myriscvx32",
                                    "32-bit MYRISCVX Target", "MYRISCVX");
  RegisterTarget<Triple::myriscvx64,
                 /*HasJIT=*/true> Y(TheMYRISCVXTarget, "myriscvx64",
                                    "64-bit MYRISCVX Target", "MYRISCVX");
  // RegisterTarget<Triple::MYRISCVXel,
  //                /*HasJIT=*/true> Y(TheMYRISCVXelTarget, "MYRISCVXel", "MYRISCVXel");
}
