//===-- MYRISCVX.h - Top-level interface for MYRISCVX representation ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in
// the LLVM MYRISCVX back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVX_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVX_H

#include "MCTargetDesc/MYRISCVXMCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class MYRISCVXTargetMachine;
  class FunctionPass;

FunctionPass *createMYRISCVXDelJmpPass(MYRISCVXTargetMachine &TM);

} // end namespace llvm;

#endif
