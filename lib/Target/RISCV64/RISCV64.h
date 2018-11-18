//===-- RISCV64.h - Top-level interface for RISCV64 representation ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in
// the LLVM RISCV64 back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MIPS_MIPS_H
#define LLVM_LIB_TARGET_MIPS_MIPS_H

#include "MCTargetDesc/RISCV64MCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class RISCV64TargetMachine;
  class ModulePass;
  class FunctionPass;
  class RISCV64RegisterBankInfo;
  class RISCV64Subtarget;
  class RISCV64TargetMachine;
  class InstructionSelector;
  class PassRegistry;

  ModulePass *createRISCV64Os16Pass();
  ModulePass *createRISCV6416HardFloatPass();

  FunctionPass *createRISCV64ModuleISelDagPass();
  FunctionPass *createRISCV64OptimizePICCallPass();
  FunctionPass *createRISCV64DelaySlotFillerPass();
  FunctionPass *createRISCV64BranchExpansion();
  FunctionPass *createRISCV64ConstantIslandPass();
  FunctionPass *createMicroRISCV64SizeReducePass();
  FunctionPass *createRISCV64ExpandPseudoPass();

  InstructionSelector *createRISCV64InstructionSelector(const RISCV64TargetMachine &,
                                                     RISCV64Subtarget &,
                                                     RISCV64RegisterBankInfo &);

  void initializeRISCV64DelaySlotFillerPass(PassRegistry &);
  void initializeRISCV64BranchExpansionPass(PassRegistry &);
  void initializeMicroRISCV64SizeReducePass(PassRegistry &);
} // end namespace llvm;

#endif
