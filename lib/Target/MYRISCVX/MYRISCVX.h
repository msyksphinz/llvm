//===-- MYRISCVX.h - Top-level interface for MYRISCVX representation ----*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
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

#define ENABLE_GPRESTORE

namespace llvm {
  class MYRISCVXTargetMachine;
  class AsmPrinter;
  class FunctionPass;
  class MCInst;
  class MCOperand;
  class MachineInstr;
  class MachineOperand;
  class PassRegistry;

  bool LowerMYRISCVXMachineOperandToMCOperand(const MachineOperand &MO,
                                            MCOperand &MCOp, const AsmPrinter &AP);

  FunctionPass *createMYRISCVXLongBranchPass(MYRISCVXTargetMachine &TM);
  FunctionPass *createMYRISCVXDelJmpPass(MYRISCVXTargetMachine &TM);

#ifdef ENABLE_GPRESTORE
  FunctionPass *createMYRISCVXEmitGPRestorePass(MYRISCVXTargetMachine &TM);
#endif

} // end namespace llvm;

#endif
