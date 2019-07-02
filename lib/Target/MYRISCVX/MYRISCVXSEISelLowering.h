//===-- MYRISCVXISEISelLowering.h - MYRISCVXISE DAG Lowering Interface ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of MYRISCVXITargetLowering specialized for MYRISCVX32/64.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSEISELLOWERING_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSEISELLOWERING_H

#include "MYRISCVXISelLowering.h"
#include "MYRISCVXRegisterInfo.h"

namespace llvm {
  class MYRISCVXSETargetLowering : public MYRISCVXTargetLowering  {
 public:
    explicit MYRISCVXSETargetLowering(const MYRISCVXTargetMachine &TM,
                                      const MYRISCVXSubtarget &STI);

    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

    /// isEligibleForTailCallOptimization - Check whether the call is eligible
    /// for tail call optimization.
    bool
    isEligibleForTailCallOptimization(const CCState &CCInfo,
                                      unsigned NextStackOffset,
                                      const MYRISCVXFunctionInfo& FI) const override;

   private:
  };
}

#endif // MYRISCVXISEISELLOWERING_H
