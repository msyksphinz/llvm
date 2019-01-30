//===-- MYRISCVXISEISelLowering.h - MYRISCVXISE DAG Lowering Interface ----*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of MYRISCVXITargetLowering specialized for MYRISCVX32/64.
//
//===----------------------------------------------------------------------===//

#pragma once

#include "MYRISCVXISelLowering.h"

#include "MYRISCVXRegisterInfo.h"
namespace llvm {
  class MYRISCVXSETargetLowering : public MYRISCVXTargetLowering {
 public:
    explicit MYRISCVXSETargetLowering(const MYRISCVXTargetMachine &TM,
                                      const MYRISCVXSubtarget &STI);
    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;
 private:
  };
}
