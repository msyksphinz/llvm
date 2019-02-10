//===-- MYRISCVXSEFrameLowering.h - MYRISCVX32/64 frame lowering --------*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#pragma once

#include "MYRISCVXFrameLowering.h"

namespace llvm {
  class MYRISCVXSEFrameLowering : public MYRISCVXFrameLowering {
 public:
    explicit MYRISCVXSEFrameLowering(const MYRISCVXSubtarget &STI);

    /// emitProlog/emitEpilog - These methods insert prolog and epilog code into
    /// the function.
    void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
    void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;

    bool hasReservedCallFrame(const MachineFunction &MF) const;
  };

} // End llvm namespace
