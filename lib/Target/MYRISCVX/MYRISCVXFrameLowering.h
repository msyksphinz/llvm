//===-- MYRISCVXFrameLowering.h - Define frame lowering for MYRISCVX -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===---------------------------------------------------------------------------===//
//
//
//
//===---------------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXFRAMELOWERING_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXFRAMELOWERING_H

#include "MYRISCVX.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {
  class MYRISCVXSubtarget;

  class MYRISCVXFrameLowering : public TargetFrameLowering {
 protected:
    const MYRISCVXSubtarget &STI;

 public:
    explicit MYRISCVXFrameLowering(const MYRISCVXSubtarget &sti, unsigned Alignment)
        : TargetFrameLowering(StackGrowsDown, Alignment, 0, Alignment),
        STI(sti) {
    }

    static const MYRISCVXFrameLowering *create(const MYRISCVXSubtarget &ST);

    bool hasFP(const MachineFunction &MF) const override;

    MachineBasicBlock::iterator
    eliminateCallFramePseudoInstr(MachineFunction &MF,
                                  MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I) const override;

  };

  /// Create MYRISCVXFrameLowering objects.
  const MYRISCVXFrameLowering *createMYRISCVXSEFrameLowering(const MYRISCVXSubtarget &ST);

} // End llvm namespace

#endif
