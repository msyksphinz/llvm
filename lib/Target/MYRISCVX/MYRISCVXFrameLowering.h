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
    explicit MYRISCVXFrameLowering(const MYRISCVXSubtarget &sti)
        : TargetFrameLowering(StackGrowsDown,
                              /*StackAlignment=*/16,
                              /*LocalAreaOffset=*/0),
          STI(sti) {
    }

    static const MYRISCVXFrameLowering *create(const MYRISCVXSubtarget &ST);

    bool hasFP(const MachineFunction &MF) const override;

    /// emitProlog/emitEpilog - These methods insert prolog and epilog code into
    /// the function.
    void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
    void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  };

} // End llvm namespace

#endif
