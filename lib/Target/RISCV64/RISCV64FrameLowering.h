//===-- RISCV64FrameLowering.h - Define frame lowering for RISCV64 ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MIPS_MIPSFRAMELOWERING_H
#define LLVM_LIB_TARGET_MIPS_MIPSFRAMELOWERING_H

#include "RISCV64.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {
  class RISCV64Subtarget;

class RISCV64FrameLowering : public TargetFrameLowering {
protected:
  const RISCV64Subtarget &STI;

public:
  explicit RISCV64FrameLowering(const RISCV64Subtarget &sti, unsigned Alignment)
    : TargetFrameLowering(StackGrowsDown, Alignment, 0, Alignment), STI(sti) {}

  static const RISCV64FrameLowering *create(const RISCV64Subtarget &ST);

  bool hasFP(const MachineFunction &MF) const override;

  bool hasBP(const MachineFunction &MF) const;

  bool isFPCloseToIncomingSP() const override { return false; }

  bool enableShrinkWrapping(const MachineFunction &MF) const override {
    return true;
  }

  MachineBasicBlock::iterator
  eliminateCallFramePseudoInstr(MachineFunction &MF,
                                MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I) const override;

protected:
  uint64_t estimateStackSize(const MachineFunction &MF) const;
};

/// Create RISCV64FrameLowering objects.
const RISCV64FrameLowering *createRISCV6416FrameLowering(const RISCV64Subtarget &ST);
const RISCV64FrameLowering *createRISCV64SEFrameLowering(const RISCV64Subtarget &ST);

} // End llvm namespace

#endif
