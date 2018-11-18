//===-- RISCV64SERegisterInfo.h - RISCV6432/64 Register Information ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RISCV6432/64 implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MIPS_MIPSSEREGISTERINFO_H
#define LLVM_LIB_TARGET_MIPS_MIPSSEREGISTERINFO_H

#include "RISCV64RegisterInfo.h"

namespace llvm {
class RISCV64SEInstrInfo;

class RISCV64SERegisterInfo : public RISCV64RegisterInfo {
public:
  RISCV64SERegisterInfo();

  bool requiresRegisterScavenging(const MachineFunction &MF) const override;

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override;

  const TargetRegisterClass *intRegClass(unsigned Size) const override;

private:
  void eliminateFI(MachineBasicBlock::iterator II, unsigned OpNo,
                   int FrameIndex, uint64_t StackSize,
                   int64_t SPOffset) const override;
};

} // end namespace llvm

#endif
