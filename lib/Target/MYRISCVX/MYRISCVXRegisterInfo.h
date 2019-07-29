//===-- MYRISCVXRegisterInfo.h - MYRISCVX Register Information Impl ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXREGISTERINFO_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXREGISTERINFO_H

#include "MYRISCVX.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "MYRISCVXGenRegisterInfo.inc"

namespace llvm {
  class MYRISCVXSubtarget;
  class TargetInstrInfo;
  class Type;

  class MYRISCVXRegisterInfo : public MYRISCVXGenRegisterInfo {
 protected:
    const MYRISCVXSubtarget &Subtarget;

 public:
    MYRISCVXRegisterInfo(const MYRISCVXSubtarget &Subtarget);

    /// Code Generation virtual methods...
    const TargetRegisterClass *getPointerRegClass(const MachineFunction &MF,
                                                  unsigned Kind) const override;

    const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;

    const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                         CallingConv::ID) const override;

    BitVector getReservedRegs(const MachineFunction &MF) const override;

    bool requiresRegisterScavenging(const MachineFunction &MF) const override;

    bool trackLivenessAfterRegAlloc(const MachineFunction &MF) const override;

    /// Stack Frame Processing Methods
    void eliminateFrameIndex(MachineBasicBlock::iterator II,
                             int SPAdj, unsigned FIOperandNum,
                             RegScavenger *RS = nullptr) const override;

    /// Debug information queries.
    unsigned getFrameRegister(const MachineFunction &MF) const override;

    /// \brief Return GPR register class.
    virtual const TargetRegisterClass *intRegClass(unsigned Size) const = 0;
  };

} // end namespace llvm

#endif
