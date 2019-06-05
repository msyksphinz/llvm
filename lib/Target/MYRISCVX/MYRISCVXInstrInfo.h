//===-- MYRISCVXInstrInfo.h - MYRISCVX Instruction Information --*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXINSTRINFO_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXINSTRINFO_H

#include "MYRISCVX.h"
#include "MYRISCVXRegisterInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "MYRISCVXGenInstrInfo.inc"

namespace llvm {

  class MYRISCVXInstrInfo : public MYRISCVXGenInstrInfo {
    virtual void anchor();
 protected:
    const MYRISCVXSubtarget &Subtarget;
 public:
    explicit MYRISCVXInstrInfo(const MYRISCVXSubtarget &STI);

    static const MYRISCVXInstrInfo *create(MYRISCVXSubtarget &STI);

    /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
    /// such, whenever a client has an instance of instruction info, it should
    /// always be able to get register info as well (through this method).
    ///
    virtual const MYRISCVXRegisterInfo &getRegisterInfo() const = 0;

    /// Return the number of bytes of code the specified instruction may be.
    unsigned GetInstSizeInBytes(const MachineInstr &MI) const;

    /// Adjust SP by Amount bytes.
    virtual void adjustStackPtr(unsigned SP, int64_t Amount, MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I) const = 0;

    void storeRegToStackSlot(MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator MBBI,
                             unsigned SrcReg, bool isKill, int FrameIndex,
                             const TargetRegisterClass *RC,
                             const TargetRegisterInfo *TRI) const override {
      storeRegToStack(MBB, MBBI, SrcReg, isKill, FrameIndex, RC, TRI, 0);
    }

    void loadRegFromStackSlot(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MBBI,
                              unsigned DestReg, int FrameIndex,
                              const TargetRegisterClass *RC,
                              const TargetRegisterInfo *TRI) const override {
      loadRegFromStack(MBB, MBBI, DestReg, FrameIndex, RC, TRI, 0);
    }

    virtual void storeRegToStack(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MI,
                                 unsigned SrcReg, bool isKill, int FrameIndex,
                                 const TargetRegisterClass *RC,
                                 const TargetRegisterInfo *TRI,
                                 int64_t Offset) const = 0;

    virtual void loadRegFromStack(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator MI,
                                  unsigned DestReg, int FrameIndex,
                                  const TargetRegisterClass *RC,
                                  const TargetRegisterInfo *TRI,
                                  int64_t Offset) const = 0;

    MachineMemOperand *GetMemOperand(MachineBasicBlock &MBB, int FI,
                                     MachineMemOperand::Flags Flags) const;
   protected:
  };
  const MYRISCVXInstrInfo *createMYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI);
}

#endif
