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

    void expandRetRA(MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const;

   public:
    explicit MYRISCVXInstrInfo();

    static const MYRISCVXInstrInfo *create(MYRISCVXSubtarget &STI);

    /// Return the number of bytes of code the specified instruction may be.
    unsigned GetInstSizeInBytes(const MachineInstr &MI) const;

    //@expandPostRAPseudo
    bool expandPostRAPseudo(MachineInstr &MI) const override;

    /// Adjust SP by Amount bytes.
    void adjustStackPtr(unsigned SP, int64_t Amount, MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I) const;
    /// Emit a series of instructions to load an immediate. If NewImm is a
    /// non-NULL parameter, the last instruction is not emitted, but instead
    /// its immediate operand is returned in NewImm.
    void loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                       MachineBasicBlock::iterator II, const DebugLoc &DL,
                       unsigned DstReg, unsigned *NewImm) const;

    void copyPhysReg(MachineBasicBlock &MBB,
                     MachineBasicBlock::iterator MBBI,
                     const DebugLoc &DL, unsigned DstReg,
                     unsigned SrcReg, bool KillSrc) const override;

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

    MachineMemOperand *GetMemOperand(MachineBasicBlock &MBB, int FI,
                                     MachineMemOperand::Flags Flags) const;

    void storeRegToStack(MachineBasicBlock &MBB,
                         MachineBasicBlock::iterator MI,
                         unsigned SrcReg, bool isKill, int FrameIndex,
                         const TargetRegisterClass *RC,
                         const TargetRegisterInfo *TRI,
                         int64_t Offset) const;

    void loadRegFromStack(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI,
                          unsigned DestReg, int FrameIndex,
                          const TargetRegisterClass *RC,
                          const TargetRegisterInfo *TRI,
                          int64_t Offset) const;

   protected:
  };
}

#endif
