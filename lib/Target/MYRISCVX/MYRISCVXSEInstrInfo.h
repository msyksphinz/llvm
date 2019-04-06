//===-- MYRISCVXSEInstrInfo.h - MYRISCVX32/64 Instruction Information ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX32/64 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSEINSTRINFO_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSEINSTRINFO_H

#include "MYRISCVXInstrInfo.h"
#include "MYRISCVXSERegisterInfo.h"
#include "MYRISCVXMachineFunction.h"

namespace llvm {

  class MYRISCVXSEInstrInfo : public MYRISCVXInstrInfo {
    const MYRISCVXSERegisterInfo RI;

 public:
    explicit MYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI);

    const MYRISCVXRegisterInfo &getRegisterInfo() const override;

    //@expandPostRAPseudo
    bool expandPostRAPseudo(MachineInstr &MI) const override;

    /// Adjust SP by Amount bytes.
    void adjustStackPtr(unsigned SP, int64_t Amount, MachineBasicBlock &MBB,
                        MachineBasicBlock::iterator I) const override;

    void expandEhReturn(MachineBasicBlock &MBB,
                        MachineBasicBlock::iterator I) const;

    /// Emit a series of instructions to load an immediate. If NewImm is a
    /// non-NULL parameter, the last instruction is not emitted, but instead
    /// its immediate operand is returned in NewImm.
    unsigned loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator II, const DebugLoc &DL,
                           unsigned *NewImm) const;

    void storeRegToStack(MachineBasicBlock &MBB,
                         MachineBasicBlock::iterator MI,
                         unsigned SrcReg, bool isKill, int FrameIndex,
                         const TargetRegisterClass *RC,
                         const TargetRegisterInfo *TRI,
                         int64_t Offset) const override;
    void loadRegFromStack(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI,
                          unsigned DestReg, int FrameIndex,
                          const TargetRegisterClass *RC,
                          const TargetRegisterInfo *TRI,
                          int64_t Offset) const override;

    void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                     const DebugLoc &DL, unsigned DstReg, unsigned SrcReg,
                     bool KillSrc) const override;

    unsigned getOppositeBranchOpc(unsigned Opc) const override;

   private:
    void expandRetLR(MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const;
  };

}

#endif
