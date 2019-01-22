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

 private:
  void expandRetLR(MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const;

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

};

}

#endif
