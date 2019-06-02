//===-- MYRISCVXSEInstrInfo.h - MYRISCVX Instruction Information ---*- C++ -*-===//
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

  /// Emit a series of instructions to load an immediate. If NewImm is a
  /// non-NULL parameter, the last instruction is not emitted, but instead
  /// its immediate operand is returned in NewImm.
  void loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                     MachineBasicBlock::iterator II, const DebugLoc &DL,
                     unsigned DstReg, unsigned *NewImm) const;
 private:
  void expandRetRA(MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const;
};

}

#endif
