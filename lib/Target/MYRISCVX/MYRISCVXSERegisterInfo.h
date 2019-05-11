//===-- MYRISCVXSERegisterInfo.h - MYRISCVX32 Register Information ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX32/64 implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSEREGISTERINFO_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSEREGISTERINFO_H

#include "MYRISCVXRegisterInfo.h"

namespace llvm {
class MYRISCVXSEInstrInfo;

class MYRISCVXSERegisterInfo : public MYRISCVXRegisterInfo {
 public:
  MYRISCVXSERegisterInfo(const MYRISCVXSubtarget &Subtarget);

  const TargetRegisterClass *intRegClass(unsigned Size) const override;
};

} // end namespace llvm

#endif
