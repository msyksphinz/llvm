//===-- llvm/Target/MYRISCVXTargetObjectFile.h - MYRISCVX Object Info ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXTARGETOBJECTFILE_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXTARGETOBJECTFILE_H

#include "MYRISCVXTargetMachine.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {

  class MYRISCVXTargetMachine;

  class MYRISCVXTargetObjectFile : public TargetLoweringObjectFileELF {
    MCSection *SmallDataSection;
    MCSection *SmallBSSSection;
    const MYRISCVXTargetMachine *TM;
  public:

    void Initialize(MCContext &Ctx, const TargetMachine &TM) override;

  };

} // end namespace llvm

#endif
