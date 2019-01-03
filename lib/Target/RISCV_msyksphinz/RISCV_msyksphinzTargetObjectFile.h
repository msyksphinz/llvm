//===-- llvm/Target/Cpu0TargetObjectFile.h - Cpu0 Object Info ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RISCV_msyksphinz_RISCV_msyksphinzTARGETOBJECTFILE_H
#define LLVM_LIB_TARGET_RISCV_msyksphinz_RISCV_msyksphinzTARGETOBJECTFILE_H

#include "RISCV_msyksphinzConfig.h"

#include "RISCV_msyksphinzTargetMachine.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {
class RISCV_msyksphinzTargetMachine;
  class RISCV_msyksphinzTargetObjectFile : public TargetLoweringObjectFileELF {
    MCSection *SmallDataSection;
    MCSection *SmallBSSSection;
    const RISCV_msyksphinzTargetMachine *TM;
  public:

    void Initialize(MCContext &Ctx, const TargetMachine &TM) override;

  };
} // end namespace llvm

#endif
