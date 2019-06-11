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

    /// IsGlobalInSmallSection - Return true if this global address should be
    /// placed into small data/bss section.
    bool IsGlobalInSmallSection(const GlobalObject *GO,
                                const TargetMachine &TM, SectionKind Kind) const;
    bool IsGlobalInSmallSection(const GlobalObject *GO,
                                const TargetMachine &TM) const;
    bool IsGlobalInSmallSectionImpl(const GlobalObject *GO,
                                    const TargetMachine &TM) const;

    MCSection *SelectSectionForGlobal(const GlobalObject *GO, SectionKind Kind,
                                      const TargetMachine &TM) const override;
  };

} // end namespace llvm

#endif
