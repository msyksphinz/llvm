//===-- MYRISCVXAsmBackend.h - MYRISCVX Asm Backend  ------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the MYRISCVXAsmBackend class.
//
//===----------------------------------------------------------------------===//
//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MCTARGETDESC_MYRISCVXASMBACKEND_H
#define LLVM_LIB_TARGET_MYRISCVX_MCTARGETDESC_MYRISCVXASMBACKEND_H

#include "MCTargetDesc/MYRISCVXFixupKinds.h"

#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/ADT/Triple.h"
#include "llvm/MC/MCAsmBackend.h"

namespace llvm {
  class MCAssembler;
  struct MCFixupKindInfo;
  class Target;
  class MCObjectWriter;

  class MYRISCVXAsmBackend : public MCAsmBackend {
    Triple TheTriple;
    Triple::OSType OSType;

   public:
    MYRISCVXAsmBackend(const Target &T,
                       const MCRegisterInfo &MRI,
                       const Triple &TT,
                       StringRef CPU)
        : MCAsmBackend(TT.isLittleEndian() ? support::little : support::big),
          TheTriple(TT) {}

    std::unique_ptr<MCObjectTargetWriter>
        createObjectTargetWriter() const override;

    void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                    const MCValue &Target, MutableArrayRef<char> Data,
                    uint64_t Value, bool IsResolved,
                    const MCSubtargetInfo *STI) const override;

    const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override;

    unsigned getNumFixupKinds() const override {
      return MYRISCVX::NumTargetFixupKinds;
    }

    /// @name Target Relaxation Interfaces
    /// @{

    /// MayNeedRelaxation - Check whether the given instruction may need
    /// relaxation.
    ///
    /// \param Inst - The instruction to test.
    bool mayNeedRelaxation(const MCInst &Inst,
                           const MCSubtargetInfo &STI) const override {
      return false;
    }

    /// fixupNeedsRelaxation - Target specific predicate for whether a given
    /// fixup requires the associated instruction to be relaxed.
    bool fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                              const MCRelaxableFragment *DF,
                              const MCAsmLayout &Layout) const override {
      // FIXME.
      llvm_unreachable("RelaxInstruction() unimplemented");
      return false;
    }

    /// RelaxInstruction - Relax the instruction in the given fragment
    /// to the next wider instruction.
    ///
    /// \param Inst - The instruction to relax, which may be the same
    /// as the output.
    /// \param [out] Res On return, the relaxed instruction.
    void relaxInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                          MCInst &Res) const override {}

    /// @}

    bool writeNopData(raw_ostream &OS, uint64_t Count) const override;
  }; // class MYRISCVXAsmBackend

} // namespace

#endif
