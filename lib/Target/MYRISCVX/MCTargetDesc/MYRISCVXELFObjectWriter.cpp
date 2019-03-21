//===-- MYRISCVXELFObjectWriter.cpp - MYRISCVX ELF Writer -------------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MYRISCVXBaseInfo.h"
#include "MCTargetDesc/MYRISCVXFixupKinds.h"
#include "MCTargetDesc/MYRISCVXMCTargetDesc.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"
#include <list>

using namespace llvm;

namespace {
class MYRISCVXELFObjectWriter : public MCELFObjectTargetWriter {
 public:
  MYRISCVXELFObjectWriter(uint8_t OSABI);
  ~MYRISCVXELFObjectWriter() override;
  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
  bool needsRelocateWithSymbol(const MCSymbol &Sym,
                               unsigned Type) const override;
};
}


MYRISCVXELFObjectWriter::MYRISCVXELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(/*_is64Bit=false*/ false, OSABI, ELF::EM_MYRISCVX,
                              /*HasRelocationAddend*/ false) {}


MYRISCVXELFObjectWriter::~MYRISCVXELFObjectWriter() {}


//@GetRelocType {
unsigned MYRISCVXELFObjectWriter::getRelocType(MCContext &Ctx,
                                               const MCValue &Target,
                                               const MCFixup &Fixup,
                                               bool IsPCRel) const {
  // determine the type of the relocation
  unsigned Type = (unsigned)ELF::R_MYRISCVX_NONE;
  unsigned Kind = (unsigned)Fixup.getKind();
  switch (Kind) {
    default:
      llvm_unreachable("invalid fixup kind!");
    case FK_Data_4:
      Type = ELF::R_MYRISCVX_32;
      break;
    case MYRISCVX::fixup_MYRISCVX_32:
      Type = ELF::R_MYRISCVX_32;
      break;
    case MYRISCVX::fixup_MYRISCVX_GPREL16:
      Type = ELF::R_MYRISCVX_GPREL16;
      break;
    case MYRISCVX::fixup_MYRISCVX_GOT:
      Type = ELF::R_MYRISCVX_GOT16;
      break;
    case MYRISCVX::fixup_MYRISCVX_HI16:
      Type = ELF::R_MYRISCVX_HI16;
      break;
    case MYRISCVX::fixup_MYRISCVX_LO16:
      Type = ELF::R_MYRISCVX_LO16;
      break;
    case MYRISCVX::fixup_MYRISCVX_GOT_HI16:
      Type = ELF::R_MYRISCVX_GOT_HI16;
      break;
    case MYRISCVX::fixup_MYRISCVX_GOT_LO16:
      Type = ELF::R_MYRISCVX_GOT_LO16;
      break;
    case MYRISCVX::fixup_MYRISCVX_CALL16:
      Type = ELF::R_MYRISCVX_CALL16;
      break;
  }
  return Type;
}


//@GetRelocType }
bool
MYRISCVXELFObjectWriter::needsRelocateWithSymbol(const MCSymbol &Sym,
                                                 unsigned Type) const {
  // FIXME: This is extremelly conservative. This really needs to use a
  // whitelist with a clear explanation for why each realocation needs to
  // point to the symbol, not to the section.
  switch (Type) {
    default:
      return true;
    case ELF::R_MYRISCVX_GOT16:
      // For MYRISCVX pic mode, I think it's OK to return true but I didn't confirm.
      // llvm_unreachable("Should have been handled already");
      return true;
      // These relocations might be paired with another relocation. The pairing is
      // done by the static linker by matching the symbol. Since we only see one
      // relocation at a time, we have to force them to relocate with a symbol to
      // avoid ending up with a pair where one points to a section and another
      // points to a symbol.
    case ELF::R_MYRISCVX_HI16:
    case ELF::R_MYRISCVX_LO16:
      // R_MYRISCVX_32 should be a relocation record, I don't know why Mips set it to
      // false.
    case ELF::R_MYRISCVX_32:
      return true;
    case ELF::R_MYRISCVX_GPREL16:
      return false;
  }
}


std::unique_ptr<MCObjectTargetWriter>
llvm::createMYRISCVXELFObjectWriter (const Triple &TT) {
  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(TT.getOS());
  return llvm::make_unique<MYRISCVXELFObjectWriter>(OSABI);
}
