//===-- MYRISCVXELFObjectWriter.cpp - MYRISCVX ELF Writer -------------------------===//
//
//                     The LLVM Compiler Infrastructure
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
    case MYRISCVX::fixup_MYRISCVX_HI20:
      return ELF::R_MYRISCVX_HI20;
    case MYRISCVX::fixup_MYRISCVX_LO12_I:
      return ELF::R_MYRISCVX_LO12_I;
    case MYRISCVX::fixup_MYRISCVX_LO12_S:
      return ELF::R_MYRISCVX_LO12_S;
    case MYRISCVX::fixup_MYRISCVX_PCREL_HI20:
      return ELF::R_MYRISCVX_PCREL_HI20;
    case MYRISCVX::fixup_MYRISCVX_PCREL_LO12_I:
      return ELF::R_MYRISCVX_PCREL_LO12_I;
    case MYRISCVX::fixup_MYRISCVX_PCREL_LO12_S:
      return ELF::R_MYRISCVX_PCREL_LO12_S;
    case MYRISCVX::fixup_MYRISCVX_JAL:
      return ELF::R_MYRISCVX_JAL;
    case MYRISCVX::fixup_MYRISCVX_BRANCH:
      return ELF::R_MYRISCVX_BRANCH;
    case MYRISCVX::fixup_MYRISCVX_RVC_JUMP:
      return ELF::R_MYRISCVX_RVC_JUMP;
    case MYRISCVX::fixup_MYRISCVX_RVC_BRANCH:
      return ELF::R_MYRISCVX_RVC_BRANCH;
    case MYRISCVX::fixup_MYRISCVX_CALL:
      return ELF::R_MYRISCVX_CALL;
    case MYRISCVX::fixup_MYRISCVX_RELAX:
      return ELF::R_MYRISCVX_RELAX;
  }

  return Type;
}
//@GetRelocType }

bool
MYRISCVXELFObjectWriter::needsRelocateWithSymbol(const MCSymbol &Sym,
                                                 unsigned Type) const {
  // TODO: this is very conservative, update once RISC-V psABI requirements
  //       are clarified.
  return true;
}


std::unique_ptr<MCObjectTargetWriter>
llvm::createMYRISCVXELFObjectWriter (const Triple &TT) {
  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(TT.getOS());
  return llvm::make_unique<MYRISCVXELFObjectWriter>(OSABI);
}
