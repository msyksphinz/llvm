//===-- MYRISCVXAsmBackend.cpp - MYRISCVX Asm Backend ----------------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MYRISCVXAsmBackend class.
//
//===----------------------------------------------------------------------===//
//

#include "MCTargetDesc/MYRISCVXFixupKinds.h"
#include "MCTargetDesc/MYRISCVXAsmBackend.h"
#include "MCTargetDesc/MYRISCVXMCTargetDesc.h"

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;


//@adjustFixupValue {
// Prepare value for the target space for it
static unsigned adjustFixupValue(const MCFixup &Fixup, uint64_t Value,
                                 MCContext *Ctx = nullptr) {
  unsigned Kind = Fixup.getKind();
  // Add/subtract and shift
  switch (Kind) {
    default:
      return 0;
    case FK_GPRel_4:
    case FK_Data_4:
    case MYRISCVX::fixup_MYRISCVX_LO16:
      break;
    case MYRISCVX::fixup_MYRISCVX_HI16:
    case MYRISCVX::fixup_MYRISCVX_GOT:
      // Get the higher 16-bits. Also add 1 if bit 15 is 1.
      Value = ((Value + 0x8000) >> 16) & 0xffff;
      break;
  }
  return Value;
}
//@adjustFixupValue }


std::unique_ptr<MCObjectTargetWriter>
MYRISCVXAsmBackend::createObjectTargetWriter() const {
  return createMYRISCVXELFObjectWriter(TheTriple);
}


/// ApplyFixup - Apply the \p Value for given \p Fixup into the provided
/// data fragment, at the offset specified by the fixup and following the
/// fixup kind as appropriate.
void MYRISCVXAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                                    const MCValue &Target,
                                    MutableArrayRef<char> Data, uint64_t Value,
                                    bool IsResolved,
                                    const MCSubtargetInfo *STI) const {

  MCFixupKind Kind = Fixup.getKind();
  Value = adjustFixupValue(Fixup, Value);
  if (!Value)
    return; // Doesn't change encoding.
  // Where do we start in the object
  unsigned Offset = Fixup.getOffset();
  // Number of bytes we need to fixup
  unsigned NumBytes = (getFixupKindInfo(Kind).TargetSize + 7) / 8;
  // Used to point to big endian bytes
  unsigned FullSize;
  switch ((unsigned)Kind) {
    default:
      FullSize = 4;
      break;
  }
  // Grab current value, if any, from bits.
  uint64_t CurVal = 0;
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned Idx = i;
    CurVal |= (uint64_t)((uint8_t)Data[Offset + Idx]) << (i*8);
  }
  uint64_t Mask = ((uint64_t)(-1) >>
                   (64 - getFixupKindInfo(Kind).TargetSize));
  CurVal |= Value & Mask;
  // Write out the fixed up bytes back to the code/data bits.
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned Idx = i;
    Data[Offset + Idx] = (uint8_t)((CurVal >> (i*8)) & 0xff);
  }
}


//@getFixupKindInfo {
const MCFixupKindInfo &MYRISCVXAsmBackend::
getFixupKindInfo(MCFixupKind Kind) const {
  const static MCFixupKindInfo Infos[MYRISCVX::NumTargetFixupKinds] = {
    // This table *must* be in same the order of fixup_* kinds in
    // MYRISCVXFixupKinds.h.
    //
    // name offset bits flags
    { "fixup_MYRISCVX_32", 0, 32, 0 },
    { "fixup_MYRISCVX_HI16", 0, 16, 0 },
    { "fixup_MYRISCVX_LO16", 0, 16, 0 },
    { "fixup_MYRISCVX_GPREL16", 0, 16, 0 },
    { "fixup_MYRISCVX_GOT", 0, 16, 0 },
    { "fixup_MYRISCVX_GOT_HI16", 0, 16, 0 },
    { "fixup_MYRISCVX_GOT_LO16", 0, 16, 0 }
  };
  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);
  assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
         "Invalid kind!");
  return Infos[Kind - FirstTargetFixupKind];
}

//@getFixupKindInfo }
/// WriteNopData - Write an (optimal) nop sequence of Count bytes
/// to the given output. If the target cannot generate such a sequence,
/// it should return an error.
///
/// \return - True on success.
bool MYRISCVXAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const {
  return true;
}


// MCAsmBackend
MCAsmBackend *llvm::createMYRISCVXAsmBackendEL32(const Target &T,
                                                 const MCRegisterInfo &MRI,
                                                 const Triple &TT, StringRef CPU) {
  return new MYRISCVXAsmBackend(T, TT.getOS(), /*IsLittle*/true);
}


MCAsmBackend *llvm::createMYRISCVXAsmBackendEB32(const Target &T,
                                                 const MCRegisterInfo &MRI,
                                                 const Triple &TT, StringRef CPU) {
  return new MYRISCVXAsmBackend(T, MRI, STI.getTargetTriple(), STI.getCPU());
}
