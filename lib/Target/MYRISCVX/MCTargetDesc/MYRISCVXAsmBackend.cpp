//===-- MYRISCVXAsmBackend.cpp - MYRISCVX Asm Backend  ----------------------------===//
//
//                     The LLVM Compiler Infrastructure
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

#include "MCTargetDesc/MYRISCVXAsmBackend.h"
#include "MCTargetDesc/MYRISCVXFixupKinds.h"
#include "MCTargetDesc/MYRISCVXMCTargetDesc.h"

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
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
                                 MCContext &Ctx) {

  unsigned Kind = Fixup.getKind();

  // Add/subtract and shift
  switch (Kind) {
    default:
      return 0;
    case FK_GPRel_4:
    case FK_Data_4:
      return Value;
    case MYRISCVX::fixup_MYRISCVX_LO12_I:
    case MYRISCVX::fixup_MYRISCVX_PCREL_LO12_I:
      return Value & 0xfff;
    case MYRISCVX::fixup_MYRISCVX_LO12_S:
    case MYRISCVX::fixup_MYRISCVX_PCREL_LO12_S:
      return (((Value >> 5) & 0x7f) << 25) | ((Value & 0x1f) << 7);
    case MYRISCVX::fixup_MYRISCVX_HI20:
    case MYRISCVX::fixup_MYRISCVX_PCREL_HI20:
      // Add 1 if bit 11 is 1, to compensate for low 12 bits being negative.
      return ((Value + 0x800) >> 12) & 0xfffff;
    case MYRISCVX::fixup_MYRISCVX_JAL: {
      if (!isInt<21>(Value))
        Ctx.reportError(Fixup.getLoc(), "fixup value out of range");
      if (Value & 0x1)
        Ctx.reportError(Fixup.getLoc(), "fixup value must be 2-byte aligned");
      // Need to produce imm[19|10:1|11|19:12] from the 21-bit Value.
      unsigned Sbit = (Value >> 20) & 0x1;
      unsigned Hi8 = (Value >> 12) & 0xff;
      unsigned Mid1 = (Value >> 11) & 0x1;
      unsigned Lo10 = (Value >> 1) & 0x3ff;
      // Inst{31} = Sbit;
      // Inst{30-21} = Lo10;
      // Inst{20} = Mid1;
      // Inst{19-12} = Hi8;
      Value = (Sbit << 19) | (Lo10 << 9) | (Mid1 << 8) | Hi8;
      return Value;
    }
    case MYRISCVX::fixup_MYRISCVX_BRANCH: {
      if (!isInt<13>(Value))
        Ctx.reportError(Fixup.getLoc(), "fixup value out of range");
      if (Value & 0x1)
        Ctx.reportError(Fixup.getLoc(), "fixup value must be 2-byte aligned");
      // Need to extract imm[12], imm[10:5], imm[4:1], imm[11] from the 13-bit
      // Value.
      unsigned Sbit = (Value >> 12) & 0x1;
      unsigned Hi1 = (Value >> 11) & 0x1;
      unsigned Mid6 = (Value >> 5) & 0x3f;
      unsigned Lo4 = (Value >> 1) & 0xf;
      // Inst{31} = Sbit;
      // Inst{30-25} = Mid6;
      // Inst{11-8} = Lo4;
      // Inst{7} = Hi1;
      Value = (Sbit << 31) | (Mid6 << 25) | (Lo4 << 8) | (Hi1 << 7);
      return Value;
    }
    case MYRISCVX::fixup_MYRISCVX_CALL: {
      // Jalr will add UpperImm with the sign-extended 12-bit LowerImm,
      // we need to add 0x800ULL before extract upper bits to reflect the
      // effect of the sign extension.
      uint64_t UpperImm = (Value + 0x800ULL) & 0xfffff000ULL;
      uint64_t LowerImm = Value & 0xfffULL;
      return UpperImm | ((LowerImm << 20) << 32);
    }
    case MYRISCVX::fixup_MYRISCVX_RVC_JUMP: {
      // Need to produce offset[11|4|9:8|10|6|7|3:1|5] from the 11-bit Value.
      unsigned Bit11  = (Value >> 11) & 0x1;
      unsigned Bit4   = (Value >> 4) & 0x1;
      unsigned Bit9_8 = (Value >> 8) & 0x3;
      unsigned Bit10  = (Value >> 10) & 0x1;
      unsigned Bit6   = (Value >> 6) & 0x1;
      unsigned Bit7   = (Value >> 7) & 0x1;
      unsigned Bit3_1 = (Value >> 1) & 0x7;
      unsigned Bit5   = (Value >> 5) & 0x1;
      Value = (Bit11 << 10) | (Bit4 << 9) | (Bit9_8 << 7) | (Bit10 << 6) |
          (Bit6 << 5) | (Bit7 << 4) | (Bit3_1 << 1) | Bit5;
      return Value;
    }
    case MYRISCVX::fixup_MYRISCVX_RVC_BRANCH: {
      // Need to produce offset[8|4:3], [reg 3 bit], offset[7:6|2:1|5]
      unsigned Bit8   = (Value >> 8) & 0x1;
      unsigned Bit7_6 = (Value >> 6) & 0x3;
      unsigned Bit5   = (Value >> 5) & 0x1;
      unsigned Bit4_3 = (Value >> 3) & 0x3;
      unsigned Bit2_1 = (Value >> 1) & 0x3;
      Value = (Bit8 << 12) | (Bit4_3 << 10) | (Bit7_6 << 5) | (Bit2_1 << 3) |
          (Bit5 << 2);
      return Value;
    }
  }

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
  MCContext &Ctx = Asm.getContext();
  MCFixupKind Kind = Fixup.getKind();
  Value = adjustFixupValue(Fixup, Value, Ctx);

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
    // name                        offset  bits  flags
    { "fixup_MYRISCVX_32",             0,     32,   0 },
    { "fixup_MYRISCVX_HI16",           0,     16,   0 },
    { "fixup_MYRISCVX_LO16",           0,     16,   0 },
    { "fixup_MYRISCVX_GPREL16",        0,     16,   0 },
    { "fixup_MYRISCVX_GOT",            0,     16,   0 },
    { "fixup_MYRISCVX_GOT_HI16",       0,     16,   0 },
    { "fixup_MYRISCVX_GOT_LO16",       0,     16,   0 }
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
                                                 const MCSubtargetInfo &STI,
                                                 const MCRegisterInfo &MRI,
                                                 const MCTargetOptions &Options) {
  return new MYRISCVXAsmBackend(T, MRI, STI.getTargetTriple(), STI.getCPU());
}


MCAsmBackend *llvm::createMYRISCVXAsmBackendEB32(const Target &T,
                                                 const MCSubtargetInfo &STI,
                                                 const MCRegisterInfo &MRI,
                                                 const MCTargetOptions &Options) {
  return new MYRISCVXAsmBackend(T, MRI, STI.getTargetTriple(), STI.getCPU());
}
