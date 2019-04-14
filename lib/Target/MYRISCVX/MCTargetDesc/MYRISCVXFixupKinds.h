//===-- MYRISCVXFixupKinds.h - MYRISCVX Specific Fixup Entries ----------*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_MYRISCVX_MCTARGETDESC_MYRISCVXFIXUPKINDS_H
#define LLVM_LIB_TARGET_MYRISCVX_MCTARGETDESC_MYRISCVXFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
  namespace MYRISCVX {
    // Although most of the current fixup types reflect a unique relocation
    // one can have multiple fixup types for a given relocation and thus need
    // to be uniquely named.
    //
    // This table *must* be in the save order of
    // MCFixupKindInfo Infos[MYRISCVX::NumTargetFixupKinds]
    // in MYRISCVXAsmBackend.cpp.
    //@Fixups {
    enum Fixups {
      //@ Pure upper 32 bit fixup resulting in - R_MYRISCVX_32.
      fixup_MYRISCVX_32 = FirstTargetFixupKind,
      // Pure upper 16 bit fixup resulting in - R_MYRISCVX_HI16.
      fixup_MYRISCVX_HI16,
      // Pure lower 16 bit fixup resulting in - R_MYRISCVX_LO16.
      fixup_MYRISCVX_LO16,
      // 16 bit fixup for GP offest resulting in - R_MYRISCVX_GPREL16.
      fixup_MYRISCVX_GPREL16,
      // Symbol fixup resulting in - R_MYRISCVX_GOT16.
      fixup_MYRISCVX_GOT,
      // resulting in - R_MYRISCVX_GOT_HI16
      fixup_MYRISCVX_GOT_HI16,
      // resulting in - R_MYRISCVX_GOT_LO16
      fixup_MYRISCVX_GOT_LO16,
      // fixup_MYRISCVX_PCREL_LO12_I - 12-bit fixup corresponding to pcrel_lo(foo) for
      // instructions like addi
      fixup_MYRISCVX_PCREL_LO12_I,
      // fixup_MYRISCVX_PCREL_LO12_S - 12-bit fixup corresponding to pcrel_lo(foo) for
      // the S-type store instructions
      fixup_MYRISCVX_PCREL_LO12_S,
      // resulting in - R_MYRISCVX_CALL16.
      fixup_MYRISCVX_CALL16,

      // resulting in - R_MYRISCVX_TLS_GD.
      fixup_MYRISCVX_TLSGD,

      // resulting in - R_MYRISCVX_TLS_GOTTPREL.
      fixup_MYRISCVX_GOTTPREL,

      // resulting in - R_MYRISCVX_TLS_TPREL_HI16.
      fixup_MYRISCVX_TP_HI,

      // resulting in - R_MYRISCVX_TLS_TPREL_LO16.
      fixup_MYRISCVX_TP_LO,

      // resulting in - R_MYRISCVX_TLS_LDM.
      fixup_MYRISCVX_TLSLDM,

      // resulting in - R_MYRISCVX_TLS_DTP_HI16.
      fixup_MYRISCVX_DTP_HI,

      // resulting in - R_MYRISCVX_TLS_DTP_LO16.
      fixup_MYRISCVX_DTP_LO,

      // Marker
      LastTargetFixupKind,
      NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
    };
    //@Fixups }
  } // namespace MYRISCVX
} // namespace llvm


#endif // LLVM_MYRISCVX_MYRISCVXFIXUPKINDS_H
