//===-- RISCV64FixupKinds.h - RISCV64 Specific Fixup Entries ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MIPS_MCTARGETDESC_MIPSFIXUPKINDS_H
#define LLVM_LIB_TARGET_MIPS_MCTARGETDESC_MIPSFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace RISCV64 {
  // Although most of the current fixup types reflect a unique relocation
  // one can have multiple fixup types for a given relocation and thus need
  // to be uniquely named.
  //
  // This table *must* be in the same order of
  // MCFixupKindInfo Infos[RISCV64::NumTargetFixupKinds]
  // in RISCV64AsmBackend.cpp.
  //
  enum Fixups {
    // Branch fixups resulting in R_MIPS_NONE.
    fixup_RISCV64_NONE = FirstTargetFixupKind,

    // Branch fixups resulting in R_MIPS_16.
    fixup_RISCV64_16,

    // Pure 32 bit data fixup resulting in - R_MIPS_32.
    fixup_RISCV64_32,

    // Full 32 bit data relative data fixup resulting in - R_MIPS_REL32.
    fixup_RISCV64_REL32,

    // Jump 26 bit fixup resulting in - R_MIPS_26.
    fixup_RISCV64_26,

    // Pure upper 16 bit fixup resulting in - R_MIPS_HI16.
    fixup_RISCV64_HI16,

    // Pure lower 16 bit fixup resulting in - R_MIPS_LO16.
    fixup_RISCV64_LO16,

    // 16 bit fixup for GP offest resulting in - R_MIPS_GPREL16.
    fixup_RISCV64_GPREL16,

    // 16 bit literal fixup resulting in - R_MIPS_LITERAL.
    fixup_RISCV64_LITERAL,

    // Symbol fixup resulting in - R_MIPS_GOT16.
    fixup_RISCV64_GOT,

    // PC relative branch fixup resulting in - R_MIPS_PC16.
    fixup_RISCV64_PC16,

    // resulting in - R_MIPS_CALL16.
    fixup_RISCV64_CALL16,

    // resulting in - R_MIPS_GPREL32.
    fixup_RISCV64_GPREL32,

    // resulting in - R_MIPS_SHIFT5.
    fixup_RISCV64_SHIFT5,

    // resulting in - R_MIPS_SHIFT6.
    fixup_RISCV64_SHIFT6,

    // Pure 64 bit data fixup resulting in - R_MIPS_64.
    fixup_RISCV64_64,

    // resulting in - R_MIPS_TLS_GD.
    fixup_RISCV64_TLSGD,

    // resulting in - R_MIPS_TLS_GOTTPREL.
    fixup_RISCV64_GOTTPREL,

    // resulting in - R_MIPS_TLS_TPREL_HI16.
    fixup_RISCV64_TPREL_HI,

    // resulting in - R_MIPS_TLS_TPREL_LO16.
    fixup_RISCV64_TPREL_LO,

    // resulting in - R_MIPS_TLS_LDM.
    fixup_RISCV64_TLSLDM,

    // resulting in - R_MIPS_TLS_DTPREL_HI16.
    fixup_RISCV64_DTPREL_HI,

    // resulting in - R_MIPS_TLS_DTPREL_LO16.
    fixup_RISCV64_DTPREL_LO,

    // PC relative branch fixup resulting in - R_MIPS_PC16
    fixup_RISCV64_Branch_PCRel,

    // resulting in - R_MIPS_GPREL16/R_MIPS_SUB/R_MIPS_HI16
    //                R_MICROMIPS_GPREL16/R_MICROMIPS_SUB/R_MICROMIPS_HI16
    fixup_RISCV64_GPOFF_HI,
    fixup_MICROMIPS_GPOFF_HI,

    // resulting in - R_MIPS_GPREL16/R_MIPS_SUB/R_MIPS_LO16
    //                R_MICROMIPS_GPREL16/R_MICROMIPS_SUB/R_MICROMIPS_LO16
    fixup_RISCV64_GPOFF_LO,
    fixup_MICROMIPS_GPOFF_LO,

    // resulting in - R_MIPS_PAGE
    fixup_RISCV64_GOT_PAGE,

    // resulting in - R_MIPS_GOT_OFST
    fixup_RISCV64_GOT_OFST,

    // resulting in - R_MIPS_GOT_DISP
    fixup_RISCV64_GOT_DISP,

    // resulting in - R_MIPS_HIGHER/R_MICROMIPS_HIGHER
    fixup_RISCV64_HIGHER,
    fixup_MICROMIPS_HIGHER,

    // resulting in - R_MIPS_HIGHEST/R_MICROMIPS_HIGHEST
    fixup_RISCV64_HIGHEST,
    fixup_MICROMIPS_HIGHEST,

    // resulting in - R_MIPS_GOT_HI16
    fixup_RISCV64_GOT_HI16,

    // resulting in - R_MIPS_GOT_LO16
    fixup_RISCV64_GOT_LO16,

    // resulting in - R_MIPS_CALL_HI16
    fixup_RISCV64_CALL_HI16,

    // resulting in - R_MIPS_CALL_LO16
    fixup_RISCV64_CALL_LO16,

    // resulting in - R_MIPS_PC18_S3
    fixup_MIPS_PC18_S3,

    // resulting in - R_MIPS_PC19_S2
    fixup_MIPS_PC19_S2,

    // resulting in - R_MIPS_PC21_S2
    fixup_MIPS_PC21_S2,

    // resulting in - R_MIPS_PC26_S2
    fixup_MIPS_PC26_S2,

    // resulting in - R_MIPS_PCHI16
    fixup_MIPS_PCHI16,

    // resulting in - R_MIPS_PCLO16
    fixup_MIPS_PCLO16,

    // resulting in - R_MICROMIPS_26_S1
    fixup_MICROMIPS_26_S1,

    // resulting in - R_MICROMIPS_HI16
    fixup_MICROMIPS_HI16,

    // resulting in - R_MICROMIPS_LO16
    fixup_MICROMIPS_LO16,

    // resulting in - R_MICROMIPS_GOT16
    fixup_MICROMIPS_GOT16,

    // resulting in - R_MICROMIPS_PC7_S1
    fixup_MICROMIPS_PC7_S1,

    // resulting in - R_MICROMIPS_PC10_S1
    fixup_MICROMIPS_PC10_S1,

    // resulting in - R_MICROMIPS_PC16_S1
    fixup_MICROMIPS_PC16_S1,

    // resulting in - R_MICROMIPS_PC26_S1
    fixup_MICROMIPS_PC26_S1,

    // resulting in - R_MICROMIPS_PC19_S2
    fixup_MICROMIPS_PC19_S2,

    // resulting in - R_MICROMIPS_PC18_S3
    fixup_MICROMIPS_PC18_S3,

    // resulting in - R_MICROMIPS_PC21_S1
    fixup_MICROMIPS_PC21_S1,

    // resulting in - R_MICROMIPS_CALL16
    fixup_MICROMIPS_CALL16,

    // resulting in - R_MICROMIPS_GOT_DISP
    fixup_MICROMIPS_GOT_DISP,

    // resulting in - R_MICROMIPS_GOT_PAGE
    fixup_MICROMIPS_GOT_PAGE,

    // resulting in - R_MICROMIPS_GOT_OFST
    fixup_MICROMIPS_GOT_OFST,

    // resulting in - R_MICROMIPS_TLS_GD
    fixup_MICROMIPS_TLS_GD,

    // resulting in - R_MICROMIPS_TLS_LDM
    fixup_MICROMIPS_TLS_LDM,

    // resulting in - R_MICROMIPS_TLS_DTPREL_HI16
    fixup_MICROMIPS_TLS_DTPREL_HI16,

    // resulting in - R_MICROMIPS_TLS_DTPREL_LO16
    fixup_MICROMIPS_TLS_DTPREL_LO16,

    // resulting in - R_MICROMIPS_TLS_GOTTPREL.
    fixup_MICROMIPS_GOTTPREL,

    // resulting in - R_MICROMIPS_TLS_TPREL_HI16
    fixup_MICROMIPS_TLS_TPREL_HI16,

    // resulting in - R_MICROMIPS_TLS_TPREL_LO16
    fixup_MICROMIPS_TLS_TPREL_LO16,

    // resulting in - R_MIPS_SUB/R_MICROMIPS_SUB
    fixup_RISCV64_SUB,
    fixup_MICROMIPS_SUB,

    // Marker
    LastTargetFixupKind,
    NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
  };
} // namespace RISCV64
} // namespace llvm


#endif
