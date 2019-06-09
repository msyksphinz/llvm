//===-- MYRISCVXFixupKinds.h - MYRISCVX Specific Fixup Entries ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
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
      // fixup_MYRISCVX_hi20 - 20-bit fixup corresponding to hi(foo) for
      // instructions like lui
      fixup_MYRISCVX_HI20 = FirstTargetFixupKind,
      // fixup_MYRISCVX_LO12_i - 12-bit fixup corresponding to lo(foo) for
      // instructions like addi
      fixup_MYRISCVX_LO12_I,
      // fixup_MYRISCVX_lo12_s - 12-bit fixup corresponding to lo(foo) for
      // the S-type store instructions
      fixup_MYRISCVX_LO12_S,
      // fixup_MYRISCVX_pcrel_hi20 - 20-bit fixup corresponding to pcrel_hi(foo) for
      // instructions like auipc
      fixup_MYRISCVX_PCREL_HI20,
      // fixup_MYRISCVX_PCREL_LO12_I - 12-bit fixup corresponding to pcrel_lo(foo) for
      // instructions like addi
      fixup_MYRISCVX_PCREL_LO12_I,
      // FIXUP_MYRISCVX_PCREL_LO12_S - 12-bit fixup corresponding to pcrel_lo(foo) for
      // the S-type store instructions
      fixup_MYRISCVX_PCREL_LO12_S,
      // fixup_MYRISCVX_JAL - 20-bit fixup for symbol references in the jal
      // instruction
      fixup_MYRISCVX_JAL,
      // fixup_MYRISCVX_BRANCH - 12-bit fixup for symbol references in the branch
      // instructions
      fixup_MYRISCVX_BRANCH,
      // fixup_MYRISCVX_RVC_JUMP - 11-bit fixup for symbol references in the
      // compressed jump instruction
      fixup_MYRISCVX_RVC_JUMP,
      // fixup_MYRISCVX_RVC_BRANCH - 8-bit fixup for symbol references in the
      // compressed branch instruction
      fixup_MYRISCVX_RVC_BRANCH,
      // fixup_MYRISCVX_CALL - A fixup representing a call attached to the auipc
      // instruction in a pair composed of adjacent auipc+jalr instructions.
      fixup_MYRISCVX_CALL,
      // fixup_MYRISCVX_RELAX - Used to generate an R_MYRISCVX_RELAX relocation type,
      // which indicates the linker may relax the instruction pair.
      fixup_MYRISCVX_RELAX,

      // fixup_MYRISCVX_invalid - used as a sentinel and a marker, must be last fixup
      LastTargetFixupKind,
      NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
    };
    //@Fixups }
  } // namespace MYRISCVX
} // namespace llvm

#endif // LLVM_MYRISCVX_MYRISCVXFIXUPKINDS_H
