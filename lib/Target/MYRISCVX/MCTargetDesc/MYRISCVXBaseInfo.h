//===-- MYRISCVXBaseInfo.h - Top level definitions for MYRISCVX MC ------*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone helper functions and enum definitions for
// the MYRISCVX target useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//

#pragma once

#include "MYRISCVXMCTargetDesc.h"
#include "MYRISCVXFixupKinds.h"

#include "llvm/MC/MCExpr.h"
#include "llvm/Support/DataTypes.h"
#include "llvm/Support/ErrorHandling.h"
namespace llvm {
  /// MYRISCVXII - This namespace holds all of the target specific flags that
  /// instruction info tracks.
  //@MYRISCVXII
  namespace MYRISCVXII {

    /// Target Operand Flag enum.
    enum TOF {
      //===------------------------------------------------------------------===//
      // MYRISCVX Specific MachineOperand flags.
      MO_NO_FLAG,
      /// MO_GOT_CALL - Represents the offset into the global offset table at
      /// which the address of a call site relocation entry symbol resides
      /// during execution. This is different from the above since this flag
      /// can only be present in call instructions.
      MO_GOT_CALL,
      /// MO_GPREL - Represents the offset from the current gp value to be used
      /// for the relocatable object file being produced.
      MO_GPREL,
      /// MO_ABS_HI/LO - Represents the hi or low part of an absolute symbol
      /// address.
      MO_ABS_HI,
      MO_ABS_LO,
      MO_GOT16,
      MO_GOT,
      /// MO_GOT_HI16/LO16 - Relocations used for large GOTs.
      MO_GOT_HI16,
      MO_GOT_LO16,
      /// MO_TLSGD - Represents the offset into the global offset table at which
      // the module ID and TSL block offset reside during execution (General
      // Dynamic TLS).
      MO_TLSGD,

      /// MO_TLSLDM - Represents the offset into the global offset table at which
      // the module ID and TSL block offset reside during execution (Local
      // Dynamic TLS).
      MO_TLSLDM,
      MO_DTP_HI,
      MO_DTP_LO,

      /// MO_GOTTPREL - Represents the offset from the thread pointer (Initial
      // Exec TLS).
      MO_GOTTPREL,

      /// MO_TPREL_HI/LO - Represents the hi and low part of the offset from
      // the thread pointer (Local Exec TLS).
      MO_TP_HI,
      MO_TP_LO,
    }; // enum TOF {

    enum {
      //===------------------------------------------------------------------===//
      // Instruction encodings. These are the standard/most common forms for
      // MYRISCVX instructions.
      //
      // Pseudo - This represents an instruction that is a pseudo instruction
      // or one that has not been implemented yet. It is illegal to code generate
      // it, but tolerated for intermediate implementation stages.
      Pseudo = 0,
      /// FrmR - This form is for instructions of the format R.
      FrmR = 1,
      /// FrmI - This form is for instructions of the format I.
      FrmI = 2,
      /// FrmJ - This form is for instructions of the format J.
      FrmS = 3,
      /// FrmOther - This form is for instructions that have no specific format.
      FrmU = 4,
      FormMask = 15
    };
  }
}
