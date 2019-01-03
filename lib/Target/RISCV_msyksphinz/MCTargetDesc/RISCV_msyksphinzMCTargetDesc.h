//===-- RISCV_msyksphinzMCTargetDesc.h - RISCV_msyksphinz Target Descriptions -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides RISCV_msyksphinz specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_CPU0_MCTARGETDESC_CPU0MCTARGETDESC_H
#define LLVM_LIB_TARGET_CPU0_MCTARGETDESC_CPU0MCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

namespace llvm {
class Target;
class Triple;

extern Target TheRISCV_msyksphinzTarget;
extern Target TheRISCV_msyksphinzelTarget;

} // End llvm namespace

// Defines symbolic names for RISCV_msyksphinz registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "RISCV_msyksphinzGenRegisterInfo.inc"

// Defines symbolic names for the RISCV_msyksphinz instructions.
#define GET_INSTRINFO_ENUM
#include "RISCV_msyksphinzGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "RISCV_msyksphinzGenSubtargetInfo.inc"

#endif
