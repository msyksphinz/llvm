//===-- MYRISCVXMCTargetDesc.h - MYRISCVX Target Descriptions -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides MYRISCVX specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MCTARGETDESC_MYRISCVXMCTARGETDESC_H
#define LLVM_LIB_TARGET_MYRISCVX_MCTARGETDESC_MYRISCVXMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

namespace llvm {
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class StringRef;
class Target;
class Triple;
class raw_ostream;
class raw_pwrite_stream;

Target &getTheMYRISCVX32Target();
Target &getTheMYRISCVX64Target();

} // End llvm namespace

// Defines symbolic names for MYRISCVX registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "MYRISCVXGenRegisterInfo.inc"

// Defines symbolic names for the MYRISCVX instructions.
#define GET_INSTRINFO_ENUM
#include "MYRISCVXGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "MYRISCVXGenSubtargetInfo.inc"

#endif
