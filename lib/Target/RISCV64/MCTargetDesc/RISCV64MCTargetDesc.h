//===-- RISCV64MCTargetDesc.h - RISCV64 Target Descriptions -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides RISCV64 specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MIPS_MCTARGETDESC_MIPSMCTARGETDESC_H
#define LLVM_LIB_TARGET_MIPS_MCTARGETDESC_MIPSMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

#include <memory>

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

Target &getTheRISCV64Target();
Target &getTheRISCV64elTarget();
Target &getTheRISCV6464Target();
Target &getTheRISCV6464elTarget();

MCCodeEmitter *createRISCV64MCCodeEmitterEB(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx);
MCCodeEmitter *createRISCV64MCCodeEmitterEL(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx);

MCAsmBackend *createRISCV64AsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                   const MCRegisterInfo &MRI,
                                   const MCTargetOptions &Options);

std::unique_ptr<MCObjectTargetWriter>
createRISCV64ELFObjectWriter(const Triple &TT, bool IsN32);

namespace MIPS_MC {
StringRef selectRISCV64CPU(const Triple &TT, StringRef CPU);
}

} // End llvm namespace

// Defines symbolic names for RISCV64 registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "RISCV64GenRegisterInfo.inc"

// Defines symbolic names for the RISCV64 instructions.
#define GET_INSTRINFO_ENUM
#include "RISCV64GenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "RISCV64GenSubtargetInfo.inc"

#endif
