//===-- MYRISCVXInstrInfo.h - MYRISCVX Instruction Information ----------*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#pragma once

#include "MYRISCVX.h"
#include "MYRISCVXRegisterInfo.h"
#include "MYRISCVXAnalyzeImmediate.h"

#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "MYRISCVXGenInstrInfo.inc"

namespace llvm {
  class MYRISCVXInstrInfo : public MYRISCVXGenInstrInfo {
    virtual void anchor();
 protected:
    const MYRISCVXSubtarget &Subtarget;
 public:
    explicit MYRISCVXInstrInfo(const MYRISCVXSubtarget &STI);
    static const MYRISCVXInstrInfo *create(MYRISCVXSubtarget &STI);
    /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info. As
    /// such, whenever a client has an instance of instruction info, it should
    /// always be able to get register info as well (through this method).
    ///
    virtual const MYRISCVXRegisterInfo &getRegisterInfo() const = 0;
    /// Return the number of bytes of code the specified instruction may be.
    unsigned GetInstSizeInBytes(const MachineInstr &MI) const;
 protected:
  };
  const MYRISCVXInstrInfo *createMYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI);
}
