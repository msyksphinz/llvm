//===---- MYRISCVXABIInfo.h - Information about MYRISCVX ABI's ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MCTARGETDESC_MYRISCVXABIINFO_H
#define LLVM_LIB_TARGET_MYRISCVX_MCTARGETDESC_MYRISCVXABIINFO_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/Triple.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {

  class MCTargetOptions;
  class StringRef;
  class TargetRegisterClass;

  class MYRISCVXABIInfo {
   public:
    enum class ABI { Unknown, LP, STACK };

   protected:
    ABI ThisABI;

   public:
    MYRISCVXABIInfo(ABI ThisABI) : ThisABI(ThisABI) {}

    static MYRISCVXABIInfo Unknown() { return MYRISCVXABIInfo(ABI::Unknown); }
    static MYRISCVXABIInfo LP() { return MYRISCVXABIInfo(ABI::LP); }
    static MYRISCVXABIInfo STACK() { return MYRISCVXABIInfo(ABI::STACK); }
    static MYRISCVXABIInfo computeTargetABI(StringRef ABIName);

    bool IsKnown() const { return ThisABI != ABI::Unknown; }
    bool IsLP   () const { return ThisABI == ABI::LP;    }
    bool IsSTACK() const { return ThisABI == ABI::STACK; }
    ABI GetEnumValue() const { return ThisABI; }

    /// The registers to use for byval arguments.
    ArrayRef<MCPhysReg> GetByValArgRegs() const;

    /// The registers to use for the variable argument list.
    ArrayRef<MCPhysReg> GetVarArgRegs() const;

    /// Obtain the size of the area allocated by the callee for arguments.
    /// CallingConv::FastCall affects the value for O32.
    unsigned GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const;

    /// Ordering of ABI's
    /// MYRISCVXGenSubtargetInfo.inc will use this to resolve conflicts when given
    /// multiple ABI options.
    bool operator<(const MYRISCVXABIInfo Other) const {
      return ThisABI < Other.GetEnumValue();
    }

    unsigned GetStackPtr() const;
    unsigned GetFramePtr() const;
    unsigned GetNullPtr() const;

    unsigned GetEhDataReg(unsigned I) const;
    int EhDataRegSize() const;
  };
}

#endif
