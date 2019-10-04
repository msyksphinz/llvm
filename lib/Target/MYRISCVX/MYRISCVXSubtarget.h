//===-- MYRISCVXSubtarget.h - Define Subtarget for the MYRISCVX ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===-----------------------------------------------------------------------===//
//
// This file declares the MYRISCVX specific subclass of TargetSubtargetInfo.
//
//===-----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSUBTARGET_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSUBTARGET_H

#include "MYRISCVXFrameLowering.h"
#include "MYRISCVXISelLowering.h"
#include "MYRISCVXInstrInfo.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/MC/MCInstrItineraries.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include <string>

#define GET_SUBTARGETINFO_HEADER
#include "MYRISCVXGenSubtargetInfo.inc"

namespace llvm {
  class StringRef;

  class MYRISCVXTargetMachine;

  class MYRISCVXSubtarget : public MYRISCVXGenSubtargetInfo {
    virtual void anchor();

 public:

 protected:
    bool HasRV64 = false;  // Set by ParseSubtargetFeatures
    MVT XLenVT = MVT::i32;

    // MYRISCVX architecture version
    InstrItineraryData InstrItins;

    const MYRISCVXTargetMachine &TM;

    Triple TargetTriple;

    const SelectionDAGTargetInfo TSInfo;

    MYRISCVXInstrInfo InstrInfo;
    MYRISCVXFrameLowering FrameLowering;
    MYRISCVXTargetLowering TLInfo;
    MYRISCVXRegisterInfo RegInfo;

 public:
    bool isPositionIndependent() const;
    const MYRISCVXABIInfo &getABI() const;

    MVT getXLenVT() const { return XLenVT; }
    bool is64Bit() const { return HasRV64; }

    unsigned getGPRSizeInBytes() const { return is64Bit() ? 8 : 4; }

    /// This constructor initializes the data members to match that
    /// of the specified triple.
    MYRISCVXSubtarget(const Triple &TT, const std::string &CPU, const std::string &FS,
                      const MYRISCVXTargetMachine &_TM);

    //- Vitual function, must have
    /// ParseSubtargetFeatures - Parses features string setting specified
    /// subtarget options.  Definition of function is auto generated by tblgen.
    void ParseSubtargetFeatures(StringRef CPU, StringRef FS);

    bool abiUsesSoftFloat() const;

    unsigned stackAlignment() const { return 8; }

    MYRISCVXSubtarget &initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                                       const TargetMachine &TM);

    const SelectionDAGTargetInfo *getSelectionDAGInfo() const override { return &TSInfo; }
    const MYRISCVXInstrInfo *getInstrInfo() const override { return &InstrInfo; }
    const TargetFrameLowering *getFrameLowering() const override { return &FrameLowering; }
    const MYRISCVXRegisterInfo *getRegisterInfo() const override {
      return &RegInfo;
    }
    const MYRISCVXTargetLowering *getTargetLowering() const override { return &TLInfo; }
    const InstrItineraryData *getInstrItineraryData() const override { return &InstrItins; }

    bool isABI_LP32()    const { return getXLenVT() == MVT::i32 && getABI().IsLP();    }
    bool isABI_STACK32() const { return getXLenVT() == MVT::i32 && getABI().IsSTACK(); }
    bool isABI_LP64()    const { return getXLenVT() == MVT::i64 && getABI().IsLP();    }
    bool isABI_STACK64() const { return getXLenVT() == MVT::i64 && getABI().IsSTACK(); }
  };
} // End llvm namespace


#endif
