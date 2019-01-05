//===-- MYRISCVXSubtarget.h - Define Subtarget for the MYRISCVX ---------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the MYRISCVX specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSUBTARGET_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSUBTARGET_H

#include "MYRISCVXFrameLowering.h"
// #include "MYRISCVXISelLowering.h"
// #include "MYRISCVXInstrInfo.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/MC/MCInstrItineraries.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include <string>

#define GET_SUBTARGETINFO_HEADER
#include "MYRISCVXGenSubtargetInfo.inc"

//@1
namespace llvm {
class StringRef;

class MYRISCVXTargetMachine;

class MYRISCVXSubtarget : public MYRISCVXGenSubtargetInfo {
  virtual void anchor();

public:
  bool HasArchX64;

protected:
  // IsLittle - The target is Little Endian
  bool IsLittle;

  InstrItineraryData InstrItins;

  const MYRISCVXTargetMachine &TM;

  Triple TargetTriple;

  const SelectionDAGTargetInfo TSInfo;

  // std::unique_ptr<const MYRISCVXInstrInfo> InstrInfo;
  std::unique_ptr<const MYRISCVXFrameLowering> FrameLowering;
  // std::unique_ptr<const MYRISCVXTargetLowering> TLInfo;

public:
  bool isPositionIndependent() const;
  // const MYRISCVXABIInfo &getABI() const;

  /// This constructor initializes the data members to match that
  /// of the specified triple.
  MYRISCVXSubtarget(const Triple &TT, const std::string &CPU, const std::string &FS,
                bool little, const MYRISCVXTargetMachine &_TM);

//- Vitual function, must have
  /// ParseSubtargetFeatures - Parses features string setting specified
  /// subtarget options.  Definition of function is auto generated by tblgen.
  void ParseSubtargetFeatures(StringRef CPU, StringRef FS);

  bool isLittle() const { return IsLittle; }

  bool abiUsesSoftFloat() const;

  unsigned stackAlignment() const { return 8; }

  MYRISCVXSubtarget &initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                                 const TargetMachine &TM);

  const SelectionDAGTargetInfo *getSelectionDAGInfo() const override {
    return &TSInfo;
  }
  // const MYRISCVXInstrInfo *getInstrInfo() const override { return InstrInfo.get(); }
  const TargetFrameLowering *getFrameLowering() const override {
    return FrameLowering.get();
  }
  // const MYRISCVXRegisterInfo *getRegisterInfo() const override {
  //   return &InstrInfo->getRegisterInfo();
  // }
  // const MYRISCVXTargetLowering *getTargetLowering() const override {
  //   return TLInfo.get();
  // }
  const InstrItineraryData *getInstrItineraryData() const override {
    return &InstrItins;
  }
};
} // End llvm namespace

#endif
