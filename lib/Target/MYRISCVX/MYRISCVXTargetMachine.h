//===-- Cpu0TargetMachine.h - Define TargetMachine for Cpu0 -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the MYRISCVX specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXTARGETMACHINE_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXTARGETMACHINE_H

#include "MCTargetDesc/MYRISCVXABIInfo.h"
#include "MYRISCVXSubtarget.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class formatted_raw_ostream;
class MYRISCVXRegisterInfo;

class MYRISCVXTargetMachine : public LLVMTargetMachine {
  bool isLittle;
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  // Selected ABI
  MYRISCVXABIInfo ABI;
  MYRISCVXSubtarget DefaultSubtarget;

  mutable StringMap<std::unique_ptr<MYRISCVXSubtarget>> SubtargetMap;
 public:
  MYRISCVXTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                        StringRef FS, const TargetOptions &Options,
                        Optional<Reloc::Model> RM,
                        Optional<CodeModel::Model> CM,
                        CodeGenOpt::Level OL, bool JIT,
                        bool isLittle);
  ~MYRISCVXTargetMachine() override;

  const MYRISCVXSubtarget *getSubtargetImpl() const {
    return &DefaultSubtarget;
  }

  const MYRISCVXSubtarget *getSubtargetImpl(const Function &F) const override;

  // Pass Pipeline Configuration
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }
  bool isLittleEndian() const { return isLittle; }
  const MYRISCVXABIInfo &getABI() const { return ABI; }
};

/// MYRISCVX32TargetMachine - MYRISCVX32 little endian target machine.
///
class MYRISCVX32TargetMachine : public MYRISCVXTargetMachine {
  virtual void anchor();
 public:
  MYRISCVX32TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                          StringRef FS, const TargetOptions &Options,
                          Optional<Reloc::Model> RM,
                          Optional<CodeModel::Model> CM,
                          CodeGenOpt::Level OL, bool JIT);
};

/// MYRISCVX64TargetMachine - MYRISCVX64 little endian target machine.
///
class MYRISCVX64TargetMachine : public MYRISCVXTargetMachine {
  virtual void anchor();
 public:
  MYRISCVX64TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                          StringRef FS, const TargetOptions &Options,
                          Optional<Reloc::Model> RM,
                          Optional<CodeModel::Model> CM,
                          CodeGenOpt::Level OL, bool JIT);
};
} // End llvm namespace

#endif // LLVM_LIB_TARGET_MYRISCVX_MYRISCVXTARGETMACHINE_H
