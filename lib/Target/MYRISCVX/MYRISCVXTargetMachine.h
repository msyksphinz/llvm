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
  MYRISCVXSubtarget Subtarget;

  mutable StringMap<std::unique_ptr<MYRISCVXSubtarget>> SubtargetMap;
public:
  MYRISCVXTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                    StringRef FS, const TargetOptions &Options,
                    Optional<Reloc::Model> RM, CodeModel::Model CM,
                    CodeGenOpt::Level OL, bool isLittle);
  ~MYRISCVXTargetMachine() override;

  const MYRISCVXSubtarget *getSubtargetImpl() const {
    return &Subtarget;
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

/// MYRISCVXebTargetMachine - MYRISCVX32 big endian target machine.
///
class MYRISCVXebTargetMachine : public MYRISCVXTargetMachine {
  virtual void anchor();
public:
  MYRISCVXebTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, CodeModel::Model CM,
                      CodeGenOpt::Level OL);
};

/// MYRISCVXelTargetMachine - MYRISCVX32 little endian target machine.
///
class MYRISCVXelTargetMachine : public MYRISCVXTargetMachine {
  virtual void anchor();
public:
  MYRISCVXelTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, CodeModel::Model CM,
                      CodeGenOpt::Level OL);
};
} // End llvm namespace

#endif
