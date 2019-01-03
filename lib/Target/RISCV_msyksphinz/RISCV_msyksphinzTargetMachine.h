//===-- Cpu0TargetMachine.h - Define TargetMachine for Cpu0 -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the RISCV_msyksphinz specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_CPU0_CPU0TARGETMACHINE_H
#define LLVM_LIB_TARGET_CPU0_CPU0TARGETMACHINE_H

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class formatted_raw_ostream;
class RISCV_msyksphinzRegisterInfo;

class RISCV_msyksphinzTargetMachine : public LLVMTargetMachine {
  bool isLittle;
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  RISCV_msyksphinzSubtarget Subtarget;

  mutable StringMap<std::unique_ptr<RISCV_msyksphinzSubtarget>> SubtargetMap;
public:
  RISCV_msyksphinzTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                    StringRef FS, const TargetOptions &Options,
                    Optional<Reloc::Model> RM, CodeModel::Model CM,
                    CodeGenOpt::Level OL, bool isLittle);
  ~RISCV_msyksphinzTargetMachine() override;

  const RISCV_msyksphinzSubtarget *getSubtargetImpl() const {
    return &Subtarget;
  }

  const RISCV_msyksphinzSubtarget *getSubtargetImpl(const Function &F) const override;

  // Pass Pipeline Configuration
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }
  bool isLittleEndian() const { return isLittle; }
  const RISCV_msyksphinzABIInfo &getABI() const { return ABI; }
};

/// RISCV_msyksphinzebTargetMachine - RISCV_msyksphinz32 big endian target machine.
///
class RISCV_msyksphinzebTargetMachine : public RISCV_msyksphinzTargetMachine {
  virtual void anchor();
public:
  RISCV_msyksphinzebTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, CodeModel::Model CM,
                      CodeGenOpt::Level OL);
};

/// RISCV_msyksphinzelTargetMachine - RISCV_msyksphinz32 little endian target machine.
///
class RISCV_msyksphinzelTargetMachine : public RISCV_msyksphinzTargetMachine {
  virtual void anchor();
public:
  RISCV_msyksphinzelTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, CodeModel::Model CM,
                      CodeGenOpt::Level OL);
};
} // End llvm namespace

#endif
