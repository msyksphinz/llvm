//===- RISCV64TargetMachine.h - Define TargetMachine for RISCV64 ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the RISCV64 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MIPS_MIPSTARGETMACHINE_H
#define LLVM_LIB_TARGET_MIPS_MIPSTARGETMACHINE_H

#include "MCTargetDesc/RISCV64ABIInfo.h"
#include "RISCV64Subtarget.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

namespace llvm {

class RISCV64TargetMachine : public LLVMTargetMachine {
  bool isLittle;
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  // Selected ABI
  RISCV64ABIInfo ABI;
  RISCV64Subtarget *Subtarget;
  RISCV64Subtarget DefaultSubtarget;
  RISCV64Subtarget NoRISCV6416Subtarget;
  RISCV64Subtarget RISCV6416Subtarget;

  mutable StringMap<std::unique_ptr<RISCV64Subtarget>> SubtargetMap;

public:
  RISCV64TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                    StringRef FS, const TargetOptions &Options,
                    Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                    CodeGenOpt::Level OL, bool JIT, bool isLittle);
  ~RISCV64TargetMachine() override;

  TargetTransformInfo getTargetTransformInfo(const Function &F) override;

  const RISCV64Subtarget *getSubtargetImpl() const {
    if (Subtarget)
      return Subtarget;
    return &DefaultSubtarget;
  }

  const RISCV64Subtarget *getSubtargetImpl(const Function &F) const override;

  /// Reset the subtarget for the RISCV64 target.
  void resetSubtarget(MachineFunction *MF);

  // Pass Pipeline Configuration
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }

  bool isLittleEndian() const { return isLittle; }
  const RISCV64ABIInfo &getABI() const { return ABI; }

  bool isMachineVerifierClean() const override {
    return false;
  }
};

/// RISCV6432/64 big endian target machine.
///
class RISCV64ebTargetMachine : public RISCV64TargetMachine {
  virtual void anchor();

public:
  RISCV64ebTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                      CodeGenOpt::Level OL, bool JIT);
};

/// RISCV6432/64 little endian target machine.
///
class RISCV64elTargetMachine : public RISCV64TargetMachine {
  virtual void anchor();

public:
  RISCV64elTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                      CodeGenOpt::Level OL, bool JIT);
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MIPS_MIPSTARGETMACHINE_H
