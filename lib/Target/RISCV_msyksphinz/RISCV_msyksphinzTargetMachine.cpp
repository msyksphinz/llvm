//===-- RISCV_msyksphinzTargetMachine.cpp - Define TargetMachine for RISCV_msyksphinz -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Implements the info about RISCV_msyksphinz target spec.
//
//===----------------------------------------------------------------------===//

#include "RISCV_msyksphinzTargetMachine.h"
#include "RISCV_msyksphinz.h"

#include "RISCV_msyksphinzSubtarget.h"
#include "RISCV_msyksphinzTargetObjectFile.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

#define DEBUG_TYPE "cpu0"

extern "C" void LLVMInitializeRISCV_msyksphinzTarget() {
  // Register the target.
  //- Big endian Target Machine
  RegisterTargetMachine<RISCV_msyksphinzebTargetMachine> X(TheRISCV_msyksphinzTarget);
  //- Little endian Target Machine
  RegisterTargetMachine<RISCV_msyksphinzelTargetMachine> Y(TheRISCV_msyksphinzelTarget);
}

static std::string computeDataLayout(const Triple &TT, StringRef CPU,
                                     const TargetOptions &Options,
                                     bool isLittle) {
  std::string Ret = "";
  // There are both little and big endian cpu0.
  if (isLittle)
    Ret += "e";
  else
    Ret += "E";

  Ret += "-m:m";

  // Pointers are 32 bit on some ABIs.
  Ret += "-p:32:32";

  // 8 and 16 bit integers only need to have natural alignment, but try to
  // align them to 32 bits. 64 bit integers have natural alignment.
  Ret += "-i8:8:32-i16:16:32-i64:64";

  // 32 bit registers are always available and the stack is at least 64 bit
  // aligned.
  Ret += "-n32-S64";

  return Ret;
}

static Reloc::Model getEffectiveRelocModel(CodeModel::Model CM,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue() || CM == CodeModel::JITDefault)
    return Reloc::Static;
  return *RM;
}

// DataLayout --> Big-endian, 32-bit pointer/ABI/alignment
// The stack is always 8 byte aligned
// On function prologue, the stack is created by decrementing
// its pointer. Once decremented, all references are done with positive
// offset from the stack/frame pointer, using StackGrowsUp enables
// an easier handling.
// Using CodeModel::Large enables different CALL behavior.
RISCV_msyksphinzTargetMachine::RISCV_msyksphinzTargetMachine(const Target &T, const Triple &TT,
                                     StringRef CPU, StringRef FS,
                                     const TargetOptions &Options,
                                     Optional<Reloc::Model> RM,
                                     CodeModel::Model CM, CodeGenOpt::Level OL,
                                     bool isLittle)
  //- Default is big endian
    : LLVMTargetMachine(T, computeDataLayout(TT, CPU, Options, isLittle), TT,
                        CPU, FS, Options, getEffectiveRelocModel(CM, RM), CM,
                        OL),
      isLittle(isLittle), TLOF(make_unique<RISCV_msyksphinzTargetObjectFile>()),
      ABI(RISCV_msyksphinzABIInfo::computeTargetABI()),
      DefaultSubtarget(TT, CPU, FS, isLittle, *this) {
  // initAsmInfo will display features by llc -march=cpu0 -mcpu=help on 3.7 but
  // not on 3.6
  initAsmInfo();
}

RISCV_msyksphinzTargetMachine::~RISCV_msyksphinzTargetMachine() {}

void RISCV_msyksphinzebTargetMachine::anchor() { }

RISCV_msyksphinzebTargetMachine::RISCV_msyksphinzebTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         CodeModel::Model CM,
                                         CodeGenOpt::Level OL)
    : RISCV_msyksphinzTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, false) {}

void RISCV_msyksphinzelTargetMachine::anchor() { }

RISCV_msyksphinzelTargetMachine::RISCV_msyksphinzelTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         CodeModel::Model CM,
                                         CodeGenOpt::Level OL)
    : RISCV_msyksphinzTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, true) {}

const RISCV_msyksphinzSubtarget *
RISCV_msyksphinzTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  std::string CPU = !CPUAttr.hasAttribute(Attribute::None)
                        ? CPUAttr.getValueAsString().str()
                        : TargetCPU;
  std::string FS = !FSAttr.hasAttribute(Attribute::None)
                       ? FSAttr.getValueAsString().str()
                       : TargetFS;

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = llvm::make_unique<RISCV_msyksphinzSubtarget>(TargetTriple, CPU, FS, isLittle,
                                         *this);
  }
  return I.get();
}

namespace {
//@RISCV_msyksphinzPassConfig {
/// RISCV_msyksphinz Code Generator Pass Configuration Options.
class RISCV_msyksphinzPassConfig : public TargetPassConfig {
public:
  RISCV_msyksphinzPassConfig(RISCV_msyksphinzTargetMachine *TM, PassManagerBase &PM)
    : TargetPassConfig(TM, PM) {}

  RISCV_msyksphinzTargetMachine &getRISCV_msyksphinzTargetMachine() const {
    return getTM<RISCV_msyksphinzTargetMachine>();
  }

  const RISCV_msyksphinzSubtarget &getRISCV_msyksphinzSubtarget() const {
    return *getRISCV_msyksphinzTargetMachine().getSubtargetImpl();
  }
};
} // namespace

TargetPassConfig *RISCV_msyksphinzTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new RISCV_msyksphinzPassConfig(this, PM);
}
