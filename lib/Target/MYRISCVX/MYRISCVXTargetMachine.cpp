//===-- MYRISCVXTargetMachine.cpp - Define TargetMachine for MYRISCVX ------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Implements the info about MYRISCVX target spec.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVX.h"
#include "MYRISCVXTargetMachine.h"
#include "MYRISCVXTargetObjectFile.h"

#include "llvm/IR/LegacyPassManager.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

extern "C" void LLVMInitializeMYRISCVXTarget() {
  // Register the target.
  //- Little endian Target Machine
  RegisterTargetMachine<MYRISCVX32TargetMachine> X(TheMYRISCVX32Target);
  RegisterTargetMachine<MYRISCVX64TargetMachine> Y(TheMYRISCVX64Target);
}


static std::string computeDataLayout(const Triple &TT, StringRef CPU,
                                     const TargetOptions &Options,
                                     bool isLittle) {
  std::string Ret = "";
  // There are both little and big endian MYRISCVX.
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

static Reloc::Model getEffectiveRelocModel(bool JIT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue() || JIT)
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
MYRISCVXTargetMachine::MYRISCVXTargetMachine(const Target &T, const Triple &TT,
                                             StringRef CPU, StringRef FS,
                                             const TargetOptions &Options,
                                             Optional<Reloc::Model> RM,
                                             Optional<CodeModel::Model> CM,
                                             CodeGenOpt::Level OL, bool JIT,
                                             bool isLittle)
    //- Default is big endian
    : LLVMTargetMachine(T, computeDataLayout(TT, CPU, Options, isLittle), TT,
                        CPU, FS, Options, getEffectiveRelocModel(JIT, RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      isLittle(isLittle), TLOF(make_unique<MYRISCVXTargetObjectFile>()),
      ABI(MYRISCVXABIInfo::computeTargetABI()),
      DefaultSubtarget(TT, CPU, FS, isLittle, *this) {
  // initAsmInfo will display features by llc -march=MYRISCVX -mcpu=help on 3.7 but
  // not on 3.6
  initAsmInfo();
}

MYRISCVXTargetMachine::~MYRISCVXTargetMachine() {}

void MYRISCVX32TargetMachine::anchor() { }

MYRISCVX32TargetMachine::MYRISCVX32TargetMachine(const Target &T, const Triple &TT,
                                                 StringRef CPU, StringRef FS,
                                                 const TargetOptions &Options,
                                                 Optional<Reloc::Model> RM,
                                                 Optional<CodeModel::Model> CM,
                                                 CodeGenOpt::Level OL, bool JIT)
    : MYRISCVXTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, true) {}

void MYRISCVX64TargetMachine::anchor() { }

MYRISCVX64TargetMachine::MYRISCVX64TargetMachine(const Target &T, const Triple &TT,
                                                 StringRef CPU, StringRef FS,
                                                 const TargetOptions &Options,
                                                 Optional<Reloc::Model> RM,
                                                 Optional<CodeModel::Model> CM,
                                                 CodeGenOpt::Level OL, bool JIT)
    : MYRISCVXTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, true) {}

const MYRISCVXSubtarget *
MYRISCVXTargetMachine::getSubtargetImpl(const Function &F) const {
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
    I = llvm::make_unique<MYRISCVXSubtarget>(TargetTriple, CPU, FS, isLittle,
                                             *this);
  }
  return I.get();
}

namespace {
//@MYRISCVXPassConfig {
/// MYRISCVX Code Generator Pass Configuration Options.
class MYRISCVXPassConfig : public TargetPassConfig {
 public:
  MYRISCVXPassConfig(MYRISCVXTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  MYRISCVXTargetMachine &getMYRISCVXTargetMachine() const {
    return getTM<MYRISCVXTargetMachine>();
  }

  const MYRISCVXSubtarget &getMYRISCVXSubtarget() const {
    return *getMYRISCVXTargetMachine().getSubtargetImpl();
  }
};
} // namespace

TargetPassConfig *MYRISCVXTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new MYRISCVXPassConfig(*this, PM);
}
