//===-- MYRISCVXTargetMachine.cpp - Define TargetMachine for MYRISCVX -------------===//
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

#include "MYRISCVXTargetMachine.h"
#include "MYRISCVX.h"

#include "MYRISCVXSEISelDAGToDAG.h"

#include "MYRISCVXSubtarget.h"
#include "MYRISCVXTargetObjectFile.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

extern "C" void LLVMInitializeMYRISCVXTarget() {
  // Register the target.
  //- Little endian Target Machine
  RegisterTargetMachine<MYRISCVXelTargetMachine> Y(TheMYRISCVXelTarget);
}

static std::string computeDataLayout(const Triple &TT, StringRef CPU,
                                     const TargetOptions &Options,
                                     bool isLittle) {
  std::string Ret = "";
  Ret += "e";
  Ret += "-m:m";
  Ret += "-p:32:32-i64:64-i128:128-n64-S128";
  return Ret;
}

static Reloc::Model getEffectiveRelocModel(bool JIT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue() /* || CM == CodeModel::JITDefault */)
    return Reloc::Static;
  return *RM;
}

static CodeModel::Model getEffectiveCodeModel(Optional<CodeModel::Model> CM) {
  if (CM)
    return *CM;
  return CodeModel::Small;
}


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
                        getEffectiveCodeModel(CM), OL),
      isLittle(isLittle), TLOF(make_unique<MYRISCVXTargetObjectFile>()),
      ABI(MYRISCVXABIInfo::computeTargetABI()),
      Subtarget(TT, CPU, FS, isLittle, *this) {
  // initAsmInfo will display features by llc -march=cpu0 -mcpu=help on 3.7 but
  // not on 3.6
  initAsmInfo();
}

MYRISCVXTargetMachine::~MYRISCVXTargetMachine() {}

void MYRISCVXelTargetMachine::anchor() { }

MYRISCVXelTargetMachine::MYRISCVXelTargetMachine(const Target &T, const Triple &TT,
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

  FS = "+myriscvx64";

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

  bool addInstSelector() override;
};
} // namespace

TargetPassConfig *MYRISCVXTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new MYRISCVXPassConfig(*this, PM);
}

// Install an instruction selector pass using
// the ISelDag to gen MYRISCVX code.
bool MYRISCVXPassConfig::addInstSelector() {
  addPass(createMYRISCVXSEISelDag(getMYRISCVXTargetMachine(), getOptLevel()));
  return false;
}
