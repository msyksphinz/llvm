//===-- MYRISCVXTargetMachine.cpp - Define TargetMachine for MYRISCVX -------------===//
//
// The LLVM Compiler Infrastructure
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
#include "MYRISCVXTargetObjectFile.h"
#include "MYRISCVXSEISelDAGToDAG.h"
#include "MYRISCVX.h"

#include "llvm/IR/LegacyPassManager.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

#define DEBUG_TYPE "myriscvx"
extern "C" void LLVMInitializeMYRISCVXTarget() {
  // Register the target.
  RegisterTargetMachine<MYRISCVXTargetMachine> X(TheMYRISCVX32Target);
  RegisterTargetMachine<MYRISCVXTargetMachine> Y(TheMYRISCVX64Target);
}


static std::string computeDataLayout(const Triple &TT, StringRef CPU,
                                     const TargetOptions &Options) {
  std::string Ret = "";
  // There are both little and big endian MYRISCVX.
  Ret += "e";
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


static Reloc::Model getEffectiveRelocModel(Optional<CodeModel::Model> CM,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue())
    return Reloc::Static;
  return *RM;
}


static CodeModel::Model getEffectiveCodeModel(Optional<CodeModel::Model> CM) {
  if (CM)
    return *CM;
  return CodeModel::Small;
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
                                             CodeGenOpt::Level OL, bool JIT)
    //- Default is big endian
    : LLVMTargetMachine(T, computeDataLayout(TT, CPU, Options), TT, CPU, FS, Options,
                        getEffectiveRelocModel(CM, RM),
                        getEffectiveCodeModel(CM), OL),
      TLOF(make_unique<MYRISCVXTargetObjectFile>()),
      ABI(MYRISCVXABIInfo::computeTargetABI()),
      DefaultSubtarget(TT, CPU, FS, *this) {
  // initAsmInfo will display features by llc -march=MYRISCVX -mcpu=help on 3.7 but
  // not on 3.6
  initAsmInfo();
}

MYRISCVXTargetMachine::~MYRISCVXTargetMachine() {}

const MYRISCVXSubtarget *
MYRISCVXTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr  = F.getFnAttribute("target-features");
  std::string CPU   = !CPUAttr.hasAttribute(Attribute::None)
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
    I = llvm::make_unique<MYRISCVXSubtarget>(TargetTriple, CPU, FS, *this);
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
  bool addInstSelector() override;
  const MYRISCVXSubtarget &getMYRISCVXSubtarget() const {
    return *getMYRISCVXTargetMachine().getSubtargetImpl();
  }

  void addPreEmitPass() override;
};

} // namespace

// Install an instruction selector pass using
// the ISelDag to gen MYRISCVX code.
bool MYRISCVXPassConfig::addInstSelector() {
  addPass(createMYRISCVXSEISelDag(getMYRISCVXTargetMachine(), getOptLevel()));

  return false;
}

TargetPassConfig *MYRISCVXTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new MYRISCVXPassConfig(*this, PM);
}


// Implemented by targets that want to run passes immediately before
// machine code is emitted. return true if -print-machineinstrs should
// print out the code after the passes.
void MYRISCVXPassConfig::addPreEmitPass() {
  MYRISCVXTargetMachine &TM = getMYRISCVXTargetMachine();
  addPass(createMYRISCVXLongBranchPass(TM));
  addPass(createMYRISCVXDelJmpPass(TM));
  return;
}
