//===-- RISCV64TargetMachine.cpp - Define TargetMachine for RISCV64 -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Implements the info about RISCV64 target spec.
//
//===----------------------------------------------------------------------===//

#include "RISCV64TargetMachine.h"
#include "MCTargetDesc/RISCV64ABIInfo.h"
#include "MCTargetDesc/RISCV64MCTargetDesc.h"
#include "RISCV64.h"
#include "RISCV6416ISelDAGToDAG.h"
#include "RISCV64SEISelDAGToDAG.h"
#include "RISCV64Subtarget.h"
#include "RISCV64TargetObjectFile.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/BasicTTIImpl.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetOptions.h"
#include <string>

using namespace llvm;

#define DEBUG_TYPE "mips"

extern "C" void LLVMInitializeRISCV64Target() {
  // Register the target.
  RegisterTargetMachine<RISCV64ebTargetMachine> X(getTheRISCV64Target());
  RegisterTargetMachine<RISCV64elTargetMachine> Y(getTheRISCV64elTarget());
  RegisterTargetMachine<RISCV64ebTargetMachine> A(getTheRISCV6464Target());
  RegisterTargetMachine<RISCV64elTargetMachine> B(getTheRISCV6464elTarget());

  PassRegistry *PR = PassRegistry::getPassRegistry();
  initializeGlobalISel(*PR);
  initializeRISCV64DelaySlotFillerPass(*PR);
  initializeRISCV64BranchExpansionPass(*PR);
  initializeMicroRISCV64SizeReducePass(*PR);
}

static std::string computeDataLayout(const Triple &TT, StringRef CPU,
                                     const TargetOptions &Options,
                                     bool isLittle) {
  std::string Ret;
  RISCV64ABIInfo ABI = RISCV64ABIInfo::computeTargetABI(TT, CPU, Options.MCOptions);

  // There are both little and big endian mips.
  if (isLittle)
    Ret += "e";
  else
    Ret += "E";

  if (ABI.IsO32())
    Ret += "-m:m";
  else
    Ret += "-m:e";

  // Pointers are 32 bit on some ABIs.
  if (!ABI.IsN64())
    Ret += "-p:32:32";

  // 8 and 16 bit integers only need to have natural alignment, but try to
  // align them to 32 bits. 64 bit integers have natural alignment.
  Ret += "-i8:8:32-i16:16:32-i64:64";

  // 32 bit registers are always available and the stack is at least 64 bit
  // aligned. On N64 64 bit registers are also available and the stack is
  // 128 bit aligned.
  if (ABI.IsN64() || ABI.IsN32())
    Ret += "-n32:64-S128";
  else
    Ret += "-n32-S64";

  return Ret;
}

static Reloc::Model getEffectiveRelocModel(bool JIT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue() || JIT)
    return Reloc::Static;
  return *RM;
}

static CodeModel::Model getEffectiveCodeModel(Optional<CodeModel::Model> CM) {
  if (CM)
    return *CM;
  return CodeModel::Small;
}

// On function prologue, the stack is created by decrementing
// its pointer. Once decremented, all references are done with positive
// offset from the stack/frame pointer, using StackGrowsUp enables
// an easier handling.
// Using CodeModel::Large enables different CALL behavior.
RISCV64TargetMachine::RISCV64TargetMachine(const Target &T, const Triple &TT,
                                     StringRef CPU, StringRef FS,
                                     const TargetOptions &Options,
                                     Optional<Reloc::Model> RM,
                                     Optional<CodeModel::Model> CM,
                                     CodeGenOpt::Level OL, bool JIT,
                                     bool isLittle)
    : LLVMTargetMachine(T, computeDataLayout(TT, CPU, Options, isLittle), TT,
                        CPU, FS, Options, getEffectiveRelocModel(JIT, RM),
                        getEffectiveCodeModel(CM), OL),
      isLittle(isLittle), TLOF(llvm::make_unique<RISCV64TargetObjectFile>()),
      ABI(RISCV64ABIInfo::computeTargetABI(TT, CPU, Options.MCOptions)),
      Subtarget(nullptr), DefaultSubtarget(TT, CPU, FS, isLittle, *this,
                                           Options.StackAlignmentOverride),
      NoRISCV6416Subtarget(TT, CPU, FS.empty() ? "-mips16" : FS.str() + ",-mips16",
                        isLittle, *this, Options.StackAlignmentOverride),
      RISCV6416Subtarget(TT, CPU, FS.empty() ? "+mips16" : FS.str() + ",+mips16",
                      isLittle, *this, Options.StackAlignmentOverride) {
  Subtarget = &DefaultSubtarget;
  initAsmInfo();
}

RISCV64TargetMachine::~RISCV64TargetMachine() = default;

void RISCV64ebTargetMachine::anchor() {}

RISCV64ebTargetMachine::RISCV64ebTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         Optional<CodeModel::Model> CM,
                                         CodeGenOpt::Level OL, bool JIT)
    : RISCV64TargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, false) {}

void RISCV64elTargetMachine::anchor() {}

RISCV64elTargetMachine::RISCV64elTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         Optional<CodeModel::Model> CM,
                                         CodeGenOpt::Level OL, bool JIT)
    : RISCV64TargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, true) {}

const RISCV64Subtarget *
RISCV64TargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  std::string CPU = !CPUAttr.hasAttribute(Attribute::None)
                        ? CPUAttr.getValueAsString().str()
                        : TargetCPU;
  std::string FS = !FSAttr.hasAttribute(Attribute::None)
                       ? FSAttr.getValueAsString().str()
                       : TargetFS;
  bool hasRISCV6416Attr =
      !F.getFnAttribute("mips16").hasAttribute(Attribute::None);
  bool hasNoRISCV6416Attr =
      !F.getFnAttribute("nomips16").hasAttribute(Attribute::None);

  bool HasMicroRISCV64Attr =
      !F.getFnAttribute("micromips").hasAttribute(Attribute::None);
  bool HasNoMicroRISCV64Attr =
      !F.getFnAttribute("nomicromips").hasAttribute(Attribute::None);

  // FIXME: This is related to the code below to reset the target options,
  // we need to know whether or not the soft float flag is set on the
  // function, so we can enable it as a subtarget feature.
  bool softFloat =
      F.hasFnAttribute("use-soft-float") &&
      F.getFnAttribute("use-soft-float").getValueAsString() == "true";

  if (hasRISCV6416Attr)
    FS += FS.empty() ? "+mips16" : ",+mips16";
  else if (hasNoRISCV6416Attr)
    FS += FS.empty() ? "-mips16" : ",-mips16";
  if (HasMicroRISCV64Attr)
    FS += FS.empty() ? "+micromips" : ",+micromips";
  else if (HasNoMicroRISCV64Attr)
    FS += FS.empty() ? "-micromips" : ",-micromips";
  if (softFloat)
    FS += FS.empty() ? "+soft-float" : ",+soft-float";

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = llvm::make_unique<RISCV64Subtarget>(TargetTriple, CPU, FS, isLittle, *this,
                                         Options.StackAlignmentOverride);
  }
  return I.get();
}

void RISCV64TargetMachine::resetSubtarget(MachineFunction *MF) {
  LLVM_DEBUG(dbgs() << "resetSubtarget\n");

  Subtarget = const_cast<RISCV64Subtarget *>(getSubtargetImpl(MF->getFunction()));
  MF->setSubtarget(Subtarget);
}

namespace {

/// RISCV64 Code Generator Pass Configuration Options.
class RISCV64PassConfig : public TargetPassConfig {
public:
  RISCV64PassConfig(RISCV64TargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {
    // The current implementation of long branch pass requires a scratch
    // register ($at) to be available before branch instructions. Tail merging
    // can break this requirement, so disable it when long branch pass is
    // enabled.
    EnableTailMerge = !getRISCV64Subtarget().enableLongBranchPass();
  }

  RISCV64TargetMachine &getRISCV64TargetMachine() const {
    return getTM<RISCV64TargetMachine>();
  }

  const RISCV64Subtarget &getRISCV64Subtarget() const {
    return *getRISCV64TargetMachine().getSubtargetImpl();
  }

  void addIRPasses() override;
  bool addInstSelector() override;
  void addPreEmitPass() override;
  void addPreRegAlloc() override;
  bool addIRTranslator() override;
  bool addLegalizeMachineIR() override;
  bool addRegBankSelect() override;
  bool addGlobalInstructionSelect() override;
};

} // end anonymous namespace

TargetPassConfig *RISCV64TargetMachine::createPassConfig(PassManagerBase &PM) {
  return new RISCV64PassConfig(*this, PM);
}

void RISCV64PassConfig::addIRPasses() {
  TargetPassConfig::addIRPasses();
  addPass(createAtomicExpandPass());
  if (getRISCV64Subtarget().os16())
    addPass(createRISCV64Os16Pass());
  if (getRISCV64Subtarget().inRISCV6416HardFloat())
    addPass(createRISCV6416HardFloatPass());
}
// Install an instruction selector pass using
// the ISelDag to gen RISCV64 code.
bool RISCV64PassConfig::addInstSelector() {
  addPass(createRISCV64ModuleISelDagPass());
  addPass(createRISCV6416ISelDag(getRISCV64TargetMachine(), getOptLevel()));
  addPass(createRISCV64SEISelDag(getRISCV64TargetMachine(), getOptLevel()));
  return false;
}

void RISCV64PassConfig::addPreRegAlloc() {
  addPass(createRISCV64OptimizePICCallPass());
}

TargetTransformInfo
RISCV64TargetMachine::getTargetTransformInfo(const Function &F) {
  if (Subtarget->allowMixed16_32()) {
    LLVM_DEBUG(errs() << "No Target Transform Info Pass Added\n");
    // FIXME: This is no longer necessary as the TTI returned is per-function.
    return TargetTransformInfo(F.getParent()->getDataLayout());
  }

  LLVM_DEBUG(errs() << "Target Transform Info Pass Added\n");
  return TargetTransformInfo(BasicTTIImpl(this, F));
}

// Implemented by targets that want to run passes immediately before
// machine code is emitted. return true if -print-machineinstrs should
// print out the code after the passes.
void RISCV64PassConfig::addPreEmitPass() {
  // Expand pseudo instructions that are sensitive to register allocation.
  addPass(createRISCV64ExpandPseudoPass());

  // The microMIPS size reduction pass performs instruction reselection for
  // instructions which can be remapped to a 16 bit instruction.
  addPass(createMicroRISCV64SizeReducePass());

  // The delay slot filler pass can potientially create forbidden slot hazards
  // for MIPSR6 and therefore it should go before RISCV64BranchExpansion pass.
  addPass(createRISCV64DelaySlotFillerPass());

  // This pass expands branches and takes care about the forbidden slot hazards.
  // Expanding branches may potentially create forbidden slot hazards for
  // MIPSR6, and fixing such hazard may potentially break a branch by extending
  // its offset out of range. That's why this pass combine these two tasks, and
  // runs them alternately until one of them finishes without any changes. Only
  // then we can be sure that all branches are expanded properly and no hazards
  // exists.
  // Any new pass should go before this pass.
  addPass(createRISCV64BranchExpansion());

  addPass(createRISCV64ConstantIslandPass());
}

bool RISCV64PassConfig::addIRTranslator() {
  addPass(new IRTranslator());
  return false;
}

bool RISCV64PassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

bool RISCV64PassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

bool RISCV64PassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect());
  return false;
}
