//===-- RISCV64Subtarget.cpp - RISCV64 Subtarget Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the RISCV64 specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "RISCV64Subtarget.h"
#include "RISCV64.h"
#include "RISCV64MachineFunction.h"
#include "RISCV64RegisterInfo.h"
#include "RISCV64TargetMachine.h"
#include "RISCV64CallLowering.h"
#include "RISCV64LegalizerInfo.h"
#include "RISCV64RegisterBankInfo.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "mips-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "RISCV64GenSubtargetInfo.inc"

// FIXME: Maybe this should be on by default when RISCV6416 is specified
//
static cl::opt<bool>
    Mixed16_32("mips-mixed-16-32", cl::init(false),
               cl::desc("Allow for a mixture of RISCV6416 "
                        "and RISCV6432 code in a single output file"),
               cl::Hidden);

static cl::opt<bool> RISCV64_Os16("mips-os16", cl::init(false),
                               cl::desc("Compile all functions that don't use "
                                        "floating point as RISCV64 16"),
                               cl::Hidden);

static cl::opt<bool> RISCV6416HardFloat("mips16-hard-float", cl::NotHidden,
                                     cl::desc("Enable mips16 hard float."),
                                     cl::init(false));

static cl::opt<bool>
    RISCV6416ConstantIslands("mips16-constant-islands", cl::NotHidden,
                          cl::desc("Enable mips16 constant islands."),
                          cl::init(true));

static cl::opt<bool>
    GPOpt("mgpopt", cl::Hidden,
          cl::desc("Enable gp-relative addressing of mips small data items"));

bool RISCV64Subtarget::DspWarningPrinted = false;
bool RISCV64Subtarget::MSAWarningPrinted = false;
bool RISCV64Subtarget::VirtWarningPrinted = false;
bool RISCV64Subtarget::CRCWarningPrinted = false;
bool RISCV64Subtarget::GINVWarningPrinted = false;

void RISCV64Subtarget::anchor() {}

RISCV64Subtarget::RISCV64Subtarget(const Triple &TT, StringRef CPU, StringRef FS,
                             bool little, const RISCV64TargetMachine &TM,
                             unsigned StackAlignOverride)
    : RISCV64GenSubtargetInfo(TT, CPU, FS), RISCV64ArchVersion(RISCV64Default),
      IsLittle(little), IsSoftFloat(false), IsSingleFloat(false), IsFPXX(false),
      NoABICalls(false), IsFP64bit(false), UseOddSPReg(true),
      IsNaN2008bit(false), IsGP64bit(false), HasVFPU(false), HasCnRISCV64(false),
      HasRISCV643_32(false), HasRISCV643_32r2(false), HasRISCV644_32(false),
      HasRISCV644_32r2(false), HasRISCV645_32r2(false), InRISCV6416Mode(false),
      InRISCV6416HardFloat(RISCV6416HardFloat), InMicroRISCV64Mode(false), HasDSP(false),
      HasDSPR2(false), HasDSPR3(false), AllowMixed16_32(Mixed16_32 | RISCV64_Os16),
      Os16(RISCV64_Os16), HasMSA(false), UseTCCInDIV(false), HasSym32(false),
      HasEVA(false), DisableMadd4(false), HasMT(false), HasCRC(false),
      HasVirt(false), HasGINV(false), UseIndirectJumpsHazard(false),
      StackAlignOverride(StackAlignOverride),
      TM(TM), TargetTriple(TT), TSInfo(),
      InstrInfo(
          RISCV64InstrInfo::create(initializeSubtargetDependencies(CPU, FS, TM))),
      FrameLowering(RISCV64FrameLowering::create(*this)),
      TLInfo(RISCV64TargetLowering::create(TM, *this)) {

  if (RISCV64ArchVersion == RISCV64Default)
    RISCV64ArchVersion = RISCV6432;

  // Don't even attempt to generate code for MIPS-I and MIPS-V. They have not
  // been tested and currently exist for the integrated assembler only.
  if (RISCV64ArchVersion == RISCV641)
    report_fatal_error("Code generation for MIPS-I is not implemented", false);
  if (RISCV64ArchVersion == RISCV645)
    report_fatal_error("Code generation for MIPS-V is not implemented", false);

  // Check if Architecture and ABI are compatible.
  assert(((!isGP64bit() && isABI_O32()) ||
          (isGP64bit() && (isABI_N32() || isABI_N64()))) &&
         "Invalid  Arch & ABI pair.");

  if (hasMSA() && !isFP64bit())
    report_fatal_error("MSA requires a 64-bit FPU register file (FR=1 mode). "
                       "See -mattr=+fp64.",
                       false);

  if (!isABI_O32() && !useOddSPReg())
    report_fatal_error("-mattr=+nooddspreg requires the O32 ABI.", false);

  if (IsFPXX && (isABI_N32() || isABI_N64()))
    report_fatal_error("FPXX is not permitted for the N32/N64 ABI's.", false);

  if (hasRISCV6464r6() && InMicroRISCV64Mode)
    report_fatal_error("microMIPS64R6 is not supported", false);

  if (!isABI_O32() && InMicroRISCV64Mode)
    report_fatal_error("microMIPS64 is not supported.", false);

  if (UseIndirectJumpsHazard) {
    if (InMicroRISCV64Mode)
      report_fatal_error(
          "cannot combine indirect jumps with hazard barriers and microMIPS");
    if (!hasRISCV6432r2())
      report_fatal_error(
          "indirect jumps with hazard barriers requires MIPS32R2 or later");
  }
  if (hasRISCV6432r6()) {
    StringRef ISA = hasRISCV6464r6() ? "MIPS64r6" : "MIPS32r6";

    assert(isFP64bit());
    assert(isNaN2008());
    if (hasDSP())
      report_fatal_error(ISA + " is not compatible with the DSP ASE", false);
  }

  if (NoABICalls && TM.isPositionIndependent())
    report_fatal_error("position-independent code requires '-mabicalls'");

  if (isABI_N64() && !TM.isPositionIndependent() && !hasSym32())
    NoABICalls = true;

  // Set UseSmallSection.
  UseSmallSection = GPOpt;
  if (!NoABICalls && GPOpt) {
    errs() << "warning: cannot use small-data accesses for '-mabicalls'"
           << "\n";
    UseSmallSection = false;
  }

  if (hasDSPR2() && !DspWarningPrinted) {
    if (hasRISCV6464() && !hasRISCV6464r2()) {
      errs() << "warning: the 'dspr2' ASE requires MIPS64 revision 2 or "
             << "greater\n";
      DspWarningPrinted = true;
    } else if (hasRISCV6432() && !hasRISCV6432r2()) {
      errs() << "warning: the 'dspr2' ASE requires MIPS32 revision 2 or "
             << "greater\n";
      DspWarningPrinted = true;
    }
  } else if (hasDSP() && !DspWarningPrinted) {
    if (hasRISCV6464() && !hasRISCV6464r2()) {
      errs() << "warning: the 'dsp' ASE requires MIPS64 revision 2 or "
             << "greater\n";
      DspWarningPrinted = true;
    } else if (hasRISCV6432() && !hasRISCV6432r2()) {
      errs() << "warning: the 'dsp' ASE requires MIPS32 revision 2 or "
             << "greater\n";
      DspWarningPrinted = true;
    }
  }

  StringRef ArchName = hasRISCV6464() ? "MIPS64" : "MIPS32";

  if (!hasRISCV6432r5() && hasMSA() && !MSAWarningPrinted) {
    errs() << "warning: the 'msa' ASE requires " << ArchName
           << " revision 5 or greater\n";
    MSAWarningPrinted = true;
  }
  if (!hasRISCV6432r5() && hasVirt() && !VirtWarningPrinted) {
    errs() << "warning: the 'virt' ASE requires " << ArchName
           << " revision 5 or greater\n";
    VirtWarningPrinted = true;
  }
  if (!hasRISCV6432r6() && hasCRC() && !CRCWarningPrinted) {
    errs() << "warning: the 'crc' ASE requires " << ArchName
           << " revision 6 or greater\n";
    CRCWarningPrinted = true;
  }
  if (!hasRISCV6432r6() && hasGINV() && !GINVWarningPrinted) {
    errs() << "warning: the 'ginv' ASE requires " << ArchName
           << " revision 6 or greater\n";
    GINVWarningPrinted = true;
  }

  CallLoweringInfo.reset(new RISCV64CallLowering(*getTargetLowering()));
  Legalizer.reset(new RISCV64LegalizerInfo(*this));

  auto *RBI = new RISCV64RegisterBankInfo(*getRegisterInfo());
  RegBankInfo.reset(RBI);
  InstSelector.reset(createRISCV64InstructionSelector(
      *static_cast<const RISCV64TargetMachine *>(&TM), *this, *RBI));
}

bool RISCV64Subtarget::isPositionIndependent() const {
  return TM.isPositionIndependent();
}

/// This overrides the PostRAScheduler bit in the SchedModel for any CPU.
bool RISCV64Subtarget::enablePostRAScheduler() const { return true; }

void RISCV64Subtarget::getCriticalPathRCs(RegClassVector &CriticalPathRCs) const {
  CriticalPathRCs.clear();
  CriticalPathRCs.push_back(isGP64bit() ? &RISCV64::GPR64RegClass
                                        : &RISCV64::GPR32RegClass);
}

CodeGenOpt::Level RISCV64Subtarget::getOptLevelToEnablePostRAScheduler() const {
  return CodeGenOpt::Aggressive;
}

RISCV64Subtarget &
RISCV64Subtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                               const TargetMachine &TM) {
  std::string CPUName = MIPS_MC::selectRISCV64CPU(TM.getTargetTriple(), CPU);

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);
  // Initialize scheduling itinerary for the specified CPU.
  InstrItins = getInstrItineraryForCPU(CPUName);

  if (InRISCV6416Mode && !IsSoftFloat)
    InRISCV6416HardFloat = true;

  if (StackAlignOverride)
    stackAlignment = StackAlignOverride;
  else if (isABI_N32() || isABI_N64())
    stackAlignment = 16;
  else {
    assert(isABI_O32() && "Unknown ABI for stack alignment!");
    stackAlignment = 8;
  }

  return *this;
}

bool RISCV64Subtarget::useConstantIslands() {
  LLVM_DEBUG(dbgs() << "use constant islands " << RISCV6416ConstantIslands
                    << "\n");
  return RISCV6416ConstantIslands;
}

Reloc::Model RISCV64Subtarget::getRelocationModel() const {
  return TM.getRelocationModel();
}

bool RISCV64Subtarget::isABI_N64() const { return getABI().IsN64(); }
bool RISCV64Subtarget::isABI_N32() const { return getABI().IsN32(); }
bool RISCV64Subtarget::isABI_O32() const { return getABI().IsO32(); }
const RISCV64ABIInfo &RISCV64Subtarget::getABI() const { return TM.getABI(); }

const CallLowering *RISCV64Subtarget::getCallLowering() const {
  return CallLoweringInfo.get();
}

const LegalizerInfo *RISCV64Subtarget::getLegalizerInfo() const {
  return Legalizer.get();
}

const RegisterBankInfo *RISCV64Subtarget::getRegBankInfo() const {
  return RegBankInfo.get();
}

const InstructionSelector *RISCV64Subtarget::getInstructionSelector() const {
  return InstSelector.get();
}
