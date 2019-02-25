//===-- MYRISCVXSubtarget.cpp - MYRISCVX Subtarget Information --------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MYRISCVX specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXSubtarget.h"
#include "MYRISCVXMachineFunction.h"
#include "MYRISCVX.h"
#include "MYRISCVXRegisterInfo.h"
#include "MYRISCVXTargetMachine.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;
#define DEBUG_TYPE "MYRISCVX-subtarget"
#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "MYRISCVXGenSubtargetInfo.inc"

extern bool FixGlobalBaseReg;

static cl::opt<bool> UseSmallSectionOpt
("myriscvx-use-small-section", cl::Hidden, cl::init(false),
 cl::desc("Use small section. Only work when -relocation-model="
          "static. pic always not use small section."));
static cl::opt<bool> ReserveGPOpt
("myriscvx-reserve-gp", cl::Hidden, cl::init(false),
 cl::desc("Never allocate $gp to variable"));
static cl::opt<bool> NoCploadOpt
("myriscvx-no-cpload", cl::Hidden, cl::init(false),
 cl::desc("No issue .cpload"));
bool MYRISCVXReserveGP;
bool MYRISCVXNoCpload;


void MYRISCVXSubtarget::anchor() { }
//@1 {
MYRISCVXSubtarget::MYRISCVXSubtarget(const Triple &TT, const std::string &CPU,
                                     const std::string &FS,
                                     const MYRISCVXTargetMachine &_TM) :
    //@1 }
    // MYRISCVXGenSubtargetInfo will display features by llc -march=MYRISCVX -mcpu=help
    MYRISCVXGenSubtargetInfo(TT, CPU, FS),
    TM(_TM), TargetTriple(TT), TSInfo(),
    InstrInfo(
        MYRISCVXInstrInfo::create(initializeSubtargetDependencies(CPU, FS, TM))),
    FrameLowering(MYRISCVXFrameLowering::create(*this)),
    TLInfo(MYRISCVXTargetLowering::create(TM, *this)) {
  // Set UseSmallSection.
  UseSmallSection = UseSmallSectionOpt;
  MYRISCVXReserveGP = ReserveGPOpt;
  MYRISCVXNoCpload = NoCploadOpt;
}


bool MYRISCVXSubtarget::isPositionIndependent() const {
  return TM.isPositionIndependent();
}

MYRISCVXSubtarget &
MYRISCVXSubtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                                   const TargetMachine &TM) {
  if (TargetTriple.getArch() == Triple::myriscvx32 || TargetTriple.getArch() == Triple::myriscvx64) {
    if (CPU.empty() || CPU == "generic") {
      CPU = "MYRISCVX";
    }
    else if (CPU == "help") {
      CPU = "";
      return *this;
    }
    else if (CPU != "MYRISCVX") {
      CPU = "MYRISCVX";
    }
  } else {
    errs() << "!!!Error, TargetTriple.getArch() = " << TargetTriple.getArch()
           << "CPU = " << CPU << "\n";
    exit(0);
  }

  HasArch64bit = false;

  // Parse features string.
  ParseSubtargetFeatures(CPU, FS);
  // Initialize scheduling itinerary for the specified CPU.
  InstrItins = getInstrItineraryForCPU(CPU);

  return *this;
}


bool MYRISCVXSubtarget::abiUsesSoftFloat() const {
  // return TM->Options.UseSoftFloat;
  return true;
}

const MYRISCVXABIInfo &MYRISCVXSubtarget::getABI() const { return TM.getABI(); }
