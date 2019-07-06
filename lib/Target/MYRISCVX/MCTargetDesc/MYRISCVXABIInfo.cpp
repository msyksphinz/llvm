//===---- MYRISCVXABIInfo.cpp - Information about MYRISCVX ABI's ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXABIInfo.h"
#include "MYRISCVXRegisterInfo.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/Support/CommandLine.h"

#include <stdio.h>

using namespace llvm;

static cl::opt<bool>
EnableMYRISCVXLP32Calls("MYRISCVX-lp32-calls", cl::Hidden,
                        cl::desc("MYRISCVX LP32 call: Default Interger Calling Convention\
                    "), cl::init(false));

namespace {
static const MCPhysReg LP32IntRegs[8] = {
  MYRISCVX::A0, MYRISCVX::A1, MYRISCVX::A2, MYRISCVX::A3,
  MYRISCVX::A4, MYRISCVX::A5, MYRISCVX::A6, MYRISCVX::A7};
static const MCPhysReg STACK32IntRegs[1] = {MYRISCVX::A0};
}

ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetByValArgRegs() const {
  if (IsLP32()) {
    return makeArrayRef(LP32IntRegs);
  } else if (IsSTACK32()) {
    return makeArrayRef(STACK32IntRegs);
  }
  llvm_unreachable("Unhandled ABI");
}

ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetVarArgRegs() const {
  if (IsLP32()) {
    return makeArrayRef(LP32IntRegs);
  } else if (IsSTACK32()) {
    return makeArrayRef(STACK32IntRegs);
  }
  llvm_unreachable("Unhandled ABI");
}

unsigned MYRISCVXABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  if (IsLP32()) {
    return CC != 0;
  } else if (IsSTACK32()) {
    return 0;
  }
  llvm_unreachable("Unhandled ABI");
}

MYRISCVXABIInfo MYRISCVXABIInfo::computeTargetABI(const MCTargetOptions &Options) {
  MYRISCVXABIInfo abi(ABI::Unknown);

  fprintf(stderr, "computeTargetABI = %s\n", Options.getABIName().str().c_str());

  if (Options.getABIName().startswith("lp32")) {
    return MYRISCVXABIInfo::LP32();
  } else if (Options.getABIName().startswith("stack32")) {
    return MYRISCVXABIInfo::STACK32();
  } else {
    return MYRISCVXABIInfo::LP32();
  }
}

unsigned MYRISCVXABIInfo::GetStackPtr() const {
  return MYRISCVX::SP;
}

unsigned MYRISCVXABIInfo::GetFramePtr() const {
  return MYRISCVX::FP;
}

unsigned MYRISCVXABIInfo::GetNullPtr() const {
  return MYRISCVX::ZERO;
}

unsigned MYRISCVXABIInfo::GetEhDataReg(unsigned I) const {
  static const unsigned EhDataReg[] = {
    MYRISCVX::A0, MYRISCVX::A1, MYRISCVX::A2, MYRISCVX::A3,
    MYRISCVX::A4, MYRISCVX::A5, MYRISCVX::A6, MYRISCVX::A7
  };

  return EhDataReg[I];
}

int MYRISCVXABIInfo::EhDataRegSize() const {
  return sizeof(LP32IntRegs);
}
