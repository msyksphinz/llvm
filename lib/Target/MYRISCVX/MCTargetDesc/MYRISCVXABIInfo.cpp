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

using namespace llvm;

static cl::opt<bool>
EnableMYRISCVXLPCalls("MYRISCVX-lp32-calls", cl::Hidden,
                      cl::desc("MYRISCVX LP call: Default Interger Calling Convention\
                    "), cl::init(false));

namespace {
static const MCPhysReg LPIntRegs[8] = {
  MYRISCVX::A0, MYRISCVX::A1, MYRISCVX::A2, MYRISCVX::A3,
  MYRISCVX::A4, MYRISCVX::A5, MYRISCVX::A6, MYRISCVX::A7};
}

ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetByValArgRegs() const {
  if (IsLP())
    return makeArrayRef(LPIntRegs);
  llvm_unreachable("Unhandled ABI");
}

ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetVarArgRegs() const {
  if (IsLP())
    return makeArrayRef(LPIntRegs);
  llvm_unreachable("Unhandled ABI");
}

unsigned MYRISCVXABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  if (IsLP())
    return CC != 0;
  llvm_unreachable("Unhandled ABI");
}

MYRISCVXABIInfo MYRISCVXABIInfo::computeTargetABI(StringRef ABIName) {
  MYRISCVXABIInfo abi(ABI::Unknown);

  if (ABIName.empty()) {
    return ABI::LP;
  }
  if (ABIName == "lp") {
    return ABI::LP;
  }
  if (ABIName == "stack") {
    return ABI::STACK;
  }

  // Assert exactly one ABI was chosen.
  errs() << "Unknown ABI : " << ABIName << '\n';

  return abi;
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
  return sizeof(LPIntRegs);
}
