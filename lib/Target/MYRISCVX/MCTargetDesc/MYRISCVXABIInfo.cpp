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
EnableMYRISCVXLP32Calls("MYRISCVX-lp32-calls", cl::Hidden,
                        cl::desc("MYRISCVX LP32 call: Default Interger Calling Convention\
                    "), cl::init(false));

namespace {
static const MCPhysReg LP32IntRegs[8] = {
  MYRISCVX::A0, MYRISCVX::A1, MYRISCVX::A2, MYRISCVX::A3,
  MYRISCVX::A4, MYRISCVX::A5, MYRISCVX::A6, MYRISCVX::A7};
}

ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetByValArgRegs() const {
  if (IsLP32())
    return makeArrayRef(LP32IntRegs);
  llvm_unreachable("Unhandled ABI");
}

ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetVarArgRegs() const {
  if (IsLP32())
    return makeArrayRef(LP32IntRegs);
  llvm_unreachable("Unhandled ABI");
}

unsigned MYRISCVXABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  if (IsLP32())
    return CC != 0;
  llvm_unreachable("Unhandled ABI");
}

MYRISCVXABIInfo MYRISCVXABIInfo::computeTargetABI() {
  MYRISCVXABIInfo abi(ABI::Unknown);

  abi = ABI::LP32;
  // Assert exactly one ABI was chosen.
  assert(abi.ThisABI != ABI::Unknown);

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
  return sizeof(LP32IntRegs);
}
