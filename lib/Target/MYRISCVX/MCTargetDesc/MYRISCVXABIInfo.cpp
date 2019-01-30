//===---- MYRISCVXABIInfo.cpp - Information about MYRISCVX ABI's ------------------===//
//
// The LLVM Compiler Infrastructure
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
EnableMYRISCVXS32Calls("MYRISCVX-s32-calls", cl::Hidden,
                       cl::desc("MYRISCVX S32 call: use stack only to pass arguments.\
"), cl::init(false));

namespace {
static const MCPhysReg O32IntRegs[4] = {MYRISCVX::A0, MYRISCVX::A1};
static const MCPhysReg S32IntRegs = {};
}

const ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetByValArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsS32())
    return makeArrayRef(S32IntRegs);
  llvm_unreachable("Unhandled ABI");
}

const ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetVarArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsS32())
    return makeArrayRef(S32IntRegs);
  llvm_unreachable("Unhandled ABI");
}

unsigned MYRISCVXABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  if (IsO32())
    return CC != 0;
  if (IsS32())
    return 0;
  llvm_unreachable("Unhandled ABI");
}

MYRISCVXABIInfo MYRISCVXABIInfo::computeTargetABI() {
  MYRISCVXABIInfo abi(ABI::Unknown);
  if (EnableMYRISCVXS32Calls)
    abi = ABI::S32;
  else
    abi = ABI::O32;
  // Assert exactly one ABI was chosen.
  assert(abi.ThisABI != ABI::Unknown);
  return abi;
}

unsigned MYRISCVXABIInfo::GetStackPtr() const {
  return MYRISCVX::SP;
}

unsigned MYRISCVXABIInfo::GetFramePtr() const {
  return MYRISCVX::S0;
}

unsigned MYRISCVXABIInfo::GetNullPtr() const {
  return MYRISCVX::ZERO;
}

unsigned MYRISCVXABIInfo::GetEhDataReg(unsigned I) const {
  static const unsigned EhDataReg[] = {
    MYRISCVX::A0, MYRISCVX::A1
  };
  return EhDataReg[I];
}

int MYRISCVXABIInfo::EhDataRegSize() const {
  if (ThisABI == ABI::S32)
    return 0;
  else
    return 2;
}
