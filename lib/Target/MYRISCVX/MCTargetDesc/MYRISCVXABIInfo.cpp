//===---- MYRISCVXABIInfo.cpp - Information about MYRISCVX ABI's ------------------===//
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

namespace {
static const MCPhysReg LP64IntRegs[8] = { MYRISCVX::A0, MYRISCVX::A1,
                                          MYRISCVX::A2, MYRISCVX::A3,
                                          MYRISCVX::A4, MYRISCVX::A5,
                                          MYRISCVX::A6, MYRISCVX::A7 };
}

const ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetByValArgRegs() const {
  if (IsLP64())
    return makeArrayRef(LP64IntRegs);
  llvm_unreachable("Unhandled ABI");
}

const ArrayRef<MCPhysReg> MYRISCVXABIInfo::GetVarArgRegs() const {
  if (IsLP64())
    return makeArrayRef(LP64IntRegs);
  llvm_unreachable("Unhandled ABI");
}

unsigned MYRISCVXABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  if (IsLP64())
    return CC != 0;
  llvm_unreachable("Unhandled ABI");
}

MYRISCVXABIInfo MYRISCVXABIInfo::computeTargetABI() {
  MYRISCVXABIInfo abi(ABI::Unknown);

  abi = ABI::LP64;

  // Assert exactly one ABI was chosen.
  assert(abi.ThisABI != ABI::Unknown);

  return abi;
}

unsigned MYRISCVXABIInfo::GetStackPtr() const {
  return MYRISCVX::SP;
}

unsigned MYRISCVXABIInfo::GetFramePtr() const {
  return MYRISCVX::S0; // S0 is FP Register
}

unsigned MYRISCVXABIInfo::GetNullPtr() const {
  return MYRISCVX::ZERO;
}

// unsigned MYRISCVXABIInfo::GetEhDataReg(unsigned I) const {
//   static const unsigned EhDataReg[] = {
//     MYRISCVX::A0, MYRISCVX::A1
//   };
//
//   return EhDataReg[I];
// }
//
// int MYRISCVXABIInfo::EhDataRegSize() const {
//   if (ThisABI == ABI::LP64)
//     return 0;
//   else
//     return 2;
// }
