//===---- RISCV64ABIInfo.cpp - Information about MIPS ABI's ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "RISCV64ABIInfo.h"
#include "RISCV64RegisterInfo.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCTargetOptions.h"

using namespace llvm;

namespace {
static const MCPhysReg O32IntRegs[4] = {RISCV64::A0, RISCV64::A1, RISCV64::A2, RISCV64::A3};

static const MCPhysReg RISCV6464IntRegs[8] = {
    RISCV64::A0_64, RISCV64::A1_64, RISCV64::A2_64, RISCV64::A3_64,
    RISCV64::T0_64, RISCV64::T1_64, RISCV64::T2_64, RISCV64::T3_64};
}

ArrayRef<MCPhysReg> RISCV64ABIInfo::GetByValArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsN32() || IsN64())
    return makeArrayRef(RISCV6464IntRegs);
  llvm_unreachable("Unhandled ABI");
}

ArrayRef<MCPhysReg> RISCV64ABIInfo::GetVarArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsN32() || IsN64())
    return makeArrayRef(RISCV6464IntRegs);
  llvm_unreachable("Unhandled ABI");
}

unsigned RISCV64ABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  if (IsO32())
    return CC != CallingConv::Fast ? 16 : 0;
  if (IsN32() || IsN64())
    return 0;
  llvm_unreachable("Unhandled ABI");
}

RISCV64ABIInfo RISCV64ABIInfo::computeTargetABI(const Triple &TT, StringRef CPU,
                                          const MCTargetOptions &Options) {
  if (Options.getABIName().startswith("o32"))
    return RISCV64ABIInfo::O32();
  if (Options.getABIName().startswith("n32"))
    return RISCV64ABIInfo::N32();
  if (Options.getABIName().startswith("n64"))
    return RISCV64ABIInfo::N64();
  if (TT.getEnvironment() == llvm::Triple::GNUABIN32)
    return RISCV64ABIInfo::N32();
  assert(Options.getABIName().empty() && "Unknown ABI option for MIPS");

  if (TT.isMIPS64())
    return RISCV64ABIInfo::N64();
  return RISCV64ABIInfo::O32();
}

unsigned RISCV64ABIInfo::GetStackPtr() const {
  return ArePtrs64bit() ? RISCV64::SP_64 : RISCV64::SP;
}

unsigned RISCV64ABIInfo::GetFramePtr() const {
  return ArePtrs64bit() ? RISCV64::FP_64 : RISCV64::FP;
}

unsigned RISCV64ABIInfo::GetBasePtr() const {
  return ArePtrs64bit() ? RISCV64::S7_64 : RISCV64::S7;
}

unsigned RISCV64ABIInfo::GetGlobalPtr() const {
  return ArePtrs64bit() ? RISCV64::GP_64 : RISCV64::GP;
}

unsigned RISCV64ABIInfo::GetNullPtr() const {
  return ArePtrs64bit() ? RISCV64::ZERO_64 : RISCV64::ZERO;
}

unsigned RISCV64ABIInfo::GetZeroReg() const {
  return AreGprs64bit() ? RISCV64::ZERO_64 : RISCV64::ZERO;
}

unsigned RISCV64ABIInfo::GetPtrAdduOp() const {
  return ArePtrs64bit() ? RISCV64::DADDu : RISCV64::ADDu;
}

unsigned RISCV64ABIInfo::GetPtrAddiuOp() const {
  return ArePtrs64bit() ? RISCV64::DADDiu : RISCV64::ADDiu;
}

unsigned RISCV64ABIInfo::GetPtrSubuOp() const {
  return ArePtrs64bit() ? RISCV64::DSUBu : RISCV64::SUBu;
}

unsigned RISCV64ABIInfo::GetPtrAndOp() const {
  return ArePtrs64bit() ? RISCV64::AND64 : RISCV64::AND;
}

unsigned RISCV64ABIInfo::GetGPRMoveOp() const {
  return ArePtrs64bit() ? RISCV64::OR64 : RISCV64::OR;
}

unsigned RISCV64ABIInfo::GetEhDataReg(unsigned I) const {
  static const unsigned EhDataReg[] = {
    RISCV64::A0, RISCV64::A1, RISCV64::A2, RISCV64::A3
  };
  static const unsigned EhDataReg64[] = {
    RISCV64::A0_64, RISCV64::A1_64, RISCV64::A2_64, RISCV64::A3_64
  };

  return IsN64() ? EhDataReg64[I] : EhDataReg[I];
}

