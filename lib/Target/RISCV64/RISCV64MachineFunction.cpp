//===-- RISCV64MachineFunctionInfo.cpp - Private data used for RISCV64 ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "RISCV64MachineFunction.h"
#include "MCTargetDesc/RISCV64ABIInfo.h"
#include "RISCV64Subtarget.h"
#include "RISCV64TargetMachine.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

static cl::opt<bool>
FixGlobalBaseReg("mips-fix-global-base-reg", cl::Hidden, cl::init(true),
                 cl::desc("Always use $gp as the global base register."));

RISCV64FunctionInfo::~RISCV64FunctionInfo() = default;

bool RISCV64FunctionInfo::globalBaseRegSet() const {
  return GlobalBaseReg;
}

static const TargetRegisterClass &getGlobalBaseRegClass(MachineFunction &MF) {
  auto &STI = static_cast<const RISCV64Subtarget &>(MF.getSubtarget());
  auto &TM = static_cast<const RISCV64TargetMachine &>(MF.getTarget());

  if (STI.inRISCV6416Mode())
    return RISCV64::CPU16RegsRegClass;

  if (STI.inMicroRISCV64Mode())
    return RISCV64::GPRMM16RegClass;

  if (TM.getABI().IsN64())
    return RISCV64::GPR64RegClass;

  return RISCV64::GPR32RegClass;
}

unsigned RISCV64FunctionInfo::getGlobalBaseReg() {
  if (!GlobalBaseReg)
    GlobalBaseReg =
        MF.getRegInfo().createVirtualRegister(&getGlobalBaseRegClass(MF));
  return GlobalBaseReg;
}

void RISCV64FunctionInfo::createEhDataRegsFI() {
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  for (int I = 0; I < 4; ++I) {
    const TargetRegisterClass &RC =
        static_cast<const RISCV64TargetMachine &>(MF.getTarget()).getABI().IsN64()
            ? RISCV64::GPR64RegClass
            : RISCV64::GPR32RegClass;

    EhDataRegFI[I] = MF.getFrameInfo().CreateStackObject(TRI.getSpillSize(RC),
        TRI.getSpillAlignment(RC), false);
  }
}

void RISCV64FunctionInfo::createISRRegFI() {
  // ISRs require spill slots for Status & ErrorPC Coprocessor 0 registers.
  // The current implementation only supports RISCV6432r2+ not RISCV6464rX. Status
  // is always 32 bits, ErrorPC is 32 or 64 bits dependent on architecture,
  // however RISCV6432r2+ is the supported architecture.
  const TargetRegisterClass &RC = RISCV64::GPR32RegClass;
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  for (int I = 0; I < 2; ++I)
    ISRDataRegFI[I] = MF.getFrameInfo().CreateStackObject(
        TRI.getSpillSize(RC), TRI.getSpillAlignment(RC), false);
}

bool RISCV64FunctionInfo::isEhDataRegFI(int FI) const {
  return CallsEhReturn && (FI == EhDataRegFI[0] || FI == EhDataRegFI[1]
                        || FI == EhDataRegFI[2] || FI == EhDataRegFI[3]);
}

bool RISCV64FunctionInfo::isISRRegFI(int FI) const {
  return IsISR && (FI == ISRDataRegFI[0] || FI == ISRDataRegFI[1]);
}
MachinePointerInfo RISCV64FunctionInfo::callPtrInfo(const char *ES) {
  return MachinePointerInfo(MF.getPSVManager().getExternalSymbolCallEntry(ES));
}

MachinePointerInfo RISCV64FunctionInfo::callPtrInfo(const GlobalValue *GV) {
  return MachinePointerInfo(MF.getPSVManager().getGlobalValueCallEntry(GV));
}

int RISCV64FunctionInfo::getMoveF64ViaSpillFI(const TargetRegisterClass *RC) {
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  if (MoveF64ViaSpillFI == -1) {
    MoveF64ViaSpillFI = MF.getFrameInfo().CreateStackObject(
        TRI.getSpillSize(*RC), TRI.getSpillAlignment(*RC), false);
  }
  return MoveF64ViaSpillFI;
}

void RISCV64FunctionInfo::anchor() {}
