//===-- MYRISCVXRegisterInfo.cpp - MYRISCVX Register Information -== ------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVX.h"
#include "MYRISCVXRegisterInfo.h"
#include "MYRISCVXSubtarget.h"
#include "MYRISCVXMachineFunction.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "MYRISCVX-reg-info"

#define GET_REGINFO_TARGET_DESC
#include "MYRISCVXGenRegisterInfo.inc"


MYRISCVXRegisterInfo::MYRISCVXRegisterInfo(const MYRISCVXSubtarget &ST)
    : MYRISCVXGenRegisterInfo(MYRISCVX::RA), Subtarget(ST) {}

//===----------------------------------------------------------------------===//
// Callee Saved Registers methods
//===----------------------------------------------------------------------===//
/// MYRISCVX Callee Saved Registers
// In MYRISCVXCallConv.td,
// def CSR_LP32...
// llc create CSR_O32_SaveList and CSR_O32_RegMask from above defined.
const MCPhysReg *
MYRISCVXRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  return CSR_LP32_SaveList;
}

const uint32_t *
MYRISCVXRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                           CallingConv::ID) const {
  return CSR_LP32_RegMask;
}

// pure virtual method
//@getReservedRegs {
BitVector MYRISCVXRegisterInfo::
getReservedRegs(const MachineFunction &MF) const {
  //@getReservedRegs body {
  static const uint16_t ReservedCPURegs[] = {
    MYRISCVX::ZERO, MYRISCVX::RA, MYRISCVX::FP, MYRISCVX::SP, MYRISCVX::GP, MYRISCVX::TP
  };
  BitVector Reserved(getNumRegs());

  for (unsigned I = 0; I < array_lengthof(ReservedCPURegs); ++I)
    Reserved.set(ReservedCPURegs[I]);

  return Reserved;
}

//@eliminateFrameIndex {
//- If no eliminateFrameIndex(), it will hang on run.
// pure virtual method
// FrameIndex represent objects inside a abstract stack.
// We must replace FrameIndex with an stack/frame pointer
// direct reference.
void MYRISCVXRegisterInfo::
eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj,
                    unsigned FIOperandNum, RegScavenger *RS) const {
}
//}

bool
MYRISCVXRegisterInfo::requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool
MYRISCVXRegisterInfo::trackLivenessAfterRegAlloc(const MachineFunction &MF) const {
  return true;
}

// pure virtual method
unsigned MYRISCVXRegisterInfo::
getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
  return TFI->hasFP(MF) ? (MYRISCVX::FP) :
      (MYRISCVX::SP);
}
