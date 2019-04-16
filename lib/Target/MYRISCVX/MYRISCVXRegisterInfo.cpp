//===-- MYRISCVXRegisterInfo.cpp - MYRISCVX Register Information -== --------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "myriscvx-reg-info"
#include "MYRISCVXRegisterInfo.h"
#include "MYRISCVX.h"
#include "MYRISCVXSubtarget.h"
#include "MYRISCVXMachineFunction.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#define GET_REGINFO_TARGET_DESC
#include "MYRISCVXGenRegisterInfo.inc"

using namespace llvm;
MYRISCVXRegisterInfo::MYRISCVXRegisterInfo(const MYRISCVXSubtarget &ST)
    : MYRISCVXGenRegisterInfo(MYRISCVX::RA), Subtarget(ST) {}

//===----------------------------------------------------------------------===//
// Callee Saved Registers methods
//===----------------------------------------------------------------------===//
/// MYRISCVX Callee Saved Registers
// In MYRISCVXCallConv.td,
// def CSR_O32 : CalleeSavedRegs<(add LR, FP,
// (sequence "S%u", 2, 0))>;
// llc create CSR_O32_SaveList and CSR_O32_RegMask from above defined.

const MCPhysReg *
MYRISCVXRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  return CSR_SaveList;
}

const uint32_t *
MYRISCVXRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                           CallingConv::ID) const {
  return CSR_RegMask;
}

// pure virtual method
//@getReservedRegs {
BitVector MYRISCVXRegisterInfo::
getReservedRegs(const MachineFunction &MF) const {
  //@getReservedRegs body {
  static const uint16_t ReservedCPURegs[] = {
    MYRISCVX::ZERO, MYRISCVX::SP, MYRISCVX::RA
  };
  BitVector Reserved(getNumRegs());
  for (unsigned I = 0; I < array_lengthof(ReservedCPURegs); ++I)
    Reserved.set(ReservedCPURegs[I]);

#ifdef ENABLE_GPRESTORE //1
  const MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();
  // Reserve GP if globalBaseRegFixed()
  if (MYRISCVXFI->globalBaseRegFixed())
#endif
  Reserved.set(MYRISCVX::GP);

  // Reserve FP if this function should have a dedicated frame pointer register.
  if (MF.getSubtarget().getFrameLowering()->hasFP(MF)) {
    Reserved.set(MYRISCVX::S0);
  }

  return Reserved;
}


//- If no eliminateFrameIndex(), it will hang on run.
// pure virtual method
// FrameIndex represent objects inside a abstract stack.
// We must replace FrameIndex with an stack/frame pointer
// direct reference.
void MYRISCVXRegisterInfo::
eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj,
                    unsigned FIOperandNum, RegScavenger *RS) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();

  unsigned i = 0;
  while (!MI.getOperand(i).isFI()) {
    ++i;
    assert(i < MI.getNumOperands() &&
           "Instr doesn't have FrameIndex operand!");
  }
  LLVM_DEBUG(dbgs() << "\nFunction : " << MF.getFunction().getName() << "\n");
  LLVM_DEBUG(dbgs() << "<--------->\n" << MI);
  int FrameIndex = MI.getOperand(i).getIndex();
  uint64_t stackSize = MF.getFrameInfo().getStackSize();
  int64_t spOffset = MF.getFrameInfo().getObjectOffset(FrameIndex);
  LLVM_DEBUG(dbgs() << "FrameIndex : " << FrameIndex << "\n"
             << "spOffset : " << spOffset << "\n"
             << "stackSize : " << stackSize << "\n");

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  int MinCSFI = 0;
  int MaxCSFI = -1;
  if (CSI.size()) {
    MinCSFI = CSI[0].getFrameIdx();
    MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  }

  // The following stack frame objects are always referenced relative to $sp:
  // 1. Outgoing arguments.
  // 2. Pointer to dynamically allocated stack space.
  // 3. Locations for callee-saved registers.
  // Everything else is referenced relative to whatever register
  // getFrameRegister() returns.
  unsigned FrameReg;
  FrameReg = MYRISCVX::SP;
  // Calculate final offset.
  // - There is no need to change the offset if the frame object is one of the
  // following: an outgoing argument, pointer to a dynamically allocated
  // stack space or a $gp restore location,
  // - If the frame object is any of the following, its offset must be adjusted
  // by adding the size of the stack:

  // incoming argument, callee-saved register location or local variable.
  int64_t Offset;

#ifdef ENABLE_GPRESTORE //2
  if (MYRISCVXFI->isOutArgFI(FrameIndex) || MYRISCVXFI->isGPFI(FrameIndex) ||
      MYRISCVXFI->isDynAllocFI(FrameIndex))
    Offset = spOffset;
  else
#endif
    Offset = spOffset + (int64_t)stackSize;

  Offset += MI.getOperand(i+1).getImm();
  LLVM_DEBUG(dbgs() << "Offset : " << Offset << "\n" << "<--------->\n");
  // If MI is not a debug value, make sure Offset fits in the 16-bit immediate
  // field.
  if (!MI.isDebugValue() && !isInt<16>(Offset)) {
    assert(0 && "(!MI.isDebugValue() && !isInt<16>(Offset))");
  }
  MI.getOperand(i).ChangeToRegister(FrameReg, false);
  MI.getOperand(i+1).ChangeToImmediate(Offset);
}


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
  return TFI->hasFP(MF) ? (MYRISCVX::S0) : (MYRISCVX::SP);
}


const TargetRegisterClass *
MYRISCVXRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                     unsigned Kind) const {
  return &MYRISCVX::GPRRegClass;
}
