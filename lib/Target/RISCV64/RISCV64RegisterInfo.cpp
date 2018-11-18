//===- RISCV64RegisterInfo.cpp - MIPS Register Information -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MIPS implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "RISCV64RegisterInfo.h"
#include "MCTargetDesc/RISCV64ABIInfo.h"
#include "RISCV64.h"
#include "RISCV64MachineFunction.h"
#include "RISCV64Subtarget.h"
#include "RISCV64TargetMachine.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <cstdint>

using namespace llvm;

#define DEBUG_TYPE "mips-reg-info"

#define GET_REGINFO_TARGET_DESC
#include "RISCV64GenRegisterInfo.inc"

RISCV64RegisterInfo::RISCV64RegisterInfo() : RISCV64GenRegisterInfo(RISCV64::RA) {}

unsigned RISCV64RegisterInfo::getPICCallReg() { return RISCV64::T9; }

const TargetRegisterClass *
RISCV64RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                     unsigned Kind) const {
  RISCV64ABIInfo ABI = MF.getSubtarget<RISCV64Subtarget>().getABI();
  RISCV64PtrClass PtrClassKind = static_cast<RISCV64PtrClass>(Kind);

  switch (PtrClassKind) {
  case RISCV64PtrClass::Default:
    return ABI.ArePtrs64bit() ? &RISCV64::GPR64RegClass : &RISCV64::GPR32RegClass;
  case RISCV64PtrClass::GPR16MM:
    return &RISCV64::GPRMM16RegClass;
  case RISCV64PtrClass::StackPointer:
    return ABI.ArePtrs64bit() ? &RISCV64::SP64RegClass : &RISCV64::SP32RegClass;
  case RISCV64PtrClass::GlobalPointer:
    return ABI.ArePtrs64bit() ? &RISCV64::GP64RegClass : &RISCV64::GP32RegClass;
  }

  llvm_unreachable("Unknown pointer kind");
}

unsigned
RISCV64RegisterInfo::getRegPressureLimit(const TargetRegisterClass *RC,
                                      MachineFunction &MF) const {
  switch (RC->getID()) {
  default:
    return 0;
  case RISCV64::GPR32RegClassID:
  case RISCV64::GPR64RegClassID:
  case RISCV64::DSPRRegClassID: {
    const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
    return 28 - TFI->hasFP(MF);
  }
  case RISCV64::FGR32RegClassID:
    return 32;
  case RISCV64::AFGR64RegClassID:
    return 16;
  case RISCV64::FGR64RegClassID:
    return 32;
  }
}

//===----------------------------------------------------------------------===//
// Callee Saved Registers methods
//===----------------------------------------------------------------------===//

/// RISCV64 Callee Saved Registers
const MCPhysReg *
RISCV64RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  const RISCV64Subtarget &Subtarget = MF->getSubtarget<RISCV64Subtarget>();
  const Function &F = MF->getFunction();
  if (F.hasFnAttribute("interrupt")) {
    if (Subtarget.hasRISCV6464())
      return Subtarget.hasRISCV6464r6() ? CSR_Interrupt_64R6_SaveList
                                     : CSR_Interrupt_64_SaveList;
    else
      return Subtarget.hasRISCV6432r6() ? CSR_Interrupt_32R6_SaveList
                                     : CSR_Interrupt_32_SaveList;
  }

  if (Subtarget.isSingleFloat())
    return CSR_SingleFloatOnly_SaveList;

  if (Subtarget.isABI_N64())
    return CSR_N64_SaveList;

  if (Subtarget.isABI_N32())
    return CSR_N32_SaveList;

  if (Subtarget.isFP64bit())
    return CSR_O32_FP64_SaveList;

  if (Subtarget.isFPXX())
    return CSR_O32_FPXX_SaveList;

  return CSR_O32_SaveList;
}

const uint32_t *
RISCV64RegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const {
  const RISCV64Subtarget &Subtarget = MF.getSubtarget<RISCV64Subtarget>();
  if (Subtarget.isSingleFloat())
    return CSR_SingleFloatOnly_RegMask;

  if (Subtarget.isABI_N64())
    return CSR_N64_RegMask;

  if (Subtarget.isABI_N32())
    return CSR_N32_RegMask;

  if (Subtarget.isFP64bit())
    return CSR_O32_FP64_RegMask;

  if (Subtarget.isFPXX())
    return CSR_O32_FPXX_RegMask;

  return CSR_O32_RegMask;
}

const uint32_t *RISCV64RegisterInfo::getRISCV6416RetHelperMask() {
  return CSR_RISCV6416RetHelper_RegMask;
}

BitVector RISCV64RegisterInfo::
getReservedRegs(const MachineFunction &MF) const {
  static const MCPhysReg ReservedGPR32[] = {
    RISCV64::ZERO, RISCV64::K0, RISCV64::K1, RISCV64::SP
  };

  static const MCPhysReg ReservedGPR64[] = {
    RISCV64::ZERO_64, RISCV64::K0_64, RISCV64::K1_64, RISCV64::SP_64
  };

  BitVector Reserved(getNumRegs());
  const RISCV64Subtarget &Subtarget = MF.getSubtarget<RISCV64Subtarget>();

  using RegIter = TargetRegisterClass::const_iterator;

  for (unsigned I = 0; I < array_lengthof(ReservedGPR32); ++I)
    Reserved.set(ReservedGPR32[I]);

  // Reserve registers for the NaCl sandbox.
  if (Subtarget.isTargetNaCl()) {
    Reserved.set(RISCV64::T6);   // Reserved for control flow mask.
    Reserved.set(RISCV64::T7);   // Reserved for memory access mask.
    Reserved.set(RISCV64::T8);   // Reserved for thread pointer.
  }

  for (unsigned I = 0; I < array_lengthof(ReservedGPR64); ++I)
    Reserved.set(ReservedGPR64[I]);

  // For mno-abicalls, GP is a program invariant!
  if (!Subtarget.isABICalls()) {
    Reserved.set(RISCV64::GP);
    Reserved.set(RISCV64::GP_64);
  }

  if (Subtarget.isFP64bit()) {
    // Reserve all registers in AFGR64.
    for (RegIter Reg = RISCV64::AFGR64RegClass.begin(),
         EReg = RISCV64::AFGR64RegClass.end(); Reg != EReg; ++Reg)
      Reserved.set(*Reg);
  } else {
    // Reserve all registers in FGR64.
    for (RegIter Reg = RISCV64::FGR64RegClass.begin(),
         EReg = RISCV64::FGR64RegClass.end(); Reg != EReg; ++Reg)
      Reserved.set(*Reg);
  }
  // Reserve FP if this function should have a dedicated frame pointer register.
  if (Subtarget.getFrameLowering()->hasFP(MF)) {
    if (Subtarget.inRISCV6416Mode())
      Reserved.set(RISCV64::S0);
    else {
      Reserved.set(RISCV64::FP);
      Reserved.set(RISCV64::FP_64);

      // Reserve the base register if we need to both realign the stack and
      // allocate variable-sized objects at runtime. This should test the
      // same conditions as RISCV64FrameLowering::hasBP().
      if (needsStackRealignment(MF) &&
          MF.getFrameInfo().hasVarSizedObjects()) {
        Reserved.set(RISCV64::S7);
        Reserved.set(RISCV64::S7_64);
      }
    }
  }

  // Reserve hardware registers.
  Reserved.set(RISCV64::HWR29);

  // Reserve DSP control register.
  Reserved.set(RISCV64::DSPPos);
  Reserved.set(RISCV64::DSPSCount);
  Reserved.set(RISCV64::DSPCarry);
  Reserved.set(RISCV64::DSPEFI);
  Reserved.set(RISCV64::DSPOutFlag);

  // Reserve MSA control registers.
  Reserved.set(RISCV64::MSAIR);
  Reserved.set(RISCV64::MSACSR);
  Reserved.set(RISCV64::MSAAccess);
  Reserved.set(RISCV64::MSASave);
  Reserved.set(RISCV64::MSAModify);
  Reserved.set(RISCV64::MSARequest);
  Reserved.set(RISCV64::MSAMap);
  Reserved.set(RISCV64::MSAUnmap);

  // Reserve RA if in mips16 mode.
  if (Subtarget.inRISCV6416Mode()) {
    const RISCV64FunctionInfo *RISCV64FI = MF.getInfo<RISCV64FunctionInfo>();
    Reserved.set(RISCV64::RA);
    Reserved.set(RISCV64::RA_64);
    Reserved.set(RISCV64::T0);
    Reserved.set(RISCV64::T1);
    if (MF.getFunction().hasFnAttribute("saveS2") || RISCV64FI->hasSaveS2())
      Reserved.set(RISCV64::S2);
  }

  // Reserve GP if small section is used.
  if (Subtarget.useSmallSection()) {
    Reserved.set(RISCV64::GP);
    Reserved.set(RISCV64::GP_64);
  }

  if (Subtarget.isABI_O32() && !Subtarget.useOddSPReg()) {
    for (const auto &Reg : RISCV64::OddSPRegClass)
      Reserved.set(Reg);
  }

  return Reserved;
}

bool
RISCV64RegisterInfo::requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool
RISCV64RegisterInfo::trackLivenessAfterRegAlloc(const MachineFunction &MF) const {
  return true;
}

// FrameIndex represent objects inside a abstract stack.
// We must replace FrameIndex with an stack/frame pointer
// direct reference.
void RISCV64RegisterInfo::
eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj,
                    unsigned FIOperandNum, RegScavenger *RS) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();

  LLVM_DEBUG(errs() << "\nFunction : " << MF.getName() << "\n";
             errs() << "<--------->\n"
                    << MI);

  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  uint64_t stackSize = MF.getFrameInfo().getStackSize();
  int64_t spOffset = MF.getFrameInfo().getObjectOffset(FrameIndex);

  LLVM_DEBUG(errs() << "FrameIndex : " << FrameIndex << "\n"
                    << "spOffset   : " << spOffset << "\n"
                    << "stackSize  : " << stackSize << "\n"
                    << "alignment  : "
                    << MF.getFrameInfo().getObjectAlignment(FrameIndex)
                    << "\n");

  eliminateFI(MI, FIOperandNum, FrameIndex, stackSize, spOffset);
}

unsigned RISCV64RegisterInfo::
getFrameRegister(const MachineFunction &MF) const {
  const RISCV64Subtarget &Subtarget = MF.getSubtarget<RISCV64Subtarget>();
  const TargetFrameLowering *TFI = Subtarget.getFrameLowering();
  bool IsN64 =
      static_cast<const RISCV64TargetMachine &>(MF.getTarget()).getABI().IsN64();

  if (Subtarget.inRISCV6416Mode())
    return TFI->hasFP(MF) ? RISCV64::S0 : RISCV64::SP;
  else
    return TFI->hasFP(MF) ? (IsN64 ? RISCV64::FP_64 : RISCV64::FP) :
                            (IsN64 ? RISCV64::SP_64 : RISCV64::SP);
}

bool RISCV64RegisterInfo::canRealignStack(const MachineFunction &MF) const {
  // Avoid realigning functions that explicitly do not want to be realigned.
  // Normally, we should report an error when a function should be dynamically
  // realigned but also has the attribute no-realign-stack. Unfortunately,
  // with this attribute, MachineFrameInfo clamps each new object's alignment
  // to that of the stack's alignment as specified by the ABI. As a result,
  // the information of whether we have objects with larger alignment
  // requirement than the stack's alignment is already lost at this point.
  if (!TargetRegisterInfo::canRealignStack(MF))
    return false;

  const RISCV64Subtarget &Subtarget = MF.getSubtarget<RISCV64Subtarget>();
  unsigned FP = Subtarget.isGP32bit() ? RISCV64::FP : RISCV64::FP_64;
  unsigned BP = Subtarget.isGP32bit() ? RISCV64::S7 : RISCV64::S7_64;

  // Support dynamic stack realignment only for targets with standard encoding.
  if (!Subtarget.hasStandardEncoding())
    return false;

  // We can't perform dynamic stack realignment if we can't reserve the
  // frame pointer register.
  if (!MF.getRegInfo().canReserveReg(FP))
    return false;

  // We can realign the stack if we know the maximum call frame size and we
  // don't have variable sized objects.
  if (Subtarget.getFrameLowering()->hasReservedCallFrame(MF))
    return true;

  // We have to reserve the base pointer register in the presence of variable
  // sized objects.
  return MF.getRegInfo().canReserveReg(BP);
}
