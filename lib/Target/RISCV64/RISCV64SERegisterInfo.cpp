//===-- RISCV64SERegisterInfo.cpp - MIPS32/64 Register Information -== -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MIPS32/64 implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#include "RISCV64SERegisterInfo.h"
#include "RISCV64.h"
#include "RISCV64MachineFunction.h"
#include "RISCV64SEInstrInfo.h"
#include "RISCV64Subtarget.h"
#include "RISCV64TargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DebugInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

#define DEBUG_TYPE "mips-reg-info"

RISCV64SERegisterInfo::RISCV64SERegisterInfo() : RISCV64RegisterInfo() {}

bool RISCV64SERegisterInfo::
requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool RISCV64SERegisterInfo::
requiresFrameIndexScavenging(const MachineFunction &MF) const {
  return true;
}

const TargetRegisterClass *
RISCV64SERegisterInfo::intRegClass(unsigned Size) const {
  if (Size == 4)
    return &RISCV64::GPR32RegClass;

  assert(Size == 8);
  return &RISCV64::GPR64RegClass;
}

/// Get the size of the offset supported by the given load/store/inline asm.
/// The result includes the effects of any scale factors applied to the
/// instruction immediate.
static inline unsigned getLoadStoreOffsetSizeInBits(const unsigned Opcode,
                                                    MachineOperand MO) {
  switch (Opcode) {
  case RISCV64::LD_B:
  case RISCV64::ST_B:
    return 10;
  case RISCV64::LD_H:
  case RISCV64::ST_H:
    return 10 + 1 /* scale factor */;
  case RISCV64::LD_W:
  case RISCV64::ST_W:
    return 10 + 2 /* scale factor */;
  case RISCV64::LD_D:
  case RISCV64::ST_D:
    return 10 + 3 /* scale factor */;
  case RISCV64::LL:
  case RISCV64::LL64:
  case RISCV64::LLD:
  case RISCV64::LLE:
  case RISCV64::SC:
  case RISCV64::SC64:
  case RISCV64::SCD:
  case RISCV64::SCE:
    return 16;
  case RISCV64::LLE_MM:
  case RISCV64::LL_MM:
  case RISCV64::SCE_MM:
  case RISCV64::SC_MM:
    return 12;
  case RISCV64::LL64_R6:
  case RISCV64::LL_R6:
  case RISCV64::LLD_R6:
  case RISCV64::SC64_R6:
  case RISCV64::SCD_R6:
  case RISCV64::SC_R6:
  case RISCV64::LL_MMR6:
  case RISCV64::SC_MMR6:
    return 9;
  case RISCV64::INLINEASM: {
    unsigned ConstraintID = InlineAsm::getMemoryConstraintID(MO.getImm());
    switch (ConstraintID) {
    case InlineAsm::Constraint_ZC: {
      const RISCV64Subtarget &Subtarget = MO.getParent()
                                           ->getParent()
                                           ->getParent()
                                           ->getSubtarget<RISCV64Subtarget>();
      if (Subtarget.inMicroRISCV64Mode())
        return 12;

      if (Subtarget.hasRISCV6432r6())
        return 9;

      return 16;
    }
    default:
      return 16;
    }
  }
  default:
    return 16;
  }
}

/// Get the scale factor applied to the immediate in the given load/store.
static inline unsigned getLoadStoreOffsetAlign(const unsigned Opcode) {
  switch (Opcode) {
  case RISCV64::LD_H:
  case RISCV64::ST_H:
    return 2;
  case RISCV64::LD_W:
  case RISCV64::ST_W:
    return 4;
  case RISCV64::LD_D:
  case RISCV64::ST_D:
    return 8;
  default:
    return 1;
  }
}

void RISCV64SERegisterInfo::eliminateFI(MachineBasicBlock::iterator II,
                                     unsigned OpNo, int FrameIndex,
                                     uint64_t StackSize,
                                     int64_t SPOffset) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  RISCV64FunctionInfo *RISCV64FI = MF.getInfo<RISCV64FunctionInfo>();

  RISCV64ABIInfo ABI =
      static_cast<const RISCV64TargetMachine &>(MF.getTarget()).getABI();
  const RISCV64RegisterInfo *RegInfo =
    static_cast<const RISCV64RegisterInfo *>(MF.getSubtarget().getRegisterInfo());

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  int MinCSFI = 0;
  int MaxCSFI = -1;

  if (CSI.size()) {
    MinCSFI = CSI[0].getFrameIdx();
    MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  }

  bool EhDataRegFI = RISCV64FI->isEhDataRegFI(FrameIndex);
  bool IsISRRegFI = RISCV64FI->isISRRegFI(FrameIndex);
  // The following stack frame objects are always referenced relative to $sp:
  //  1. Outgoing arguments.
  //  2. Pointer to dynamically allocated stack space.
  //  3. Locations for callee-saved registers.
  //  4. Locations for eh data registers.
  //  5. Locations for ISR saved Coprocessor 0 registers 12 & 14.
  // Everything else is referenced relative to whatever register
  // getFrameRegister() returns.
  unsigned FrameReg;

  if ((FrameIndex >= MinCSFI && FrameIndex <= MaxCSFI) || EhDataRegFI ||
      IsISRRegFI)
    FrameReg = ABI.GetStackPtr();
  else if (RegInfo->needsStackRealignment(MF)) {
    if (MFI.hasVarSizedObjects() && !MFI.isFixedObjectIndex(FrameIndex))
      FrameReg = ABI.GetBasePtr();
    else if (MFI.isFixedObjectIndex(FrameIndex))
      FrameReg = getFrameRegister(MF);
    else
      FrameReg = ABI.GetStackPtr();
  } else
    FrameReg = getFrameRegister(MF);

  // Calculate final offset.
  // - There is no need to change the offset if the frame object is one of the
  //   following: an outgoing argument, pointer to a dynamically allocated
  //   stack space or a $gp restore location,
  // - If the frame object is any of the following, its offset must be adjusted
  //   by adding the size of the stack:
  //   incoming argument, callee-saved register location or local variable.
  bool IsKill = false;
  int64_t Offset;

  Offset = SPOffset + (int64_t)StackSize;
  Offset += MI.getOperand(OpNo + 1).getImm();

  LLVM_DEBUG(errs() << "Offset     : " << Offset << "\n"
                    << "<--------->\n");

  if (!MI.isDebugValue()) {
    // Make sure Offset fits within the field available.
    // For MSA instructions, this is a 10-bit signed immediate (scaled by
    // element size), otherwise it is a 16-bit signed immediate.
    unsigned OffsetBitSize =
        getLoadStoreOffsetSizeInBits(MI.getOpcode(), MI.getOperand(OpNo - 1));
    unsigned OffsetAlign = getLoadStoreOffsetAlign(MI.getOpcode());

    if (OffsetBitSize < 16 && isInt<16>(Offset) &&
        (!isIntN(OffsetBitSize, Offset) ||
         OffsetToAlignment(Offset, OffsetAlign) != 0)) {
      // If we have an offset that needs to fit into a signed n-bit immediate
      // (where n < 16) and doesn't, but does fit into 16-bits then use an ADDiu
      MachineBasicBlock &MBB = *MI.getParent();
      DebugLoc DL = II->getDebugLoc();
      const TargetRegisterClass *PtrRC =
          ABI.ArePtrs64bit() ? &RISCV64::GPR64RegClass : &RISCV64::GPR32RegClass;
      MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
      unsigned Reg = RegInfo.createVirtualRegister(PtrRC);
      const RISCV64SEInstrInfo &TII =
          *static_cast<const RISCV64SEInstrInfo *>(
              MBB.getParent()->getSubtarget().getInstrInfo());
      BuildMI(MBB, II, DL, TII.get(ABI.GetPtrAddiuOp()), Reg)
          .addReg(FrameReg)
          .addImm(Offset);

      FrameReg = Reg;
      Offset = 0;
      IsKill = true;
    } else if (!isInt<16>(Offset)) {
      // Otherwise split the offset into 16-bit pieces and add it in multiple
      // instructions.
      MachineBasicBlock &MBB = *MI.getParent();
      DebugLoc DL = II->getDebugLoc();
      unsigned NewImm = 0;
      const RISCV64SEInstrInfo &TII =
          *static_cast<const RISCV64SEInstrInfo *>(
              MBB.getParent()->getSubtarget().getInstrInfo());
      unsigned Reg = TII.loadImmediate(Offset, MBB, II, DL,
                                       OffsetBitSize == 16 ? &NewImm : nullptr);
      BuildMI(MBB, II, DL, TII.get(ABI.GetPtrAdduOp()), Reg).addReg(FrameReg)
        .addReg(Reg, RegState::Kill);

      FrameReg = Reg;
      Offset = SignExtend64<16>(NewImm);
      IsKill = true;
    }
  }

  MI.getOperand(OpNo).ChangeToRegister(FrameReg, false, false, IsKill);
  MI.getOperand(OpNo + 1).ChangeToImmediate(Offset);
}
