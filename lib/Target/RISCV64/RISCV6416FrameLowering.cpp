//===- RISCV6416FrameLowering.cpp - RISCV6416 Frame Information -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RISCV6416 implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "RISCV6416FrameLowering.h"
#include "MCTargetDesc/RISCV64BaseInfo.h"
#include "RISCV6416InstrInfo.h"
#include "RISCV64InstrInfo.h"
#include "RISCV64RegisterInfo.h"
#include "RISCV64Subtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDwarf.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include <cassert>
#include <cstdint>
#include <vector>

using namespace llvm;

RISCV6416FrameLowering::RISCV6416FrameLowering(const RISCV64Subtarget &STI)
    : RISCV64FrameLowering(STI, STI.getStackAlignment()) {}

void RISCV6416FrameLowering::emitPrologue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const RISCV6416InstrInfo &TII =
      *static_cast<const RISCV6416InstrInfo *>(STI.getInstrInfo());
  MachineBasicBlock::iterator MBBI = MBB.begin();

  // Debug location must be unknown since the first debug location is used
  // to determine the end of the prologue.
  DebugLoc dl;

  uint64_t StackSize = MFI.getStackSize();

  // No need to allocate space on the stack.
  if (StackSize == 0 && !MFI.adjustsStack()) return;

  MachineModuleInfo &MMI = MF.getMMI();
  const MCRegisterInfo *MRI = MMI.getContext().getRegisterInfo();

  // Adjust stack.
  TII.makeFrame(RISCV64::SP, StackSize, MBB, MBBI);

  // emit ".cfi_def_cfa_offset StackSize"
  unsigned CFIIndex = MF.addFrameInst(
      MCCFIInstruction::createDefCfaOffset(nullptr, -StackSize));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();

  if (!CSI.empty()) {
    const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();

    for (std::vector<CalleeSavedInfo>::const_iterator I = CSI.begin(),
         E = CSI.end(); I != E; ++I) {
      int64_t Offset = MFI.getObjectOffset(I->getFrameIdx());
      unsigned Reg = I->getReg();
      unsigned DReg = MRI->getDwarfRegNum(Reg, true);
      unsigned CFIIndex = MF.addFrameInst(
          MCCFIInstruction::createOffset(nullptr, DReg, Offset));
      BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
          .addCFIIndex(CFIIndex);
    }
  }
  if (hasFP(MF))
    BuildMI(MBB, MBBI, dl, TII.get(RISCV64::MoveR3216), RISCV64::S0)
      .addReg(RISCV64::SP).setMIFlag(MachineInstr::FrameSetup);
}

void RISCV6416FrameLowering::emitEpilogue(MachineFunction &MF,
                                 MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.getFirstTerminator();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const RISCV6416InstrInfo &TII =
      *static_cast<const RISCV6416InstrInfo *>(STI.getInstrInfo());
  DebugLoc dl = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  uint64_t StackSize = MFI.getStackSize();

  if (!StackSize)
    return;

  if (hasFP(MF))
    BuildMI(MBB, MBBI, dl, TII.get(RISCV64::Move32R16), RISCV64::SP)
      .addReg(RISCV64::S0);

  // Adjust stack.
  // assumes stacksize multiple of 8
  TII.restoreFrame(RISCV64::SP, StackSize, MBB, MBBI);
}

bool RISCV6416FrameLowering::
spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI,
                          const std::vector<CalleeSavedInfo> &CSI,
                          const TargetRegisterInfo *TRI) const {
  MachineFunction *MF = MBB.getParent();

  //
  // Registers RA, S0,S1 are the callee saved registers and they
  // will be saved with the "save" instruction
  // during emitPrologue
  //
  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
    // Add the callee-saved register as live-in. Do not add if the register is
    // RA and return address is taken, because it has already been added in
    // method RISCV64TargetLowering::lowerRETURNADDR.
    // It's killed at the spill, unless the register is RA and return address
    // is taken.
    unsigned Reg = CSI[i].getReg();
    bool IsRAAndRetAddrIsTaken = (Reg == RISCV64::RA)
      && MF->getFrameInfo().isReturnAddressTaken();
    if (!IsRAAndRetAddrIsTaken)
      MBB.addLiveIn(Reg);
  }

  return true;
}

bool RISCV6416FrameLowering::restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator MI,
                                       std::vector<CalleeSavedInfo> &CSI,
                                       const TargetRegisterInfo *TRI) const {
  //
  // Registers RA,S0,S1 are the callee saved registers and they will be restored
  // with the restore instruction during emitEpilogue.
  // We need to override this virtual function, otherwise llvm will try and
  // restore the registers on it's on from the stack.
  //

  return true;
}

bool
RISCV6416FrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  // Reserve call frame if the size of the maximum call frame fits into 15-bit
  // immediate field and there are no variable sized objects on the stack.
  return isInt<15>(MFI.getMaxCallFrameSize()) && !MFI.hasVarSizedObjects();
}

void RISCV6416FrameLowering::determineCalleeSaves(MachineFunction &MF,
                                               BitVector &SavedRegs,
                                               RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  const RISCV6416InstrInfo &TII =
      *static_cast<const RISCV6416InstrInfo *>(STI.getInstrInfo());
  const RISCV64RegisterInfo &RI = TII.getRegisterInfo();
  const BitVector Reserved = RI.getReservedRegs(MF);
  bool SaveS2 = Reserved[RISCV64::S2];
  if (SaveS2)
    SavedRegs.set(RISCV64::S2);
  if (hasFP(MF))
    SavedRegs.set(RISCV64::S0);
}

const RISCV64FrameLowering *
llvm::createRISCV6416FrameLowering(const RISCV64Subtarget &ST) {
  return new RISCV6416FrameLowering(ST);
}
