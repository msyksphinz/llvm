//===-- MYRISCVXSEFrameLowering.cpp - MYRISCVX Frame Information ------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXSEFrameLowering.h"
#include "MYRISCVXMachineFunction.h"
#include "MYRISCVXSEInstrInfo.h"
#include "MYRISCVXSubtarget.h"

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

MYRISCVXSEFrameLowering::MYRISCVXSEFrameLowering(const MYRISCVXSubtarget &STI)
    : MYRISCVXFrameLowering(STI, STI.stackAlignment()) {}

//@emitPrologue {
void MYRISCVXSEFrameLowering::emitPrologue(MachineFunction &MF,
                                           MachineBasicBlock &MBB) const {
  assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");

  MachineFrameInfo &MFI = MF.getFrameInfo();
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();
  const MYRISCVXSEInstrInfo &TII =
      *static_cast<const MYRISCVXSEInstrInfo*>(STI.getInstrInfo());
  const MYRISCVXRegisterInfo &RegInfo =
      *static_cast<const MYRISCVXRegisterInfo *>(STI.getRegisterInfo());
  MachineBasicBlock::iterator MBBI = MBB.begin();

  DebugLoc dl = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  MYRISCVXABIInfo ABI = STI.getABI();
  unsigned SP = MYRISCVX::SP;
  unsigned FP = MYRISCVX::S0;
  unsigned ZERO = MYRISCVX::ZERO;
  unsigned ADD  = MYRISCVX::ADD;

  const TargetRegisterClass *RC = &MYRISCVX::GPRRegClass;

  // First, compute final stack size.
  uint64_t StackSize = MFI.getStackSize();
  // No need to allocate space on the stack.
  if (StackSize == 0 && !MFI.adjustsStack()) return;
  MachineModuleInfo &MMI = MF.getMMI();
  const MCRegisterInfo *MRI = MMI.getContext().getRegisterInfo();
  // MachineLocation DstML, SrcML;

  // Adjust stack.
  TII.adjustStackPtr(SP, -StackSize, MBB, MBBI);

  // emit ".cfi_def_cfa_offset StackSize"
  unsigned CFIIndex = MF.addFrameInst(
      MCCFIInstruction::createDefCfaOffset(nullptr, -StackSize));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  if (CSI.size()) {
    // Find the instruction past the last instruction that saves a callee-saved
    // register to the stack.
    for (unsigned i = 0; i < CSI.size(); ++i)
      ++MBBI;
    // Iterate over list of callee-saved registers and emit .cfi_offset
    // directives.
    for (std::vector<CalleeSavedInfo>::const_iterator I = CSI.begin(),
             E = CSI.end(); I != E; ++I) {
      int64_t Offset = MFI.getObjectOffset(I->getFrameIdx());
      unsigned Reg = I->getReg();
      {
        // Reg is in CPURegs.
        unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createOffset(
            nullptr, MRI->getDwarfRegNum(Reg, 1), Offset));
        BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
            .addCFIIndex(CFIIndex);
      }
    }
  }

  if (MYRISCVXFI->callsEhReturn()) {
    // Insert instructions that spill eh data registers.
    for (int I = 0; I < ABI.EhDataRegSize(); ++I) {
      if (!MBB.isLiveIn(ABI.GetEhDataReg(I)))
        MBB.addLiveIn(ABI.GetEhDataReg(I));
      TII.storeRegToStackSlot(MBB, MBBI, ABI.GetEhDataReg(I), false,
                              MYRISCVXFI->getEhDataRegFI(I), RC, &RegInfo);
    }

    // Emit .cfi_offset directives for eh data registers.
    for (int I = 0; I < ABI.EhDataRegSize(); ++I) {
      int64_t Offset = MFI.getObjectOffset(MYRISCVXFI->getEhDataRegFI(I));
      unsigned Reg = MRI->getDwarfRegNum(ABI.GetEhDataReg(I), true);
      unsigned CFIIndex = MF.addFrameInst(
          MCCFIInstruction::createOffset(nullptr, Reg, Offset));
      BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
          .addCFIIndex(CFIIndex);
    }
  }

  // if framepointer enabled, set it to point to the stack pointer.
  if (hasFP(MF)) {
    if (MYRISCVXFI->callsEhDwarf()) {
      BuildMI(MBB, MBBI, dl, TII.get(ADD), MYRISCVX::A0).addReg(FP).addReg(ZERO)
        .setMIFlag(MachineInstr::FrameSetup);
    }
    //@ Insert instruction "move $fp, $sp" at this location.
    BuildMI(MBB, MBBI, dl, TII.get(ADD), FP).addReg(SP).addReg(ZERO)
      .setMIFlag(MachineInstr::FrameSetup);

    // emit ".cfi_def_cfa_register $fp"
    unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createDefCfaRegister(
        nullptr, MRI->getDwarfRegNum(FP, true)));
    BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
        .addCFIIndex(CFIIndex);
  }

  //@ENABLE_GPRESTORE {
#ifdef ENABLE_GPRESTORE
  // Restore GP from the saved stack location
  if (MYRISCVXFI->needGPSaveRestore()) {
    unsigned Offset = MFI.getObjectOffset(MYRISCVXFI->getGPFI());
    BuildMI(MBB, MBBI, dl, TII.get(MYRISCVX::CPRESTORE)).addImm(Offset)
      .addReg(MYRISCVX::GP);
  }
#endif
//@ENABLE_GPRESTORE }

}

//}

//@emitEpilogue {
void MYRISCVXSEFrameLowering::emitEpilogue(MachineFunction &MF,
                                           MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();
  const MYRISCVXSEInstrInfo &TII =
      *static_cast<const MYRISCVXSEInstrInfo *>(STI.getInstrInfo());
  const MYRISCVXRegisterInfo &RegInfo =
      *static_cast<const MYRISCVXRegisterInfo *>(STI.getRegisterInfo());
  DebugLoc dl = MBBI->getDebugLoc();
  MYRISCVXABIInfo ABI = STI.getABI();

  unsigned SP = MYRISCVX::SP;
  unsigned FP = MYRISCVX::S0;
  unsigned ZERO = MYRISCVX::ZERO;
  unsigned ADD = MYRISCVX::ADD;

  // if framepointer enabled, restore the stack pointer.
  if (hasFP(MF)) {
    // Find the first instruction that restores a callee-saved register.
    MachineBasicBlock::iterator I = MBBI;
    for (unsigned i = 0; i < MFI.getCalleeSavedInfo().size(); ++i)
      --I;
    // Insert instruction "move $sp, $fp" at this location.
    BuildMI(MBB, I, dl, TII.get(ADD), SP).addReg(FP).addReg(ZERO);
  }

  if (MYRISCVXFI->callsEhReturn()) {
    const TargetRegisterClass *RC = &MYRISCVX::GPRRegClass;

    // Find first instruction that restores a callee-saved register.
    MachineBasicBlock::iterator I = MBBI;
    for (unsigned i = 0; i < MFI.getCalleeSavedInfo().size(); ++i)
      --I;

    // Insert instructions that restore eh data registers.
    for (int J = 0; J < ABI.EhDataRegSize(); ++J) {
      TII.loadRegFromStackSlot(MBB, I, ABI.GetEhDataReg(J),
                               MYRISCVXFI->getEhDataRegFI(J), RC, &RegInfo);
    }
  }

  // Get the number of bytes from FrameInfo
  uint64_t StackSize = MFI.getStackSize();

  if (!StackSize)
    return;
  // Adjust stack.
  TII.adjustStackPtr(SP, StackSize, MBB, MBBI);
}
//}

const MYRISCVXFrameLowering *
llvm::createMYRISCVXSEFrameLowering(const MYRISCVXSubtarget &ST) {
  return new MYRISCVXSEFrameLowering(ST);
}


bool
MYRISCVXSEFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  // Reserve call frame if the size of the maximum call frame fits into 16-bit
  // immediate field and there are no variable sized objects on the stack.
  // Make sure the second register scavenger spill slot can be accessed with one
  // instruction.
  return isInt<16>(MFI.getMaxCallFrameSize() + getStackAlignment()) &&
      !MFI.hasVarSizedObjects();
}


/// Mark \p Reg and all registers aliasing it in the bitset.
static void setAliasRegs(MachineFunction &MF, BitVector &SavedRegs, unsigned Reg) {
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  for (MCRegAliasIterator AI(Reg, TRI, true); AI.isValid(); ++AI)
    SavedRegs.set(*AI);
}


// This method is called immediately before PrologEpilogInserter scans the
// physical registers used to determine what callee saved registers should be
// spilled. This method is optional.
void MYRISCVXSEFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                                   BitVector &SavedRegs,
                                                   RegScavenger *RS) const {

  //@determineCalleeSaves-body
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();
  // MachineRegisterInfo& MRI = MF.getRegInfo();
  unsigned FP = MYRISCVX::S0;

  // Mark $fp as used if function has dedicated frame pointer.
  if (hasFP(MF))
    setAliasRegs(MF, SavedRegs, FP);

  //@callsEhReturn
  // Create spill slots for eh data registers if function calls eh_return.
  if (MYRISCVXFI->callsEhReturn())
    MYRISCVXFI->createEhDataRegsFI();

  if (MF.getFrameInfo().hasCalls())
    setAliasRegs(MF, SavedRegs, MYRISCVX::RA);

  return;
}


bool MYRISCVXSEFrameLowering::
spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI,
                          const std::vector<CalleeSavedInfo> &CSI,
                          const TargetRegisterInfo *TRI) const {
  MachineFunction *MF = MBB.getParent();
  MachineBasicBlock *EntryBlock = &MF->front();
  const MYRISCVXSEInstrInfo &TII =
      *static_cast<const MYRISCVXSEInstrInfo*>(STI.getInstrInfo());
  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
    // Add the callee-saved register as live-in. Do not add if the register is
    // LR and return address is taken, because it has already been added in
    // method MYRISCVXTargetLowering::LowerRETURNADDR.
    // It's killed at the spill, unless the register is LR and return address
    // is taken.
    unsigned Reg = CSI[i].getReg();
    bool IsRAAndRetAddrIsTaken = (Reg == MYRISCVX::RA)
        && MF->getFrameInfo().isReturnAddressTaken();
    if (!IsRAAndRetAddrIsTaken)
      EntryBlock->addLiveIn(Reg);
    // Insert the spill to the stack frame.
    bool IsKill = !IsRAAndRetAddrIsTaken;
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.storeRegToStackSlot(*EntryBlock, MI, Reg, IsKill,
                            CSI[i].getFrameIdx(), RC, TRI);
  }
  return true;
}
