//===-- MYRISCVXSEFrameLowering.cpp - MYRISCVX Frame Information ----------===//
//
//                     The LLVM Compiler Infrastructure
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
  MachineFrameInfo MFI    = MF.getFrameInfo();
  // MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();

  const MYRISCVXSEInstrInfo &TII =
      *static_cast<const MYRISCVXSEInstrInfo*>(STI.getInstrInfo());
  // const MYRISCVXRegisterInfo &RegInfo =
  //     *static_cast<const MYRISCVXRegisterInfo *>(STI.getRegisterInfo());

  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc dl = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  // MYRISCVXABIInfo ABI = STI.getABI();
  unsigned SP = MYRISCVX::SP;
  // const TargetRegisterClass *RC = &MYRISCVX::GPRRegClass;

  // First, compute final stack size.
  uint64_t StackSize = MFI.getStackSize();

  // No need to allocate space on the stack.
  if (StackSize == 0 && !MFI.adjustsStack()) return;

  MachineModuleInfo &MMI = MF.getMMI();
  const MCRegisterInfo *MRI = MMI.getContext().getRegisterInfo();

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
}
//}

//@emitEpilogue {
void MYRISCVXSEFrameLowering::emitEpilogue(MachineFunction &MF,
                                           MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  MachineFrameInfo MFI             = MF.getFrameInfo();
  // MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();

  const MYRISCVXSEInstrInfo &TII =
      *static_cast<const MYRISCVXSEInstrInfo *>(STI.getInstrInfo());
  // const MYRISCVXRegisterInfo &RegInfo =
  //     *static_cast<const MYRISCVXRegisterInfo *>(STI.getRegisterInfo());

  DebugLoc dl = MBBI->getDebugLoc();
  // MYRISCVXABIInfo ABI = STI.getABI();
  unsigned SP = MYRISCVX::SP;

  // Get the number of bytes from FrameInfo
  uint64_t StackSize = MFI.getStackSize();

  if (!StackSize)
    return;

  // Adjust stack.
  TII.adjustStackPtr(SP, StackSize, MBB, MBBI);
}
//}


bool
MYRISCVXSEFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  const MachineFrameInfo MFI = MF.getFrameInfo();

  // Reserve call frame if the size of the maximum call frame fits into 16-bit
  // immediate field and there are no variable sized objects on the stack.
  // Make sure the second register scavenger spill slot can be accessed with one
  // instruction.

  return isInt<16>(MFI.getMaxCallFrameSize() + getStackAlignment()) &&
      !MFI.hasVarSizedObjects();
}


/// Mark \p Reg and all registers aliasing it in the bitset.
static void setAliasRegs(MachineFunction &MF, BitVector &SavedRegs,
                         unsigned Reg) {
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  for (MCRegAliasIterator AI(Reg, TRI, true); AI.isValid(); ++AI)
    SavedRegs.set(*AI);
}


// This method is called immediately before PrologEpilogInserter scans the
//  physical registers used to determine what callee saved registers should be
//  spilled. This method is optional.
void MYRISCVXSEFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                                   BitVector &SavedRegs,
                                                   RegScavenger *RS) const {
  //@determineCalleeSaves-body
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  // MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();
  // MachineRegisterInfo& MRI = MF.getRegInfo();

  if (MF.getFrameInfo().hasCalls())
    setAliasRegs(MF, SavedRegs, MYRISCVX::RA);

  return;
}


const MYRISCVXFrameLowering *
llvm::createMYRISCVXSEFrameLowering(const MYRISCVXSubtarget &ST) {
  return new MYRISCVXSEFrameLowering(ST);
}
