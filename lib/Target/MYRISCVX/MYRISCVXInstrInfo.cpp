//===-- MYRISCVXInstrInfo.cpp - MYRISCVX Instruction Information ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXInstrInfo.h"

#include "MYRISCVXTargetMachine.h"
#include "MYRISCVXMachineFunction.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_CTOR_DTOR
#include "MYRISCVXGenInstrInfo.inc"

// Pin the vtable to this file.
void MYRISCVXInstrInfo::anchor() {}

//@MYRISCVXInstrInfo {
MYRISCVXInstrInfo::MYRISCVXInstrInfo(const MYRISCVXSubtarget &STI)
    :
      Subtarget(STI) {}

const MYRISCVXInstrInfo *MYRISCVXInstrInfo::create(MYRISCVXSubtarget &STI) {
  return llvm::createMYRISCVXSEInstrInfo(STI);
}

//@GetInstSizeInBytes {
/// Return the number of bytes of code the specified instruction may be.
unsigned MYRISCVXInstrInfo::GetInstSizeInBytes(const MachineInstr &MI) const {
//@GetInstSizeInBytes - body
  switch (MI.getOpcode()) {
  default:
    return MI.getDesc().getSize();
  }
}


void MYRISCVXInstrInfo::movImm32(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 const DebugLoc &DL, unsigned DstReg, uint64_t Val,
                                 MachineInstr::MIFlag Flag) const {

  assert(isInt<32>(Val) && "Can only materialize 32-bit constants");

  // TODO: If the value can be materialized using only one instruction, only
  // insert a single instruction.

  uint64_t Hi20 = ((Val + 0x800) >> 12) & 0xfffff;
  uint64_t Lo12 = SignExtend64<12>(Val);
  BuildMI(MBB, MBBI, DL, get(MYRISCVX::LUI), DstReg)
      .addImm(Hi20)
      .setMIFlag(Flag);
  BuildMI(MBB, MBBI, DL, get(MYRISCVX::ADDI), DstReg)
      .addReg(DstReg, RegState::Kill)
      .addImm(Lo12)
      .setMIFlag(Flag);
}


MachineMemOperand *
MYRISCVXInstrInfo::GetMemOperand(MachineBasicBlock &MBB, int FI,
                             MachineMemOperand::Flags Flags) const {
  MachineFunction  &MF  = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned Align = MFI.getObjectAlignment(FI);
  return MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(MF, FI),
                                 Flags, MFI.getObjectSize(FI), Align);
}
