//===-- MYRISCVXInstrInfo.cpp - MYRISCVX Instruction Information ----------===//
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
MYRISCVXInstrInfo::MYRISCVXInstrInfo()
    : MYRISCVXGenInstrInfo(MYRISCVX::ADJCALLSTACKDOWN, MYRISCVX::ADJCALLSTACKUP)
{}

//@GetInstSizeInBytes {
/// Return the number of bytes of code the specified instruction may be.
unsigned MYRISCVXInstrInfo::GetInstSizeInBytes(const MachineInstr &MI) const {
  //@GetInstSizeInBytes - body
  switch (MI.getOpcode()) {
    default:
      return MI.getDesc().getSize();
  }
}

void MYRISCVXInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator MBBI,
                                    const DebugLoc &DL, unsigned DstReg,
                                    unsigned SrcReg, bool KillSrc) const {
  if (MYRISCVX::GPRRegClass.contains(DstReg, SrcReg)) {
    BuildMI(MBB, MBBI, DL, get(MYRISCVX::ADDI), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addImm(0);
    return;
  }
}


//@expandPostRAPseudo
/// Expand Pseudo instructions into real backend instructions
bool MYRISCVXInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  //@expandPostRAPseudo-body
  MachineBasicBlock &MBB = *MI.getParent();

  switch (MI.getDesc().getOpcode()) {
    default:
      return false;
    case MYRISCVX::RetRA:
      expandRetRA(MBB, MI);
      break;
  }

  MBB.erase(MI);
  return true;
}


/// Adjust SP by Amount bytes.
void MYRISCVXInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                       MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator I) const {
  DebugLoc DL = I != MBB.end() ? I->getDebugLoc() : DebugLoc();
  unsigned ADD  = MYRISCVX::ADD;
  unsigned ADDI = MYRISCVX::ADDI;

  if (isInt<12>(Amount)) {
    // addiu sp, sp, amount
    BuildMI(MBB, I, DL, get(ADDI), SP).addReg(SP).addImm(Amount);
  } else { // Expand immediate that doesn't fit in 12-bit.
    // unsigned Reg = MRI.createVirtualRegister(&MYRISCVX::GPRRegClass);
    unsigned Reg = MYRISCVX::T0;
    loadImmediate(Amount, MBB, I, DL, Reg, nullptr);
    BuildMI(MBB, I, DL, get(ADD), SP).addReg(SP).addReg(Reg, RegState::Kill);
  }
}


/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
void
MYRISCVXInstrInfo::loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator II,
                                 const DebugLoc &DL, unsigned DstReg,
                                 unsigned *NewImm) const {
  uint64_t Hi20 = ((Imm + 0x800) >> 12) & 0xfffff;
  uint64_t Lo12 = SignExtend64<12>(Imm);
  BuildMI(MBB, II, DL, get(MYRISCVX::LUI), DstReg)
      .addImm(Hi20);
  BuildMI(MBB, II, DL, get(MYRISCVX::ADDI), DstReg)
      .addReg(DstReg, RegState::Kill)
      .addImm(Lo12);
}


void MYRISCVXInstrInfo::expandRetRA(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(MYRISCVX::JALR))
      .addReg(MYRISCVX::ZERO).addReg(MYRISCVX::RA).addImm(0);
}


MachineMemOperand *
MYRISCVXInstrInfo::GetMemOperand(MachineBasicBlock &MBB, int FI,
                                 MachineMemOperand::Flags Flags) const {

  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned Align = MFI.getObjectAlignment(FI);

  return MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(MF, FI),
                                 Flags, MFI.getObjectSize(FI), Align);
}


void MYRISCVXInstrInfo::
storeRegToStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                unsigned SrcReg, bool isKill, int FI,
                const TargetRegisterClass *RC, const TargetRegisterInfo *TRI,
                int64_t Offset) const {
  DebugLoc DL;
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);

  unsigned Opc = 0;

  Opc = MYRISCVX::SW;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill))
      .addFrameIndex(FI).addImm(Offset).addMemOperand(MMO);
}

void MYRISCVXInstrInfo::
loadRegFromStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                 unsigned DestReg, int FI, const TargetRegisterClass *RC,
                 const TargetRegisterInfo *TRI, int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
  unsigned Opc = 0;

  Opc = MYRISCVX::LW;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc), DestReg).addFrameIndex(FI).addImm(Offset)
      .addMemOperand(MMO);
}
