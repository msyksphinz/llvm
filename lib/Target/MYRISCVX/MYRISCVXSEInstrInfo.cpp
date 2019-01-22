//===-- MYRISCVXSEInstrInfo.cpp - MYRISCVX32/64 Instruction Information -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX32/64 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXSEInstrInfo.h"

#include "MYRISCVXMachineFunction.h"
#include "MYRISCVXTargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

MYRISCVXSEInstrInfo::MYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI)
    : MYRISCVXInstrInfo(STI),
      RI(STI) {}

const MYRISCVXRegisterInfo &MYRISCVXSEInstrInfo::getRegisterInfo() const {
  return RI;
}

//@expandPostRAPseudo
/// Expand Pseudo instructions into real backend instructions
bool MYRISCVXSEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
//@expandPostRAPseudo-body
  MachineBasicBlock &MBB = *MI.getParent();

  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case MYRISCVX::RetLR:
    expandRetLR(MBB, MI);
    break;
  }

  MBB.erase(MI);
  return true;
}

void MYRISCVXSEInstrInfo::expandRetLR(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(MYRISCVX::RET)).addReg(MYRISCVX::RA);
}


const MYRISCVXInstrInfo *llvm::createMYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI) {
  return new MYRISCVXSEInstrInfo(STI);
}


void MYRISCVXSEInstrInfo::
storeRegToStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                unsigned SrcReg, bool isKill, int FI,
                const TargetRegisterClass *RC, const TargetRegisterInfo *TRI,
                int64_t Offset) const {
  DebugLoc DL;
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);
  unsigned Opc = 0;
  Opc = MYRISCVX::ST;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill))
      .addFrameIndex(FI).addImm(Offset).addMemOperand(MMO);
}
void MYRISCVXSEInstrInfo::
loadRegFromStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                 unsigned DestReg, int FI, const TargetRegisterClass *RC,
                 const TargetRegisterInfo *TRI, int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
  unsigned Opc = 0;
  Opc = MYRISCVX::LD;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc), DestReg).addFrameIndex(FI).addImm(Offset)
      .addMemOperand(MMO);
}
