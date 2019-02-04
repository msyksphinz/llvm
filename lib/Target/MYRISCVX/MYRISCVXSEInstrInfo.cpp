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

const MYRISCVXInstrInfo *llvm::createMYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI) {
  return new MYRISCVXSEInstrInfo(STI);
}


//@expandPostRAPseudo
/// Expand Pseudo instructions into real backend instructions
bool MYRISCVXSEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  //@expandPostRAPseudo-body
  MachineBasicBlock &MBB = *MI.getParent();
  switch (MI.getDesc().getOpcode()) {
    default:
      return false;
    case MYRISCVX::RetRA:
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
