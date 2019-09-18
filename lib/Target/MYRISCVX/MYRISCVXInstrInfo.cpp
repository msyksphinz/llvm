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
MYRISCVXInstrInfo::MYRISCVXInstrInfo() {}

//@GetInstSizeInBytes {
/// Return the number of bytes of code the specified instruction may be.
unsigned MYRISCVXInstrInfo::GetInstSizeInBytes(const MachineInstr &MI) const {
  //@GetInstSizeInBytes - body
  switch (MI.getOpcode()) {
    default:
      return MI.getDesc().getSize();
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


void MYRISCVXInstrInfo::expandRetRA(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(MYRISCVX::JALR))
      .addReg(MYRISCVX::ZERO).addReg(MYRISCVX::RA).addImm(0);
}
