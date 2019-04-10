//===-- MYRISCVXInstrInfo.cpp - MYRISCVX Instruction Information ------------------===//
//
// The LLVM Compiler Infrastructure
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
    MYRISCVXGenInstrInfo(MYRISCVX::ADJCALLSTACKDOWN, MYRISCVX::ADJCALLSTACKUP),
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
    case  TargetOpcode::INLINEASM: {       // Inline Asm: Variable size.
      const MachineFunction *MF = MI.getParent()->getParent();
      const char *AsmStr = MI.getOperand(0).getSymbolName();
      return getInlineAsmLength(AsmStr, *MF->getTarget().getMCAsmInfo());
    }
  }
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
