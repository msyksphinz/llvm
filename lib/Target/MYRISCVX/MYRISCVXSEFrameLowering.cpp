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
// #include "MYRISCVXSEInstrInfo.h"
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
}
//}

//@emitEpilogue {
void MYRISCVXSEFrameLowering::emitEpilogue(MachineFunction &MF,
                                           MachineBasicBlock &MBB) const {
}
//}

const MYRISCVXFrameLowering *
llvm::createMYRISCVXSEFrameLowering(const MYRISCVXSubtarget &ST) {
  return new MYRISCVXSEFrameLowering(ST);
}
