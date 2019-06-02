//===-- MYRISCVXSEISelDAGToDAG.cpp - A Dag to Dag Inst Selector for MYRISCVXSE ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===-----------------------------------------------------------------------------===//
//
// Subclass of MYRISCVXDAGToDAGISel specialized for MYRISCVX32.
//
//===-----------------------------------------------------------------------------===//

#include "MYRISCVXSEISelDAGToDAG.h"

#include "MCTargetDesc/MYRISCVXBaseInfo.h"
#include "MYRISCVX.h"
#include "MYRISCVXMachineFunction.h"
#include "MYRISCVXRegisterInfo.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
using namespace llvm;

#define DEBUG_TYPE "MYRISCVX-isel"

bool MYRISCVXSEDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &static_cast<const MYRISCVXSubtarget &>(MF.getSubtarget());
  return MYRISCVXDAGToDAGISel::runOnMachineFunction(MF);
}

void MYRISCVXSEDAGToDAGISel::processFunctionAfterISel(MachineFunction &MF) {
}

//@selectNode
bool MYRISCVXSEDAGToDAGISel::trySelect(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();
  SDLoc DL(Node);

  ///
  // Instruction Selection not handled by the auto-generated
  // tablegen selection should be handled here.
  ///

  ///
  // Instruction Selection not handled by the auto-generated
  // tablegen selection should be handled here.
  ///
  // EVT NodeTy = Node->getValueType(0);
  // unsigned MultOpc;

  switch(Opcode) {
    default: break;

  }

  return false;
}

FunctionPass *llvm::createMYRISCVXSEISelDag(MYRISCVXTargetMachine &TM,
                                            CodeGenOpt::Level OptLevel) {
  return new MYRISCVXSEDAGToDAGISel(TM, OptLevel);
}
