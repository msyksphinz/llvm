//===-- MYRISCVXISelDAGToDAG.cpp - A Dag to Dag Inst Selector for MYRISCVX --------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the MYRISCVX target.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXISelDAGToDAG.h"
#include "MYRISCVX.h"
#include "MYRISCVXMachineFunction.h"
#include "MYRISCVXRegisterInfo.h"
#include "MYRISCVXSEISelDAGToDAG.h"
#include "MYRISCVXTargetMachine.h"

#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
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

//===----------------------------------------------------------------------===//
// Instruction Selector Implementation
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// MYRISCVXDAGToDAGISel - MYRISCVX specific code to select MYRISCVX machine
// instructions for SelectionDAG operations.
//===----------------------------------------------------------------------===//
bool MYRISCVXDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  bool Ret = SelectionDAGISel::runOnMachineFunction(MF);
  return Ret;
}


//@SelectAddr {
/// ComplexPattern used on MYRISCVXInstrInfo
/// Used on MYRISCVX Load/Store instructions
bool MYRISCVXDAGToDAGISel::
SelectAddr(SDNode *Parent, SDValue Addr, SDValue &Base, SDValue &Offset) {
  //@SelectAddr }
  EVT ValTy = Addr.getValueType();
  SDLoc DL(Addr);
  // If Parent is an unaligned f32 load or store, select a (base + index)
  // floating point load/store instruction (luxc1 or suxc1).
  const LSBaseSDNode* LS = 0;
  if (Parent && (LS = dyn_cast<LSBaseSDNode>(Parent))) {
    EVT VT = LS->getMemoryVT();
    if (VT.getSizeInBits() / 8 > LS->getAlignment()) {
      assert("Unaligned loads/stores not supported for this type.");
      if (VT == MVT::f32)
        return false;
    }
  }
  // if Address is FI, get the TargetFrameIndex.
  if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
    Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
    Offset = CurDAG->getTargetConstant(0, DL, ValTy);
    return true;
  }


  // on PIC code Load GA
  if (Addr.getOpcode() == MYRISCVXISD::Wrapper) {
    Base   = Addr.getOperand(0);
    Offset = Addr.getOperand(1);
    return true;
  }

  //@static
  if (TM.getRelocationModel() != Reloc::PIC_) {
    if ((Addr.getOpcode() == ISD::TargetExternalSymbol ||
        Addr.getOpcode() == ISD::TargetGlobalAddress))
      return false;
  }

  Base = Addr;
  Offset = CurDAG->getTargetConstant(0, DL, ValTy);
  return true;
}


//@Select {
/// Select instructions not customized! Used for
/// expanded, promoted and normal instructions
void MYRISCVXDAGToDAGISel::Select(SDNode *Node) {
  //@Select }
  unsigned Opcode = Node->getOpcode();
  // Dump information about the Node being selected
  dbgs() << "Selecting: "; Node->dump(CurDAG); dbgs() << "\n";
  // If we have a custom node, we already have selected!
  if (Node->isMachineOpcode()) {
    dbgs() << "== "; Node->dump(CurDAG); dbgs() << "\n";
    Node->setNodeId(-1);
    return;
  }

  // See if subclasses can handle this node.
  if (trySelect(Node))
    return;
  switch(Opcode) {
    default: break;
      // Get target GOT address.
    case ISD::GLOBAL_OFFSET_TABLE:
      ReplaceNode(Node, getGlobalBaseReg());
      return;
  }

  // Select the default instruction
  SelectCode(Node);
}


/// getGlobalBaseReg - Output the instructions required to put the
/// GOT address into a register.
SDNode *MYRISCVXDAGToDAGISel::getGlobalBaseReg() {
  unsigned GlobalBaseReg = MF->getInfo<MYRISCVXFunctionInfo>()->getGlobalBaseReg();
  return CurDAG->getRegister(GlobalBaseReg, getTargetLowering()->getPointerTy(
      CurDAG->getDataLayout()))
      .getNode();
}
