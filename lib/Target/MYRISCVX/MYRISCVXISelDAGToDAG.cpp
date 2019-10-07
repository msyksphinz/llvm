//===-- MYRISCVXISelDAGToDAG.cpp - A Dag to Dag Inst Selector for MYRISCVX --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===------------------------------------------------------------------------------===//
//
// This file defines an instruction selector for the MYRISCVX target.
//
//===------------------------------------------------------------------------------===//

#include "MYRISCVXISelDAGToDAG.h"
#include "MYRISCVX.h"

#include "MYRISCVXMachineFunction.h"
#include "MYRISCVXRegisterInfo.h"
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
  Subtarget = &MF.getSubtarget<MYRISCVXSubtarget>();
  bool Ret = SelectionDAGISel::runOnMachineFunction(MF);

  return Ret;
}

/// getGlobalBaseReg - Output the instructions required to put the
/// GOT address into a register.
SDNode *MYRISCVXDAGToDAGISel::getGlobalBaseReg() {
  unsigned GlobalBaseReg = MF->getInfo<MYRISCVXFunctionInfo>()->getGlobalBaseReg();
  return CurDAG->getRegister(GlobalBaseReg, getTargetLowering()->getPointerTy(
      CurDAG->getDataLayout()))
      .getNode();
}


//@SelectAddr {
/// ComplexPattern used on MYRISCVXInstrInfo
/// Used on MYRISCVX Load/Store instructions
bool MYRISCVXDAGToDAGISel::
SelectAddr(SDValue Addr, SDValue &Base, SDValue &Offset) {
  //@SelectAddr }
  EVT ValTy = Addr.getValueType();
  SDLoc DL(Addr);

  // if Address is FI, get the TargetFrameIndex.
  if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
    Base   = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
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

  Base   = Addr;
  Offset = CurDAG->getTargetConstant(0, DL, ValTy);
  return true;
}


bool MYRISCVXDAGToDAGISel::SelectAddrFI(SDValue Addr, SDValue &Base) {
  LLVM_DEBUG(dbgs() << "MYRISCVXDAGToDAGISel::SelectAddrFI is called :");

  if (auto FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
    Base = CurDAG->getTargetFrameIndex(FIN->getIndex(),
                                       Subtarget->getXLenVT());
    LLVM_DEBUG(dbgs() << "True\n");
    return true;
  }

  LLVM_DEBUG(dbgs() << "False\n");
  return false;
}


//@Select {
/// Select instructions not customized! Used for
/// expanded, promoted and normal instructions
void MYRISCVXDAGToDAGISel::Select(SDNode *Node) {
  //@Select }
  unsigned Opcode = Node->getOpcode();

  // If we have a custom node, we already have selected!
  if (Node->isMachineOpcode()) {
    LLVM_DEBUG(errs() << "== "; Node->dump(CurDAG); errs() << "\n");
    Node->setNodeId(-1);
    return;
  }

  // See if subclasses can handle this node.
  if (trySelect(Node))
    return;

  MVT XLenVT = Subtarget->getXLenVT();
  SDLoc DL(Node);
  EVT VT = Node->getValueType(0);

  switch(Opcode) {
    // Get target GOT address.
    case ISD::GLOBAL_OFFSET_TABLE:
      ReplaceNode(Node, getGlobalBaseReg());
      return;
    case ISD::FrameIndex: {
      LLVM_DEBUG(dbgs() << "MYRISCVXDAGToDAGISel::Select:FrameIndex is called :");

      SDValue Imm = CurDAG->getTargetConstant(0, DL, XLenVT);
      int FI = cast<FrameIndexSDNode>(Node)->getIndex();
      SDValue TFI = CurDAG->getTargetFrameIndex(FI, VT);
      ReplaceNode(Node, CurDAG->getMachineNode(MYRISCVX::ADDI, DL, VT, TFI, Imm));
      return;
    }
    default: break;
  }

  // Select the default instruction
  SelectCode(Node);
}


//@selectNode
bool MYRISCVXDAGToDAGISel::trySelect(SDNode *Node) {
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

void MYRISCVXDAGToDAGISel::processFunctionAfterISel(MachineFunction &MF) {
}


FunctionPass *llvm::createMYRISCVXISelDag(MYRISCVXTargetMachine &TM,
                                          CodeGenOpt::Level OptLevel) {
  return new MYRISCVXDAGToDAGISel(TM, OptLevel);
}
