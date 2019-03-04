//===-- MYRISCVXSEISelDAGToDAG.cpp - A Dag to Dag Inst Selector for MYRISCVXSE ----===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of MYRISCVXDAGToDAGISel specialized for MYRISCVX32.
//
//===----------------------------------------------------------------------===//

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

void MYRISCVXSEDAGToDAGISel::selectAddESubE(unsigned MOp, SDValue InFlag,
                                            SDValue CmpLHS, const SDLoc &DL,
                                            SDNode *Node) const {
  unsigned Opc = InFlag.getOpcode(); (void)Opc;
  assert(((Opc == ISD::ADDC || Opc == ISD::ADDE) ||
          (Opc == ISD::SUBC || Opc == ISD::SUBE)) &&
         "(ADD|SUB)E flag operand must come from (ADD|SUB)C/E insn");

  SDValue Ops[] = { CmpLHS, InFlag.getOperand(1) };
  SDValue LHS = Node->getOperand(0), RHS = Node->getOperand(1);
  EVT VT = LHS.getValueType();

  SDNode *Carry;

  if (Subtarget->hasMYRISCVX32II()) {
    Carry = CurDAG->getMachineNode(MYRISCVX::SLTu, DL, VT, Ops);
  } else {
    SDNode *StatusWord = CurDAG->getMachineNode(MYRISCVX::CMP, DL, VT, Ops);
    SDValue Constant1 = CurDAG->getTargetConstant(1, DL, VT);
    Carry = CurDAG->getMachineNode(MYRISCVX::ANDi, DL, VT,
                                   SDValue(StatusWord,0), Constant1);
  }

  SDNode *AddCarry = CurDAG->getMachineNode(MYRISCVX::ADDu, DL, VT,
                                            SDValue(Carry,0), RHS);
  CurDAG->SelectNodeTo(Node, MOp, VT, MVT::Glue, LHS, SDValue(AddCarry,0));
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

  EVT NodeTy = Node->getValueType(0);
  unsigned MultOpc;
  switch(Opcode) {
    default: break;
    case ISD::SUBE: {
      SDValue InFlag = Node->getOperand(2);
      selectAddESubE(MYRISCVX::SUBu, InFlag, InFlag.getOperand(0), DL, Node);
      return true;
    }
    case ISD::ADDE: {
      SDValue InFlag = Node->getOperand(2);
      selectAddESubE(MYRISCVX::ADDu, InFlag, InFlag.getValue(0), DL, Node);
      return true;
    }
      /// Mul with two results
    case ISD::SMUL_LOHI:
    case ISD::UMUL_LOHI: {
      MultOpc = (Opcode == ISD::UMUL_LOHI ? MYRISCVX::MULTu : MYRISCVX::MULT);
      std::pair<SDNode*, SDNode*> LoHi =
          selectMULT(Node, MultOpc, DL, NodeTy, true, true);
      if (!SDValue(Node, 0).use_empty())
        ReplaceUses(SDValue(Node, 0), SDValue(LoHi.first, 0));
      if (!SDValue(Node, 1).use_empty())
        ReplaceUses(SDValue(Node, 1), SDValue(LoHi.second, 0));
      CurDAG->RemoveDeadNode(Node);
      return true;
    }
  }
  return true;
}


FunctionPass *llvm::createMYRISCVXSEISelDag(MYRISCVXTargetMachine &TM,
                                            CodeGenOpt::Level OptLevel) {
  return new MYRISCVXSEDAGToDAGISel(TM, OptLevel);
}
