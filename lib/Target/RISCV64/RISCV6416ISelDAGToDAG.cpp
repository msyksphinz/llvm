//===-- RISCV6416ISelDAGToDAG.cpp - A Dag to Dag Inst Selector for RISCV6416 ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of RISCV64DAGToDAGISel specialized for mips16.
//
//===----------------------------------------------------------------------===//

#include "RISCV6416ISelDAGToDAG.h"
#include "MCTargetDesc/RISCV64BaseInfo.h"
#include "RISCV64.h"
#include "RISCV64MachineFunction.h"
#include "RISCV64RegisterInfo.h"
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

#define DEBUG_TYPE "mips-isel"

bool RISCV6416DAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &static_cast<const RISCV64Subtarget &>(MF.getSubtarget());
  if (!Subtarget->inRISCV6416Mode())
    return false;
  return RISCV64DAGToDAGISel::runOnMachineFunction(MF);
}
/// Select multiply instructions.
std::pair<SDNode *, SDNode *>
RISCV6416DAGToDAGISel::selectMULT(SDNode *N, unsigned Opc, const SDLoc &DL, EVT Ty,
                               bool HasLo, bool HasHi) {
  SDNode *Lo = nullptr, *Hi = nullptr;
  SDNode *Mul = CurDAG->getMachineNode(Opc, DL, MVT::Glue, N->getOperand(0),
                                       N->getOperand(1));
  SDValue InFlag = SDValue(Mul, 0);

  if (HasLo) {
    unsigned Opcode = RISCV64::Mflo16;
    Lo = CurDAG->getMachineNode(Opcode, DL, Ty, MVT::Glue, InFlag);
    InFlag = SDValue(Lo, 1);
  }
  if (HasHi) {
    unsigned Opcode = RISCV64::Mfhi16;
    Hi = CurDAG->getMachineNode(Opcode, DL, Ty, InFlag);
  }
  return std::make_pair(Lo, Hi);
}

void RISCV6416DAGToDAGISel::initGlobalBaseReg(MachineFunction &MF) {
  RISCV64FunctionInfo *RISCV64FI = MF.getInfo<RISCV64FunctionInfo>();

  if (!RISCV64FI->globalBaseRegSet())
    return;

  MachineBasicBlock &MBB = MF.front();
  MachineBasicBlock::iterator I = MBB.begin();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();
  const TargetInstrInfo &TII = *Subtarget->getInstrInfo();
  DebugLoc DL;
  unsigned V0, V1, V2, GlobalBaseReg = RISCV64FI->getGlobalBaseReg();
  const TargetRegisterClass *RC = &RISCV64::CPU16RegsRegClass;

  V0 = RegInfo.createVirtualRegister(RC);
  V1 = RegInfo.createVirtualRegister(RC);
  V2 = RegInfo.createVirtualRegister(RC);


  BuildMI(MBB, I, DL, TII.get(RISCV64::LiRxImmX16), V0)
      .addExternalSymbol("_gp_disp", RISCV64II::MO_ABS_HI);
  BuildMI(MBB, I, DL, TII.get(RISCV64::AddiuRxPcImmX16), V1)
      .addExternalSymbol("_gp_disp", RISCV64II::MO_ABS_LO);

  BuildMI(MBB, I, DL, TII.get(RISCV64::SllX16), V2).addReg(V0).addImm(16);
  BuildMI(MBB, I, DL, TII.get(RISCV64::AdduRxRyRz16), GlobalBaseReg)
      .addReg(V1)
      .addReg(V2);
}

void RISCV6416DAGToDAGISel::processFunctionAfterISel(MachineFunction &MF) {
  initGlobalBaseReg(MF);
}

bool RISCV6416DAGToDAGISel::selectAddr(bool SPAllowed, SDValue Addr, SDValue &Base,
                                    SDValue &Offset) {
  SDLoc DL(Addr);
  EVT ValTy = Addr.getValueType();

  // if Address is FI, get the TargetFrameIndex.
  if (SPAllowed) {
    if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
      Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
      Offset = CurDAG->getTargetConstant(0, DL, ValTy);
      return true;
    }
  }
  // on PIC code Load GA
  if (Addr.getOpcode() == RISCV64ISD::Wrapper) {
    Base = Addr.getOperand(0);
    Offset = Addr.getOperand(1);
    return true;
  }
  if (!TM.isPositionIndependent()) {
    if ((Addr.getOpcode() == ISD::TargetExternalSymbol ||
         Addr.getOpcode() == ISD::TargetGlobalAddress))
      return false;
  }
  // Addresses of the form FI+const or FI|const
  if (CurDAG->isBaseWithConstantOffset(Addr)) {
    ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Addr.getOperand(1));
    if (isInt<16>(CN->getSExtValue())) {
      // If the first operand is a FI, get the TargetFI Node
      if (SPAllowed) {
        if (FrameIndexSDNode *FIN =
                dyn_cast<FrameIndexSDNode>(Addr.getOperand(0))) {
          Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
          Offset = CurDAG->getTargetConstant(CN->getZExtValue(), DL, ValTy);
          return true;
        }
      }

      Base = Addr.getOperand(0);
      Offset = CurDAG->getTargetConstant(CN->getZExtValue(), DL, ValTy);
      return true;
    }
  }
  // Operand is a result from an ADD.
  if (Addr.getOpcode() == ISD::ADD) {
    // When loading from constant pools, load the lower address part in
    // the instruction itself. Example, instead of:
    //  lui $2, %hi($CPI1_0)
    //  addiu $2, $2, %lo($CPI1_0)
    //  lwc1 $f0, 0($2)
    // Generate:
    //  lui $2, %hi($CPI1_0)
    //  lwc1 $f0, %lo($CPI1_0)($2)
    if (Addr.getOperand(1).getOpcode() == RISCV64ISD::Lo ||
        Addr.getOperand(1).getOpcode() == RISCV64ISD::GPRel) {
      SDValue Opnd0 = Addr.getOperand(1).getOperand(0);
      if (isa<ConstantPoolSDNode>(Opnd0) || isa<GlobalAddressSDNode>(Opnd0) ||
          isa<JumpTableSDNode>(Opnd0)) {
        Base = Addr.getOperand(0);
        Offset = Opnd0;
        return true;
      }
    }
  }
  Base = Addr;
  Offset = CurDAG->getTargetConstant(0, DL, ValTy);
  return true;
}

bool RISCV6416DAGToDAGISel::selectAddr16(SDValue Addr, SDValue &Base,
                                      SDValue &Offset) {
  return selectAddr(false, Addr, Base, Offset);
}

bool RISCV6416DAGToDAGISel::selectAddr16SP(SDValue Addr, SDValue &Base,
                                        SDValue &Offset) {
  return selectAddr(true, Addr, Base, Offset);
}

/// Select instructions not customized! Used for
/// expanded, promoted and normal instructions
bool RISCV6416DAGToDAGISel::trySelect(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();
  SDLoc DL(Node);

  ///
  // Instruction Selection not handled by the auto-generated
  // tablegen selection should be handled here.
  ///
  EVT NodeTy = Node->getValueType(0);
  unsigned MultOpc;

  switch (Opcode) {
  default:
    break;

  /// Mul with two results
  case ISD::SMUL_LOHI:
  case ISD::UMUL_LOHI: {
    MultOpc = (Opcode == ISD::UMUL_LOHI ? RISCV64::MultuRxRy16 : RISCV64::MultRxRy16);
    std::pair<SDNode *, SDNode *> LoHi =
        selectMULT(Node, MultOpc, DL, NodeTy, true, true);
    if (!SDValue(Node, 0).use_empty())
      ReplaceUses(SDValue(Node, 0), SDValue(LoHi.first, 0));

    if (!SDValue(Node, 1).use_empty())
      ReplaceUses(SDValue(Node, 1), SDValue(LoHi.second, 0));

    CurDAG->RemoveDeadNode(Node);
    return true;
  }

  case ISD::MULHS:
  case ISD::MULHU: {
    MultOpc = (Opcode == ISD::MULHU ? RISCV64::MultuRxRy16 : RISCV64::MultRxRy16);
    auto LoHi = selectMULT(Node, MultOpc, DL, NodeTy, false, true);
    ReplaceNode(Node, LoHi.second);
    return true;
  }
  }

  return false;
}

FunctionPass *llvm::createRISCV6416ISelDag(RISCV64TargetMachine &TM,
                                        CodeGenOpt::Level OptLevel) {
  return new RISCV6416DAGToDAGISel(TM, OptLevel);
}
