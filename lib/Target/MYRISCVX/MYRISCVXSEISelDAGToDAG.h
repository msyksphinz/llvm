//===-- MYRISCVXSEISelDAGToDAG.h - A Dag to Dag Inst Selector for MYRISCVXSE -----===//
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
#pragma once

#include "MYRISCVXISelDAGToDAG.h"
namespace llvm {
  class MYRISCVXSEDAGToDAGISel : public MYRISCVXDAGToDAGISel {
 public:
    explicit MYRISCVXSEDAGToDAGISel(MYRISCVXTargetMachine &TM, CodeGenOpt::Level OL)
        : MYRISCVXDAGToDAGISel(TM, OL) {}
 private:
    bool runOnMachineFunction(MachineFunction &MF) override;

    void selectAddESubE(unsigned MOp, SDValue InFlag,
                        SDValue CmpLHS, const SDLoc &DL,
                        SDNode *Node) const;
    std::pair<SDNode *, SDNode *> selectMULT(SDNode *N, unsigned Opc,
                                             const SDLoc &DL, EVT Ty, bool HasLo,
                                             bool HasHi);

    bool trySelect(SDNode *Node) override;
    void processFunctionAfterISel(MachineFunction &MF) override;
    // Insert instructions to initialize the global base register in the
    // first MBB of the function.
    // void initGlobalBaseReg(MachineFunction &MF);
  };

  FunctionPass *createMYRISCVXSEISelDag(MYRISCVXTargetMachine &TM,
                                        CodeGenOpt::Level OptLevel);
}
