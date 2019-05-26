//===-- MYRISCVXSEISelDAGToDAG.h - A Dag to Dag Inst Selector for MYRISCVXSE -----===//
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

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSEISELDAGTODAG_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXSEISELDAGTODAG_H

#include "MYRISCVXISelDAGToDAG.h"

namespace llvm {

  class MYRISCVXSEDAGToDAGISel : public MYRISCVXDAGToDAGISel {

 public:
    explicit MYRISCVXSEDAGToDAGISel(MYRISCVXTargetMachine &TM, CodeGenOpt::Level OL)
        : MYRISCVXDAGToDAGISel(TM, OL) {}

 private:

    bool runOnMachineFunction(MachineFunction &MF) override;

    bool trySelect(SDNode *Node) override;

    void processFunctionAfterISel(MachineFunction &MF) override;

    // Insert instructions to initialize the global base register in the
    // first MBB of the function.
    //  void initGlobalBaseReg(MachineFunction &MF);

  };

  FunctionPass *createMYRISCVXSEISelDag(MYRISCVXTargetMachine &TM,
                                        CodeGenOpt::Level OptLevel);

}

#endif
