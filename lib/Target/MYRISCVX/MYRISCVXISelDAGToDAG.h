//===---- MYRISCVXISelDAGToDAG.h - A Dag to Dag Inst Selector for MYRISCVX --------===//
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

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXISELDAGTODAG_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXISELDAGTODAG_H

#include "MYRISCVX.h"
#include "MYRISCVXSubtarget.h"
#include "MYRISCVXTargetMachine.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"

//===----------------------------------------------------------------------===//
// Instruction Selector Implementation
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// MYRISCVXDAGToDAGISel - MYRISCVX specific code to select MYRISCVX machine
// instructions for SelectionDAG operations.
//===----------------------------------------------------------------------===//
namespace llvm {

  class MYRISCVXDAGToDAGISel : public SelectionDAGISel {
 public:
    explicit MYRISCVXDAGToDAGISel(MYRISCVXTargetMachine &TM, CodeGenOpt::Level OL)
        : SelectionDAGISel(TM, OL), Subtarget(nullptr) {}

    // Pass Name
    StringRef getPassName() const override {
      return "MYRISCVX DAG->DAG Pattern Instruction Selection";
    }

    bool runOnMachineFunction(MachineFunction &MF) override;

    SDNode *getGlobalBaseReg();

 protected:

    /// Keep a pointer to the MYRISCVXSubtarget around so that we can make the right
    /// decision when generating code for different targets.
    const MYRISCVXSubtarget *Subtarget;

 private:
    // Include the pieces autogenerated from the target description.
#include "MYRISCVXGenDAGISel.inc"

    /// getTargetMachine - Return a reference to the TargetMachine, casted
    /// to the target-specific type.
    const MYRISCVXTargetMachine &getTargetMachine() {
      return static_cast<const MYRISCVXTargetMachine &>(TM);
    }

    void Select(SDNode *N) override;

    virtual bool trySelect(SDNode *Node) = 0;

    // Complex Pattern.
    bool SelectAddr(SDValue N, SDValue &Base, SDValue &Offset);
    bool SelectAddrFI(SDValue Addr, SDValue &Base);

    // getImm - Return a target constant with the specified value.
    inline SDValue getImm(const SDNode *Node, unsigned Imm) {
      return CurDAG->getTargetConstant(Imm, SDLoc(Node), Node->getValueType(0));
    }

    bool SelectInlineAsmMemoryOperand(const SDValue &Op,
                                      unsigned ConstraintID,
                                      std::vector<SDValue> &OutOps) override;

    virtual void processFunctionAfterISel(MachineFunction &MF) = 0;

  };

}

#endif
