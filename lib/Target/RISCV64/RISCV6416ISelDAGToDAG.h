//===---- RISCV6416ISelDAGToDAG.h - A Dag to Dag Inst Selector for RISCV64 ------===//
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

#ifndef LLVM_LIB_TARGET_MIPS_MIPS16ISELDAGTODAG_H
#define LLVM_LIB_TARGET_MIPS_MIPS16ISELDAGTODAG_H

#include "RISCV64ISelDAGToDAG.h"

namespace llvm {

class RISCV6416DAGToDAGISel : public RISCV64DAGToDAGISel {
public:
  explicit RISCV6416DAGToDAGISel(RISCV64TargetMachine &TM, CodeGenOpt::Level OL)
      : RISCV64DAGToDAGISel(TM, OL) {}

private:
  std::pair<SDNode *, SDNode *> selectMULT(SDNode *N, unsigned Opc,
                                           const SDLoc &DL, EVT Ty, bool HasLo,
                                           bool HasHi);

  bool runOnMachineFunction(MachineFunction &MF) override;

  bool selectAddr(bool SPAllowed, SDValue Addr, SDValue &Base,
                  SDValue &Offset);
  bool selectAddr16(SDValue Addr, SDValue &Base,
                    SDValue &Offset) override;
  bool selectAddr16SP(SDValue Addr, SDValue &Base,
                      SDValue &Offset) override;

  bool trySelect(SDNode *Node) override;

  void processFunctionAfterISel(MachineFunction &MF) override;

  // Insert instructions to initialize the global base register in the
  // first MBB of the function.
  void initGlobalBaseReg(MachineFunction &MF);

  void initRISCV6416SPAliasReg(MachineFunction &MF);
};

FunctionPass *createRISCV6416ISelDag(RISCV64TargetMachine &TM,
                                  CodeGenOpt::Level OptLevel);
}

#endif
