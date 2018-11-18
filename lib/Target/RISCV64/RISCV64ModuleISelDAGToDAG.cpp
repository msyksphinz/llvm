//===----------------------------------------------------------------------===//
// Instruction Selector Subtarget Control
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// This file defines a pass used to change the subtarget for the
// RISCV64 Instruction selector.
//
//===----------------------------------------------------------------------===//

#include "RISCV64.h"
#include "RISCV64TargetMachine.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/CodeGen/StackProtector.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "mips-isel"

namespace {
  class RISCV64ModuleDAGToDAGISel : public MachineFunctionPass {
  public:
    static char ID;

    RISCV64ModuleDAGToDAGISel() : MachineFunctionPass(ID) {}

    // Pass Name
    StringRef getPassName() const override {
      return "MIPS DAG->DAG Pattern Instruction Selection";
    }

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      AU.addRequired<TargetPassConfig>();
      AU.addPreserved<StackProtector>();
      MachineFunctionPass::getAnalysisUsage(AU);
    }

    bool runOnMachineFunction(MachineFunction &MF) override;
  };

  char RISCV64ModuleDAGToDAGISel::ID = 0;
}

bool RISCV64ModuleDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(errs() << "In RISCV64ModuleDAGToDAGISel::runMachineFunction\n");
  auto &TPC = getAnalysis<TargetPassConfig>();
  auto &TM = TPC.getTM<RISCV64TargetMachine>();
  TM.resetSubtarget(&MF);
  return false;
}

llvm::FunctionPass *llvm::createRISCV64ModuleISelDagPass() {
  return new RISCV64ModuleDAGToDAGISel();
}
