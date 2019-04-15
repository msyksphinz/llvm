//===-- MYRISCVXSEISelLowering.cpp - MYRISCVXSE DAG Lowering Interface --*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of MYRISCVXTargetLowering specialized for MYRISCVX32.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXMachineFunction.h"
#include "MYRISCVXSEISelLowering.h"
#include "MYRISCVXRegisterInfo.h"
#include "MYRISCVXTargetMachine.h"

#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

using namespace llvm;
#define DEBUG_TYPE "myriscvx-isel"

static cl::opt<bool>
EnableMYRISCVXTailCalls("enable-myriscvx-tail-calls", cl::Hidden,
                        cl::desc("MYRISCVX: Enable tail calls."), cl::init(false));

//@MYRISCVXSETargetLowering {
MYRISCVXSETargetLowering::MYRISCVXSETargetLowering(const MYRISCVXTargetMachine &TM,
                                                   const MYRISCVXSubtarget &STI)
    : MYRISCVXTargetLowering(TM, STI) {
  //@MYRISCVXSETargetLowering body {
  // Set up the register classes
  addRegisterClass(MVT::i32, &MYRISCVX::GPRRegClass);

  setOperationAction(ISD::ATOMIC_FENCE,       MVT::Other, Custom);

  // must, computeRegisterProperties - Once all of the register classes are
  // added, this allows us to compute derived properties we expose.
  computeRegisterProperties(Subtarget.getRegisterInfo());
}

SDValue MYRISCVXSETargetLowering::LowerOperation(SDValue Op,
                                                 SelectionDAG &DAG) const {
  return MYRISCVXTargetLowering::LowerOperation(Op, DAG);
}

const MYRISCVXTargetLowering *
llvm::createMYRISCVXSETargetLowering(const MYRISCVXTargetMachine &TM,
                                     const MYRISCVXSubtarget &STI) {
  return new MYRISCVXSETargetLowering(TM, STI);
}


bool MYRISCVXSETargetLowering::
isEligibleForTailCallOptimization(const MYRISCVXCC &MYRISCVXCCInfo,
                                  unsigned NextStackOffset,
                                  const MYRISCVXFunctionInfo& FI) const {
  if (!EnableMYRISCVXTailCalls)
    return false;

  // Return false if either the callee or caller has a byval argument.
  if (MYRISCVXCCInfo.hasByValArg() || FI.hasByvalArg())
    return false;

  // Return true if the callee's argument area is no larger than the
  // caller's.
  return NextStackOffset <= FI.getIncomingArgSize();
}
