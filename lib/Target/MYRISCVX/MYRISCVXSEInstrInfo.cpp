//===-- MYRISCVXSEInstrInfo.cpp - MYRISCVX32/64 Instruction Information -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX32/64 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXSEInstrInfo.h"

#include "MYRISCVXMachineFunction.h"
#include "MYRISCVXTargetMachine.h"
#include "MYRISCVXAnalyzeImmediate.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

MYRISCVXSEInstrInfo::MYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI)
    : MYRISCVXInstrInfo(STI),
      RI(STI) {}

const MYRISCVXRegisterInfo &MYRISCVXSEInstrInfo::getRegisterInfo() const {
  return RI;
}

//@expandPostRAPseudo
/// Expand Pseudo instructions into real backend instructions
bool MYRISCVXSEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
//@expandPostRAPseudo-body
  MachineBasicBlock &MBB = *MI.getParent();

  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case MYRISCVX::RetLR:
    expandRetLR(MBB, MI);
    break;
  }

  MBB.erase(MI);
  return true;
}

void MYRISCVXSEInstrInfo::expandRetLR(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(MYRISCVX::RET)).addReg(MYRISCVX::RA);
}


const MYRISCVXInstrInfo *llvm::createMYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI) {
  return new MYRISCVXSEInstrInfo(STI);
}


/// Adjust SP by Amount bytes.
void MYRISCVXSEInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                         MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I) const {
  DebugLoc DL = I != MBB.end() ? I->getDebugLoc() : DebugLoc();
  unsigned ADDu = MYRISCVX::ADDu;
  unsigned ADDiu = MYRISCVX::ADDiu;

  if (isInt<16>(Amount)) {
    // addiu sp, sp, amount
    BuildMI(MBB, I, DL, get(ADDiu), SP).addReg(SP).addImm(Amount);
  } else { // Expand immediate that doesn't fit in 16-bit.
    unsigned Reg = loadImmediate(Amount, MBB, I, DL, nullptr);
    BuildMI(MBB, I, DL, get(ADDu), SP).addReg(SP).addReg(Reg, RegState::Kill);
  }
}

/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
unsigned
MYRISCVXSEInstrInfo::loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator II,
                                   const DebugLoc &DL,
                                   unsigned *NewImm) const {
  MYRISCVXAnalyzeImmediate AnalyzeImm;
  unsigned Size = 32;
  unsigned LUi = MYRISCVX::LUi;
  unsigned ZEROReg = MYRISCVX::ZERO;
  unsigned ATReg = MYRISCVX::AT;
  bool LastInstrIsADDiu = NewImm;

  const MYRISCVXAnalyzeImmediate::InstSeq &Seq =
    AnalyzeImm.Analyze(Imm, Size, LastInstrIsADDiu);
  MYRISCVXAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();

  assert(Seq.size() && (!LastInstrIsADDiu || (Seq.size() > 1)));

  // The first instruction can be a LUi, which is different from other
  // instructions (ADDiu, ORI and SLL) in that it does not have a register
  // operand.
  if (Inst->Opc == LUi)
    BuildMI(MBB, II, DL, get(LUi), ATReg).addImm(SignExtend64<16>(Inst->ImmOpnd));
  else
    BuildMI(MBB, II, DL, get(Inst->Opc), ATReg).addReg(ZEROReg)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  // Build the remaining instructions in Seq.
  for (++Inst; Inst != Seq.end() - LastInstrIsADDiu; ++Inst)
    BuildMI(MBB, II, DL, get(Inst->Opc), ATReg).addReg(ATReg)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  if (LastInstrIsADDiu)
    *NewImm = Inst->ImmOpnd;

  return ATReg;
}
