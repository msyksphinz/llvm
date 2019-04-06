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

const MYRISCVXInstrInfo *llvm::createMYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI) {
  return new MYRISCVXSEInstrInfo(STI);
}


//@expandPostRAPseudo
/// Expand Pseudo instructions into real backend instructions
bool MYRISCVXSEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  //@expandPostRAPseudo-body
  MachineBasicBlock &MBB = *MI.getParent();
  switch (MI.getDesc().getOpcode()) {
    default:
      return false;
    case MYRISCVX::RetRA:
      expandRetLR(MBB, MI);
      break;
    case MYRISCVX::MYRISCVXeh_return32:
      expandEhReturn(MBB, MI);
      break;
  }
  MBB.erase(MI);
  return true;
}


void MYRISCVXSEInstrInfo::expandRetLR(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(MYRISCVX::JALR)).addReg(MYRISCVX::RA);
}


void MYRISCVXSEInstrInfo::expandEhReturn(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  // This pseudo instruction is generated as part of the lowering of
  // ISD::EH_RETURN. We convert it to a stack increment by OffsetReg, and
  // indirect jump to TargetReg
  unsigned ADDU = MYRISCVX::ADD;
  unsigned SP = MYRISCVX::SP;
  unsigned LR = MYRISCVX::RA;
  unsigned T9 = MYRISCVX::T1;
  unsigned ZERO = MYRISCVX::ZERO;
  unsigned OffsetReg = I->getOperand(0).getReg();
  unsigned TargetReg = I->getOperand(1).getReg();

  // addu $lr, $v0, $zero
  // addu $sp, $sp, $v1
  // jr   $lr (via RetLR)
  const TargetMachine &TM = MBB.getParent()->getTarget();
  if (TM.isPositionIndependent())
    BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), T9)
        .addReg(TargetReg)
        .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), LR)
      .addReg(TargetReg)
      .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), SP).addReg(SP).addReg(OffsetReg);
  expandRetLR(MBB, I);
}


/// Adjust SP by Amount bytes.
void MYRISCVXSEInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                         MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I) const {
  DebugLoc DL = I != MBB.end() ? I->getDebugLoc() : DebugLoc();

  if (isInt<12>(Amount)) {
    // addiu sp, sp, amount
    BuildMI(MBB, I, DL, get(MYRISCVX::ADDI), SP).addReg(SP).addImm(Amount);
  }
  else { // Expand immediate that doesn't fit in 16-bit.
    // For numbers which are not 16bit integers we synthesize Amount inline
    // then add or subtract it from sp.
    unsigned Opc = MYRISCVX::ADD;
    if (Amount < 0) {
      Opc = MYRISCVX::SUB;
      Amount = -Amount;
    }
    unsigned Reg = loadImmediate(Amount, MBB, I, DL, nullptr);
    BuildMI(MBB, I, DL, get(Opc), SP).addReg(SP).addReg(Reg, RegState::Kill);
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
  unsigned LUI = MYRISCVX::LUI;
  unsigned ZEROReg = MYRISCVX::ZERO;
  bool LastInstrIsADDiu = NewImm;

  // MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();

  // The first instruction can be a LUi, which is different from other
  // instructions (ADDiu, ORI and SLL) in that it does not have a register
  // operand.
  // const TargetRegisterClass *RC = &MYRISCVX::GPRRegClass;

  // MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  // xxx: It's very temporal implementation
  unsigned Reg = MYRISCVX::S0;

  const MYRISCVXAnalyzeImmediate::InstSeq &Seq =
      AnalyzeImm.Analyze(Imm, Size, LastInstrIsADDiu);

  MYRISCVXAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();
  assert(Seq.size() && (!LastInstrIsADDiu || (Seq.size() > 1)));
  // The first instruction can be a LUi, which is different from other
  // instructions (ADDiu, ORI and SLL) in that it does not have a register
  // operand.
  if (Inst->Opc == LUI)
    BuildMI(MBB, II, DL, get(LUI), Reg).addImm(SignExtend64<16>(Inst->ImmOpnd));
  else
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(ZEROReg)
        .addImm(SignExtend64<16>(Inst->ImmOpnd));

  // Build the remaining instructions in Seq.
  for (++Inst; Inst != Seq.end() - LastInstrIsADDiu; ++Inst)
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(Reg)
        .addImm(SignExtend64<16>(Inst->ImmOpnd));

  if (LastInstrIsADDiu)
    *NewImm = Inst->ImmOpnd;

  return Reg;
}


void MYRISCVXSEInstrInfo::
storeRegToStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                unsigned SrcReg, bool isKill, int FI,
                const TargetRegisterClass *RC, const TargetRegisterInfo *TRI,
                int64_t Offset) const {
  DebugLoc DL;
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);
  unsigned Opc = 0;
  Opc = MYRISCVX::SW;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill))
      .addFrameIndex(FI).addImm(Offset).addMemOperand(MMO);
}


void MYRISCVXSEInstrInfo::
loadRegFromStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                 unsigned DestReg, int FI, const TargetRegisterClass *RC,
                 const TargetRegisterInfo *TRI, int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
  unsigned Opc = 0;
  Opc = MYRISCVX::LW;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc), DestReg).addFrameIndex(FI).addImm(Offset)
      .addMemOperand(MMO);
}


void MYRISCVXSEInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator MBBI,
                                      const DebugLoc &DL, unsigned DstReg,
                                      unsigned SrcReg, bool KillSrc) const {
  if (MYRISCVX::GPRRegClass.contains(DstReg, SrcReg)) {
    BuildMI(MBB, MBBI, DL, get(MYRISCVX::ADDI), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addImm(0);
    return;
  }
}


/// getOppositeBranchOpc - Return the inverse of the specified
/// opcode, e.g. turning BEQ to BNE.
unsigned MYRISCVXSEInstrInfo::getOppositeBranchOpc(unsigned Opc) const {
  switch (Opc) {
    default: llvm_unreachable("Illegal opcode!");
    case MYRISCVX::BEQ: return MYRISCVX::BNE;
    case MYRISCVX::BNE: return MYRISCVX::BEQ;
  }
}
