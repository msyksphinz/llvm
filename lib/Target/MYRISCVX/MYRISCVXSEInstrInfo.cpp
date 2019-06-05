//===-- MYRISCVXSEInstrInfo.cpp - MYRISCVX Instruction Information --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of the TargetInstrInfo class.
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

//@expandPostRAPseudo
/// Expand Pseudo instructions into real backend instructions
bool MYRISCVXSEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
//@expandPostRAPseudo-body
  MachineBasicBlock &MBB = *MI.getParent();

  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case MYRISCVX::RetRA:
    expandRetRA(MBB, MI);
    break;
  }

  MBB.erase(MI);
  return true;
}


/// Adjust SP by Amount bytes.
void MYRISCVXSEInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                         MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I) const {
  DebugLoc DL = I != MBB.end() ? I->getDebugLoc() : DebugLoc();
  unsigned ADD = MYRISCVX::ADD;
  unsigned ADDI = MYRISCVX::ADDI;

  MachineFunction *MF = MBB.getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();

  if (isInt<12>(Amount)) {
    // addiu sp, sp, amount
    BuildMI(MBB, I, DL, get(ADDI), SP).addReg(SP).addImm(Amount);
  } else { // Expand immediate that doesn't fit in 12-bit.
    unsigned Reg = MRI.createVirtualRegister(&MYRISCVX::GPRRegClass);
    loadImmediate(Amount, MBB, I, DL, Reg, nullptr);
    BuildMI(MBB, I, DL, get(ADD), SP).addReg(SP).addReg(Reg, RegState::Kill);
  }
}


/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
void
MYRISCVXSEInstrInfo::loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator II,
                                   const DebugLoc &DL, unsigned DstReg,
                                   unsigned *NewImm) const {
  // MYRISCVXAnalyzeImmediate AnalyzeImm;
  // unsigned Size = 32;
  // unsigned LUI = MYRISCVX::LUI;
  // unsigned ZEROReg = MYRISCVX::ZERO;
  // bool LastInstrIsADDI = NewImm;
  //
  // const MYRISCVXAnalyzeImmediate::InstSeq &Seq =
  //     AnalyzeImm.Analyze(Imm, Size, LastInstrIsADDI);
  // MYRISCVXAnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();
  //
  // assert(Seq.size() && (!LastInstrIsADDI || (Seq.size() > 1)));
  //
  // // The first instruction can be a LUI, which is different from other
  // // instructions (ADDI, ORI and SLL) in that it does not have a register
  // // operand.
  // if (Inst->Opc == LUI)
  //   BuildMI(MBB, II, DL, get(LUI), DstReg).addImm(SignExtend64<12>(Inst->ImmOpnd));
  // else
  //   BuildMI(MBB, II, DL, get(Inst->Opc), DstReg).addReg(ZEROReg)
  //       .addImm(SignExtend64<12>(Inst->ImmOpnd));
  //
  // // Build the remaining instructions in Seq.
  // for (++Inst; Inst != Seq.end() - LastInstrIsADDI; ++Inst)
  //   BuildMI(MBB, II, DL, get(Inst->Opc), DstReg).addReg(DstReg)
  //       .addImm(SignExtend64<12>(Inst->ImmOpnd));

  uint64_t Hi20 = ((Imm + 0x800) >> 12) & 0xfffff;
  uint64_t Lo12 = SignExtend64<12>(Imm);
  BuildMI(MBB, II, DL, get(MYRISCVX::LUI), DstReg)
      .addImm(Hi20);
  BuildMI(MBB, II, DL, get(MYRISCVX::ADDI), DstReg)
      .addReg(DstReg, RegState::Kill)
      .addImm(Lo12);
  //
  // if (LastInstrIsADDI)
  //   *NewImm = Inst->ImmOpnd;
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


void MYRISCVXSEInstrInfo::expandRetRA(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(MYRISCVX::RET)).addReg(MYRISCVX::RA);
}


const MYRISCVXInstrInfo *llvm::createMYRISCVXSEInstrInfo(const MYRISCVXSubtarget &STI) {
  return new MYRISCVXSEInstrInfo(STI);
}
