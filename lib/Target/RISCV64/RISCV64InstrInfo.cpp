//===- RISCV64InstrInfo.cpp - RISCV64 Instruction Information -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RISCV64 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "RISCV64InstrInfo.h"
#include "MCTargetDesc/RISCV64BaseInfo.h"
#include "MCTargetDesc/RISCV64MCTargetDesc.h"
#include "RISCV64Subtarget.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Target/TargetMachine.h"
#include <cassert>

using namespace llvm;

#define GET_INSTRINFO_CTOR_DTOR
#include "RISCV64GenInstrInfo.inc"

// Pin the vtable to this file.
void RISCV64InstrInfo::anchor() {}

RISCV64InstrInfo::RISCV64InstrInfo(const RISCV64Subtarget &STI, unsigned UncondBr)
    : RISCV64GenInstrInfo(RISCV64::ADJCALLSTACKDOWN, RISCV64::ADJCALLSTACKUP),
      Subtarget(STI), UncondBrOpc(UncondBr) {}

const RISCV64InstrInfo *RISCV64InstrInfo::create(RISCV64Subtarget &STI) {
  if (STI.inRISCV6416Mode())
    return createRISCV6416InstrInfo(STI);

  return createRISCV64SEInstrInfo(STI);
}

bool RISCV64InstrInfo::isZeroImm(const MachineOperand &op) const {
  return op.isImm() && op.getImm() == 0;
}

/// insertNoop - If data hazard condition is found insert the target nop
/// instruction.
// FIXME: This appears to be dead code.
void RISCV64InstrInfo::
insertNoop(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI) const
{
  DebugLoc DL;
  BuildMI(MBB, MI, DL, get(RISCV64::NOP));
}

MachineMemOperand *
RISCV64InstrInfo::GetMemOperand(MachineBasicBlock &MBB, int FI,
                             MachineMemOperand::Flags Flags) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned Align = MFI.getObjectAlignment(FI);

  return MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(MF, FI),
                                 Flags, MFI.getObjectSize(FI), Align);
}

//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//

void RISCV64InstrInfo::AnalyzeCondBr(const MachineInstr *Inst, unsigned Opc,
                                  MachineBasicBlock *&BB,
                                  SmallVectorImpl<MachineOperand> &Cond) const {
  assert(getAnalyzableBrOpc(Opc) && "Not an analyzable branch");
  int NumOp = Inst->getNumExplicitOperands();

  // for both int and fp branches, the last explicit operand is the
  // MBB.
  BB = Inst->getOperand(NumOp-1).getMBB();
  Cond.push_back(MachineOperand::CreateImm(Opc));

  for (int i = 0; i < NumOp-1; i++)
    Cond.push_back(Inst->getOperand(i));
}

bool RISCV64InstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                  MachineBasicBlock *&TBB,
                                  MachineBasicBlock *&FBB,
                                  SmallVectorImpl<MachineOperand> &Cond,
                                  bool AllowModify) const {
  SmallVector<MachineInstr*, 2> BranchInstrs;
  BranchType BT = analyzeBranch(MBB, TBB, FBB, Cond, AllowModify, BranchInstrs);

  return (BT == BT_None) || (BT == BT_Indirect);
}

void RISCV64InstrInfo::BuildCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                                const DebugLoc &DL,
                                ArrayRef<MachineOperand> Cond) const {
  unsigned Opc = Cond[0].getImm();
  const MCInstrDesc &MCID = get(Opc);
  MachineInstrBuilder MIB = BuildMI(&MBB, DL, MCID);

  for (unsigned i = 1; i < Cond.size(); ++i) {
    assert((Cond[i].isImm() || Cond[i].isReg()) &&
           "Cannot copy operand for conditional branch!");
    MIB.add(Cond[i]);
  }
  MIB.addMBB(TBB);
}

unsigned RISCV64InstrInfo::insertBranch(MachineBasicBlock &MBB,
                                     MachineBasicBlock *TBB,
                                     MachineBasicBlock *FBB,
                                     ArrayRef<MachineOperand> Cond,
                                     const DebugLoc &DL,
                                     int *BytesAdded) const {
  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert(!BytesAdded && "code size not handled");

  // # of condition operands:
  //  Unconditional branches: 0
  //  Floating point branches: 1 (opc)
  //  Int BranchZero: 2 (opc, reg)
  //  Int Branch: 3 (opc, reg0, reg1)
  assert((Cond.size() <= 3) &&
         "# of RISCV64 branch conditions must be <= 3!");

  // Two-way Conditional branch.
  if (FBB) {
    BuildCondBr(MBB, TBB, DL, Cond);
    BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(FBB);
    return 2;
  }

  // One way branch.
  // Unconditional branch.
  if (Cond.empty())
    BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(TBB);
  else // Conditional branch.
    BuildCondBr(MBB, TBB, DL, Cond);
  return 1;
}

unsigned RISCV64InstrInfo::removeBranch(MachineBasicBlock &MBB,
                                     int *BytesRemoved) const {
  assert(!BytesRemoved && "code size not handled");

  MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();
  unsigned removed = 0;

  // Up to 2 branches are removed.
  // Note that indirect branches are not removed.
  while (I != REnd && removed < 2) {
    // Skip past debug instructions.
    if (I->isDebugInstr()) {
      ++I;
      continue;
    }
    if (!getAnalyzableBrOpc(I->getOpcode()))
      break;
    // Remove the branch.
    I->eraseFromParent();
    I = MBB.rbegin();
    ++removed;
  }

  return removed;
}

/// reverseBranchCondition - Return the inverse opcode of the
/// specified Branch instruction.
bool RISCV64InstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert( (Cond.size() && Cond.size() <= 3) &&
          "Invalid RISCV64 branch condition!");
  Cond[0].setImm(getOppositeBranchOpc(Cond[0].getImm()));
  return false;
}

RISCV64InstrInfo::BranchType RISCV64InstrInfo::analyzeBranch(
    MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB,
    SmallVectorImpl<MachineOperand> &Cond, bool AllowModify,
    SmallVectorImpl<MachineInstr *> &BranchInstrs) const {
  MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();

  // Skip all the debug instructions.
  while (I != REnd && I->isDebugInstr())
    ++I;

  if (I == REnd || !isUnpredicatedTerminator(*I)) {
    // This block ends with no branches (it just falls through to its succ).
    // Leave TBB/FBB null.
    TBB = FBB = nullptr;
    return BT_NoBranch;
  }

  MachineInstr *LastInst = &*I;
  unsigned LastOpc = LastInst->getOpcode();
  BranchInstrs.push_back(LastInst);

  // Not an analyzable branch (e.g., indirect jump).
  if (!getAnalyzableBrOpc(LastOpc))
    return LastInst->isIndirectBranch() ? BT_Indirect : BT_None;

  // Get the second to last instruction in the block.
  unsigned SecondLastOpc = 0;
  MachineInstr *SecondLastInst = nullptr;

  // Skip past any debug instruction to see if the second last actual
  // is a branch.
  ++I;
  while (I != REnd && I->isDebugInstr())
    ++I;

  if (I != REnd) {
    SecondLastInst = &*I;
    SecondLastOpc = getAnalyzableBrOpc(SecondLastInst->getOpcode());

    // Not an analyzable branch (must be an indirect jump).
    if (isUnpredicatedTerminator(*SecondLastInst) && !SecondLastOpc)
      return BT_None;
  }

  // If there is only one terminator instruction, process it.
  if (!SecondLastOpc) {
    // Unconditional branch.
    if (LastInst->isUnconditionalBranch()) {
      TBB = LastInst->getOperand(0).getMBB();
      return BT_Uncond;
    }

    // Conditional branch
    AnalyzeCondBr(LastInst, LastOpc, TBB, Cond);
    return BT_Cond;
  }

  // If we reached here, there are two branches.
  // If there are three terminators, we don't know what sort of block this is.
  if (++I != REnd && isUnpredicatedTerminator(*I))
    return BT_None;

  BranchInstrs.insert(BranchInstrs.begin(), SecondLastInst);

  // If second to last instruction is an unconditional branch,
  // analyze it and remove the last instruction.
  if (SecondLastInst->isUnconditionalBranch()) {
    // Return if the last instruction cannot be removed.
    if (!AllowModify)
      return BT_None;

    TBB = SecondLastInst->getOperand(0).getMBB();
    LastInst->eraseFromParent();
    BranchInstrs.pop_back();
    return BT_Uncond;
  }

  // Conditional branch followed by an unconditional branch.
  // The last one must be unconditional.
  if (!LastInst->isUnconditionalBranch())
    return BT_None;

  AnalyzeCondBr(SecondLastInst, SecondLastOpc, TBB, Cond);
  FBB = LastInst->getOperand(0).getMBB();

  return BT_CondUncond;
}

bool RISCV64InstrInfo::isBranchOffsetInRange(unsigned BranchOpc, int64_t BrOffset) const {
  switch (BranchOpc) {
  case RISCV64::B:
  case RISCV64::BAL:
  case RISCV64::BAL_BR:
  case RISCV64::BAL_BR_MM:
  case RISCV64::BC1F:
  case RISCV64::BC1FL:
  case RISCV64::BC1T:
  case RISCV64::BC1TL:
  case RISCV64::BEQ:     case RISCV64::BEQ64:
  case RISCV64::BEQL:
  case RISCV64::BGEZ:    case RISCV64::BGEZ64:
  case RISCV64::BGEZL:
  case RISCV64::BGEZAL:
  case RISCV64::BGEZALL:
  case RISCV64::BGTZ:    case RISCV64::BGTZ64:
  case RISCV64::BGTZL:
  case RISCV64::BLEZ:    case RISCV64::BLEZ64:
  case RISCV64::BLEZL:
  case RISCV64::BLTZ:    case RISCV64::BLTZ64:
  case RISCV64::BLTZL:
  case RISCV64::BLTZAL:
  case RISCV64::BLTZALL:
  case RISCV64::BNE:     case RISCV64::BNE64:
  case RISCV64::BNEL:
    return isInt<18>(BrOffset);

  // microMIPSr3 branches
  case RISCV64::B_MM:
  case RISCV64::BC1F_MM:
  case RISCV64::BC1T_MM:
  case RISCV64::BEQ_MM:
  case RISCV64::BGEZ_MM:
  case RISCV64::BGEZAL_MM:
  case RISCV64::BGTZ_MM:
  case RISCV64::BLEZ_MM:
  case RISCV64::BLTZ_MM:
  case RISCV64::BLTZAL_MM:
  case RISCV64::BNE_MM:
  case RISCV64::BEQZC_MM:
  case RISCV64::BNEZC_MM:
    return isInt<17>(BrOffset);

  // microMIPSR3 short branches.
  case RISCV64::B16_MM:
    return isInt<11>(BrOffset);

  case RISCV64::BEQZ16_MM:
  case RISCV64::BNEZ16_MM:
    return isInt<8>(BrOffset);

  // MIPSR6 branches.
  case RISCV64::BALC:
  case RISCV64::BC:
    return isInt<28>(BrOffset);

  case RISCV64::BC1EQZ:
  case RISCV64::BC1NEZ:
  case RISCV64::BC2EQZ:
  case RISCV64::BC2NEZ:
  case RISCV64::BEQC:   case RISCV64::BEQC64:
  case RISCV64::BNEC:   case RISCV64::BNEC64:
  case RISCV64::BGEC:   case RISCV64::BGEC64:
  case RISCV64::BGEUC:  case RISCV64::BGEUC64:
  case RISCV64::BGEZC:  case RISCV64::BGEZC64:
  case RISCV64::BGTZC:  case RISCV64::BGTZC64:
  case RISCV64::BLEZC:  case RISCV64::BLEZC64:
  case RISCV64::BLTC:   case RISCV64::BLTC64:
  case RISCV64::BLTUC:  case RISCV64::BLTUC64:
  case RISCV64::BLTZC:  case RISCV64::BLTZC64:
  case RISCV64::BNVC:
  case RISCV64::BOVC:
  case RISCV64::BGEZALC:
  case RISCV64::BEQZALC:
  case RISCV64::BGTZALC:
  case RISCV64::BLEZALC:
  case RISCV64::BLTZALC:
  case RISCV64::BNEZALC:
    return isInt<18>(BrOffset);

  case RISCV64::BEQZC:  case RISCV64::BEQZC64:
  case RISCV64::BNEZC:  case RISCV64::BNEZC64:
    return isInt<23>(BrOffset);

  // microMIPSR6 branches
  case RISCV64::BC16_MMR6:
    return isInt<11>(BrOffset);

  case RISCV64::BEQZC16_MMR6:
  case RISCV64::BNEZC16_MMR6:
    return isInt<8>(BrOffset);

  case RISCV64::BALC_MMR6:
  case RISCV64::BC_MMR6:
    return isInt<27>(BrOffset);

  case RISCV64::BC1EQZC_MMR6:
  case RISCV64::BC1NEZC_MMR6:
  case RISCV64::BC2EQZC_MMR6:
  case RISCV64::BC2NEZC_MMR6:
  case RISCV64::BGEZALC_MMR6:
  case RISCV64::BEQZALC_MMR6:
  case RISCV64::BGTZALC_MMR6:
  case RISCV64::BLEZALC_MMR6:
  case RISCV64::BLTZALC_MMR6:
  case RISCV64::BNEZALC_MMR6:
  case RISCV64::BNVC_MMR6:
  case RISCV64::BOVC_MMR6:
    return isInt<17>(BrOffset);

  case RISCV64::BEQC_MMR6:
  case RISCV64::BNEC_MMR6:
  case RISCV64::BGEC_MMR6:
  case RISCV64::BGEUC_MMR6:
  case RISCV64::BGEZC_MMR6:
  case RISCV64::BGTZC_MMR6:
  case RISCV64::BLEZC_MMR6:
  case RISCV64::BLTC_MMR6:
  case RISCV64::BLTUC_MMR6:
  case RISCV64::BLTZC_MMR6:
    return isInt<18>(BrOffset);

  case RISCV64::BEQZC_MMR6:
  case RISCV64::BNEZC_MMR6:
    return isInt<23>(BrOffset);

  // DSP branches.
  case RISCV64::BPOSGE32:
    return isInt<18>(BrOffset);
  case RISCV64::BPOSGE32_MM:
  case RISCV64::BPOSGE32C_MMR3:
    return isInt<17>(BrOffset);

  // cnMIPS branches.
  case RISCV64::BBIT0:
  case RISCV64::BBIT032:
  case RISCV64::BBIT1:
  case RISCV64::BBIT132:
    return isInt<18>(BrOffset);

  // MSA branches.
  case RISCV64::BZ_B:
  case RISCV64::BZ_H:
  case RISCV64::BZ_W:
  case RISCV64::BZ_D:
  case RISCV64::BZ_V:
  case RISCV64::BNZ_B:
  case RISCV64::BNZ_H:
  case RISCV64::BNZ_W:
  case RISCV64::BNZ_D:
  case RISCV64::BNZ_V:
    return isInt<18>(BrOffset);
  }

  llvm_unreachable("Unknown branch instruction!");
}


/// Return the corresponding compact (no delay slot) form of a branch.
unsigned RISCV64InstrInfo::getEquivalentCompactForm(
    const MachineBasicBlock::iterator I) const {
  unsigned Opcode = I->getOpcode();
  bool canUseShortMicroRISCV64CTI = false;

  if (Subtarget.inMicroRISCV64Mode()) {
    switch (Opcode) {
    case RISCV64::BNE:
    case RISCV64::BNE_MM:
    case RISCV64::BEQ:
    case RISCV64::BEQ_MM:
    // microMIPS has NE,EQ branches that do not have delay slots provided one
    // of the operands is zero.
      if (I->getOperand(1).getReg() == Subtarget.getABI().GetZeroReg())
        canUseShortMicroRISCV64CTI = true;
      break;
    // For microMIPS the PseudoReturn and PseudoIndirectBranch are always
    // expanded to JR_MM, so they can be replaced with JRC16_MM.
    case RISCV64::JR:
    case RISCV64::PseudoReturn:
    case RISCV64::PseudoIndirectBranch:
      canUseShortMicroRISCV64CTI = true;
      break;
    }
  }

  // MIPSR6 forbids both operands being the zero register.
  if (Subtarget.hasRISCV6432r6() && (I->getNumOperands() > 1) &&
      (I->getOperand(0).isReg() &&
       (I->getOperand(0).getReg() == RISCV64::ZERO ||
        I->getOperand(0).getReg() == RISCV64::ZERO_64)) &&
      (I->getOperand(1).isReg() &&
       (I->getOperand(1).getReg() == RISCV64::ZERO ||
        I->getOperand(1).getReg() == RISCV64::ZERO_64)))
    return 0;

  if (Subtarget.hasRISCV6432r6() || canUseShortMicroRISCV64CTI) {
    switch (Opcode) {
    case RISCV64::B:
      return RISCV64::BC;
    case RISCV64::BAL:
      return RISCV64::BALC;
    case RISCV64::BEQ:
    case RISCV64::BEQ_MM:
      if (canUseShortMicroRISCV64CTI)
        return RISCV64::BEQZC_MM;
      else if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return RISCV64::BEQC;
    case RISCV64::BNE:
    case RISCV64::BNE_MM:
      if (canUseShortMicroRISCV64CTI)
        return RISCV64::BNEZC_MM;
      else if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return RISCV64::BNEC;
    case RISCV64::BGE:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return RISCV64::BGEC;
    case RISCV64::BGEU:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return RISCV64::BGEUC;
    case RISCV64::BGEZ:
      return RISCV64::BGEZC;
    case RISCV64::BGTZ:
      return RISCV64::BGTZC;
    case RISCV64::BLEZ:
      return RISCV64::BLEZC;
    case RISCV64::BLT:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return RISCV64::BLTC;
    case RISCV64::BLTU:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return RISCV64::BLTUC;
    case RISCV64::BLTZ:
      return RISCV64::BLTZC;
    case RISCV64::BEQ64:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return RISCV64::BEQC64;
    case RISCV64::BNE64:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return RISCV64::BNEC64;
    case RISCV64::BGTZ64:
      return RISCV64::BGTZC64;
    case RISCV64::BGEZ64:
      return RISCV64::BGEZC64;
    case RISCV64::BLTZ64:
      return RISCV64::BLTZC64;
    case RISCV64::BLEZ64:
      return RISCV64::BLEZC64;
    // For MIPSR6, the instruction 'jic' can be used for these cases. Some
    // tools will accept 'jrc reg' as an alias for 'jic 0, $reg'.
    case RISCV64::JR:
    case RISCV64::PseudoIndirectBranchR6:
    case RISCV64::PseudoReturn:
    case RISCV64::TAILCALLR6REG:
      if (canUseShortMicroRISCV64CTI)
        return RISCV64::JRC16_MM;
      return RISCV64::JIC;
    case RISCV64::JALRPseudo:
      return RISCV64::JIALC;
    case RISCV64::JR64:
    case RISCV64::PseudoIndirectBranch64R6:
    case RISCV64::PseudoReturn64:
    case RISCV64::TAILCALL64R6REG:
      return RISCV64::JIC64;
    case RISCV64::JALR64Pseudo:
      return RISCV64::JIALC64;
    default:
      return 0;
    }
  }

  return 0;
}

/// Predicate for distingushing between control transfer instructions and all
/// other instructions for handling forbidden slots. Consider inline assembly
/// as unsafe as well.
bool RISCV64InstrInfo::SafeInForbiddenSlot(const MachineInstr &MI) const {
  if (MI.isInlineAsm())
    return false;

  return (MI.getDesc().TSFlags & RISCV64II::IsCTI) == 0;
}

/// Predicate for distingushing instructions that have forbidden slots.
bool RISCV64InstrInfo::HasForbiddenSlot(const MachineInstr &MI) const {
  return (MI.getDesc().TSFlags & RISCV64II::HasForbiddenSlot) != 0;
}

/// Return the number of bytes of code the specified instruction may be.
unsigned RISCV64InstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    return MI.getDesc().getSize();
  case  TargetOpcode::INLINEASM: {       // Inline Asm: Variable size.
    const MachineFunction *MF = MI.getParent()->getParent();
    const char *AsmStr = MI.getOperand(0).getSymbolName();
    return getInlineAsmLength(AsmStr, *MF->getTarget().getMCAsmInfo());
  }
  case RISCV64::CONSTPOOL_ENTRY:
    // If this machine instr is a constant pool entry, its size is recorded as
    // operand #2.
    return MI.getOperand(2).getImm();
  }
}

MachineInstrBuilder
RISCV64InstrInfo::genInstrWithNewOpc(unsigned NewOpc,
                                  MachineBasicBlock::iterator I) const {
  MachineInstrBuilder MIB;

  // Certain branches have two forms: e.g beq $1, $zero, dest vs beqz $1, dest
  // Pick the zero form of the branch for readable assembly and for greater
  // branch distance in non-microMIPS mode.
  // Additional MIPSR6 does not permit the use of register $zero for compact
  // branches.
  // FIXME: Certain atomic sequences on mips64 generate 32bit references to
  // RISCV64::ZERO, which is incorrect. This test should be updated to use
  // Subtarget.getABI().GetZeroReg() when those atomic sequences and others
  // are fixed.
  int ZeroOperandPosition = -1;
  bool BranchWithZeroOperand = false;
  if (I->isBranch() && !I->isPseudo()) {
    auto TRI = I->getParent()->getParent()->getSubtarget().getRegisterInfo();
    ZeroOperandPosition = I->findRegisterUseOperandIdx(RISCV64::ZERO, false, TRI);
    BranchWithZeroOperand = ZeroOperandPosition != -1;
  }

  if (BranchWithZeroOperand) {
    switch (NewOpc) {
    case RISCV64::BEQC:
      NewOpc = RISCV64::BEQZC;
      break;
    case RISCV64::BNEC:
      NewOpc = RISCV64::BNEZC;
      break;
    case RISCV64::BGEC:
      NewOpc = RISCV64::BGEZC;
      break;
    case RISCV64::BLTC:
      NewOpc = RISCV64::BLTZC;
      break;
    case RISCV64::BEQC64:
      NewOpc = RISCV64::BEQZC64;
      break;
    case RISCV64::BNEC64:
      NewOpc = RISCV64::BNEZC64;
      break;
    }
  }

  MIB = BuildMI(*I->getParent(), I, I->getDebugLoc(), get(NewOpc));

  // For MIPSR6 JI*C requires an immediate 0 as an operand, JIALC(64) an
  // immediate 0 as an operand and requires the removal of it's implicit-def %ra
  // implicit operand as copying the implicit operations of the instructio we're
  // looking at will give us the correct flags.
  if (NewOpc == RISCV64::JIC || NewOpc == RISCV64::JIALC || NewOpc == RISCV64::JIC64 ||
      NewOpc == RISCV64::JIALC64) {

    if (NewOpc == RISCV64::JIALC || NewOpc == RISCV64::JIALC64)
      MIB->RemoveOperand(0);

    for (unsigned J = 0, E = I->getDesc().getNumOperands(); J < E; ++J) {
      MIB.add(I->getOperand(J));
    }

    MIB.addImm(0);

  } else {
    for (unsigned J = 0, E = I->getDesc().getNumOperands(); J < E; ++J) {
      if (BranchWithZeroOperand && (unsigned)ZeroOperandPosition == J)
        continue;

      MIB.add(I->getOperand(J));
    }
  }

  MIB.copyImplicitOps(*I);
  MIB.cloneMemRefs(*I);
  return MIB;
}

bool RISCV64InstrInfo::findCommutedOpIndices(MachineInstr &MI, unsigned &SrcOpIdx1,
                                          unsigned &SrcOpIdx2) const {
  assert(!MI.isBundle() &&
         "TargetInstrInfo::findCommutedOpIndices() can't handle bundles");

  const MCInstrDesc &MCID = MI.getDesc();
  if (!MCID.isCommutable())
    return false;

  switch (MI.getOpcode()) {
  case RISCV64::DPADD_U_H:
  case RISCV64::DPADD_U_W:
  case RISCV64::DPADD_U_D:
  case RISCV64::DPADD_S_H:
  case RISCV64::DPADD_S_W:
  case RISCV64::DPADD_S_D:
    // The first operand is both input and output, so it should not commute
    if (!fixCommutedOpIndices(SrcOpIdx1, SrcOpIdx2, 2, 3))
      return false;

    if (!MI.getOperand(SrcOpIdx1).isReg() || !MI.getOperand(SrcOpIdx2).isReg())
      return false;
    return true;
  }
  return TargetInstrInfo::findCommutedOpIndices(MI, SrcOpIdx1, SrcOpIdx2);
}

// ins, ext, dext*, dins have the following constraints:
// X <= pos      <  Y
// X <  size     <= Y
// X <  pos+size <= Y
//
// dinsm and dinsu have the following constraints:
// X <= pos      <  Y
// X <= size     <= Y
// X <  pos+size <= Y
//
// The callee of verifyInsExtInstruction however gives the bounds of
// dins[um] like the other (d)ins (d)ext(um) instructions, so that this
// function doesn't have to vary it's behaviour based on the instruction
// being checked.
static bool verifyInsExtInstruction(const MachineInstr &MI, StringRef &ErrInfo,
                                    const int64_t PosLow, const int64_t PosHigh,
                                    const int64_t SizeLow,
                                    const int64_t SizeHigh,
                                    const int64_t BothLow,
                                    const int64_t BothHigh) {
  MachineOperand MOPos = MI.getOperand(2);
  if (!MOPos.isImm()) {
    ErrInfo = "Position is not an immediate!";
    return false;
  }
  int64_t Pos = MOPos.getImm();
  if (!((PosLow <= Pos) && (Pos < PosHigh))) {
    ErrInfo = "Position operand is out of range!";
    return false;
  }

  MachineOperand MOSize = MI.getOperand(3);
  if (!MOSize.isImm()) {
    ErrInfo = "Size operand is not an immediate!";
    return false;
  }
  int64_t Size = MOSize.getImm();
  if (!((SizeLow < Size) && (Size <= SizeHigh))) {
    ErrInfo = "Size operand is out of range!";
    return false;
  }

  if (!((BothLow < (Pos + Size)) && ((Pos + Size) <= BothHigh))) {
    ErrInfo = "Position + Size is out of range!";
    return false;
  }

  return true;
}

//  Perform target specific instruction verification.
bool RISCV64InstrInfo::verifyInstruction(const MachineInstr &MI,
                                      StringRef &ErrInfo) const {
  // Verify that ins and ext instructions are well formed.
  switch (MI.getOpcode()) {
    case RISCV64::EXT:
    case RISCV64::EXT_MM:
    case RISCV64::INS:
    case RISCV64::INS_MM:
    case RISCV64::DINS:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 32);
    case RISCV64::DINSM:
      // The ISA spec has a subtle difference between dinsm and dextm
      // in that it says:
      // 2 <= size <= 64 for 'dinsm' but 'dextm' has 32 < size <= 64.
      // To make the bounds checks similar, the range 1 < size <= 64 is checked
      // for 'dinsm'.
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 1, 64, 32, 64);
    case RISCV64::DINSU:
      // The ISA spec has a subtle difference between dinsu and dextu in that
      // the size range of dinsu is specified as 1 <= size <= 32 whereas size
      // for dextu is 0 < size <= 32. The range checked for dinsu here is
      // 0 < size <= 32, which is equivalent and similar to dextu.
      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 0, 32, 32, 64);
    case RISCV64::DEXT:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 63);
    case RISCV64::DEXTM:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 32, 64, 32, 64);
    case RISCV64::DEXTU:
      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 0, 32, 32, 64);
    case RISCV64::TAILCALLREG:
    case RISCV64::PseudoIndirectBranch:
    case RISCV64::JR:
    case RISCV64::JR64:
    case RISCV64::JALR:
    case RISCV64::JALR64:
    case RISCV64::JALRPseudo:
      if (!Subtarget.useIndirectJumpsHazard())
        return true;

      ErrInfo = "invalid instruction when using jump guards!";
      return false;
    default:
      return true;
  }

  return true;
}

std::pair<unsigned, unsigned>
RISCV64InstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<unsigned, const char*>>
RISCV64InstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
 using namespace RISCV64II;

 static const std::pair<unsigned, const char*> Flags[] = {
    {MO_GOT,          "mips-got"},
    {MO_GOT_CALL,     "mips-got-call"},
    {MO_GPREL,        "mips-gprel"},
    {MO_ABS_HI,       "mips-abs-hi"},
    {MO_ABS_LO,       "mips-abs-lo"},
    {MO_TLSGD,        "mips-tlsgd"},
    {MO_TLSLDM,       "mips-tlsldm"},
    {MO_DTPREL_HI,    "mips-dtprel-hi"},
    {MO_DTPREL_LO,    "mips-dtprel-lo"},
    {MO_GOTTPREL,     "mips-gottprel"},
    {MO_TPREL_HI,     "mips-tprel-hi"},
    {MO_TPREL_LO,     "mips-tprel-lo"},
    {MO_GPOFF_HI,     "mips-gpoff-hi"},
    {MO_GPOFF_LO,     "mips-gpoff-lo"},
    {MO_GOT_DISP,     "mips-got-disp"},
    {MO_GOT_PAGE,     "mips-got-page"},
    {MO_GOT_OFST,     "mips-got-ofst"},
    {MO_HIGHER,       "mips-higher"},
    {MO_HIGHEST,      "mips-highest"},
    {MO_GOT_HI16,     "mips-got-hi16"},
    {MO_GOT_LO16,     "mips-got-lo16"},
    {MO_CALL_HI16,    "mips-call-hi16"},
    {MO_CALL_LO16,    "mips-call-lo16"}
  };
  return makeArrayRef(Flags);
}
