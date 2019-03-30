//===-- MYRISCVXLongBranch.cpp - Emit long branches ---------------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass expands a branch or jump instruction into a long branch if its
// offset is too large to fit into its immediate field.
//
// FIXME: Fix pc-region jump instructions which cross 256MB segment boundaries.
//===----------------------------------------------------------------------===//
#include "MYRISCVX.h"

#include "MCTargetDesc/MYRISCVXBaseInfo.h"
#include "MYRISCVXMachineFunction.h"
#include "MYRISCVXTargetMachine.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/IR/Function.h"

#include "llvm/Support/CommandLine.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;
#define DEBUG_TYPE "myriscvx-long-branch"

STATISTIC(LongBranches, "Number of long branches.");
static cl::opt<bool> ForceLongBranch(
    "force-MYRISCVX-long-branch",
    cl::init(false),
    cl::desc("MYRISCVX: Expand all branches to long format."),
    cl::Hidden);
namespace {
typedef MachineBasicBlock::iterator Iter;
typedef MachineBasicBlock::reverse_iterator ReverseIter;

struct MBBInfo {
  uint64_t Size, Address;
  bool HasLongBranch;
  MachineInstr *Br;
  MBBInfo() : Size(0), HasLongBranch(false), Br(nullptr) {}
};


class MYRISCVXLongBranch : public MachineFunctionPass {
 public:
  static char ID;
  MYRISCVXLongBranch(TargetMachine &tm)
      : MachineFunctionPass(ID), TM(tm), IsPIC(TM.isPositionIndependent()),
        ABI(static_cast<const MYRISCVXTargetMachine &>(TM).getABI()) {}
  StringRef getPassName() const override {
    return "MYRISCVX Long Branch";
  }
  bool runOnMachineFunction(MachineFunction &F) override;
 private:
  void splitMBB(MachineBasicBlock *MBB);
  void initMBBInfo();
  int64_t computeOffset(const MachineInstr *Br);
  void replaceBranch(MachineBasicBlock &MBB, Iter Br, const DebugLoc &DL,
                     MachineBasicBlock *MBBOpnd);
  void expandToLongBranch(MBBInfo &Info);
  const TargetMachine &TM;
  MachineFunction *MFp;
  SmallVector<MBBInfo, 16> MBBInfos;
  bool IsPIC;
  MYRISCVXABIInfo ABI;
  unsigned LongBranchSeqSize;
};

char MYRISCVXLongBranch::ID = 0;
} // end of anonymous namespace


/// createMYRISCVXLongBranchPass - Returns a pass that converts branches to long
/// branches.
FunctionPass *llvm::createMYRISCVXLongBranchPass(MYRISCVXTargetMachine &tm) {
  return new MYRISCVXLongBranch(tm);
}


/// Iterate over list of Br's operands and search for a MachineBasicBlock
/// operand.
static MachineBasicBlock *getTargetMBB(const MachineInstr &Br) {
  for (unsigned I = 0, E = Br.getDesc().getNumOperands(); I < E; ++I) {
    const MachineOperand &MO = Br.getOperand(I);
    if (MO.isMBB())
      return MO.getMBB();
  }
  llvm_unreachable("This instruction does not have an MBB operand.");
}


// Traverse the list of instructions backwards until a non-debug instruction is
// found or it reaches E.
static ReverseIter getNonDebugInstr(ReverseIter B, const ReverseIter &E) {
  for (; B != E; ++B)
    if (!B->isDebugValue())
      return B;
  return E;
}


// Split MBB if it has two direct jumps/branches.
void MYRISCVXLongBranch::splitMBB(MachineBasicBlock *MBB) {
  ReverseIter End = MBB->rend();
  ReverseIter LastBr = getNonDebugInstr(MBB->rbegin(), End);
  // Return if MBB has no branch instructions.
  if ((LastBr == End) ||
      (!LastBr->isConditionalBranch() && !LastBr->isUnconditionalBranch()))
    return;
  ReverseIter FirstBr = getNonDebugInstr(std::next(LastBr), End);
  // MBB has only one branch instruction if FirstBr is not a branch
  // instruction.
  if ((FirstBr == End) ||
      (!FirstBr->isConditionalBranch() && !FirstBr->isUnconditionalBranch()))
    return;
  assert(!FirstBr->isIndirectBranch() && "Unexpected indirect branch found.");
  // Create a new MBB. Move instructions in MBB to the newly created MBB.
  MachineBasicBlock *NewMBB =
      MFp->CreateMachineBasicBlock(MBB->getBasicBlock());

  // Insert NewMBB and fix control flow.
  MachineBasicBlock *Tgt = getTargetMBB(*FirstBr);
  NewMBB->transferSuccessors(MBB);
  NewMBB->removeSuccessor(Tgt, true);
  MBB->addSuccessor(NewMBB);
  MBB->addSuccessor(Tgt);
  MFp->insert(std::next(MachineFunction::iterator(MBB)), NewMBB);

  NewMBB->splice(NewMBB->end(), MBB, LastBr.getReverse(), MBB->end());
}


// Fill MBBInfos.
void MYRISCVXLongBranch::initMBBInfo() {
  // Split the MBBs if they have two branches. Each basic block should have at
  // most one branch after this loop is executed.
  for (auto &MBB : *MFp)
    splitMBB(&MBB);
  MFp->RenumberBlocks();
  MBBInfos.clear();
  MBBInfos.resize(MFp->size());
  const MYRISCVXInstrInfo *TII =
      static_cast<const MYRISCVXInstrInfo *>(MFp->getSubtarget().getInstrInfo());
  for (unsigned I = 0, E = MBBInfos.size(); I < E; ++I) {
    MachineBasicBlock *MBB = MFp->getBlockNumbered(I);
    // Compute size of MBB.
    for (MachineBasicBlock::instr_iterator MI = MBB->instr_begin();
         MI != MBB->instr_end(); ++MI)
      MBBInfos[I].Size += TII->GetInstSizeInBytes(*MI);
    // Search for MBB's branch instruction.
    ReverseIter End = MBB->rend();
    ReverseIter Br = getNonDebugInstr(MBB->rbegin(), End);
    if ((Br != End) && !Br->isIndirectBranch() &&
        (Br->isConditionalBranch() || (Br->isUnconditionalBranch() && IsPIC)))
      MBBInfos[I].Br = &*Br;
  }
}


// Compute offset of branch in number of bytes.
int64_t MYRISCVXLongBranch::computeOffset(const MachineInstr *Br) {
  int64_t Offset = 0;
  int ThisMBB = Br->getParent()->getNumber();
  int TargetMBB = getTargetMBB(*Br)->getNumber();
  // Compute offset of a forward branch.
  if (ThisMBB < TargetMBB) {
    for (int N = ThisMBB + 1; N < TargetMBB; ++N)
      Offset += MBBInfos[N].Size;
    return Offset + 4;
  }
  // Compute offset of a backward branch.
  for (int N = ThisMBB; N >= TargetMBB; --N)
    Offset += MBBInfos[N].Size;
  return -Offset + 4;
}


// Replace Br with a branch which has the opposite condition code and a
// MachineBasicBlock operand MBBOpnd.
void MYRISCVXLongBranch::replaceBranch(MachineBasicBlock &MBB, Iter Br,
                                       const DebugLoc &DL,
                                       MachineBasicBlock *MBBOpnd) {
  const MYRISCVXInstrInfo *TII = static_cast<const MYRISCVXInstrInfo *>(
      MBB.getParent()->getSubtarget().getInstrInfo());
  unsigned NewOpc = TII->getOppositeBranchOpc(Br->getOpcode());
  const MCInstrDesc &NewDesc = TII->get(NewOpc);
  MachineInstrBuilder MIB = BuildMI(MBB, Br, DL, NewDesc);
  for (unsigned I = 0, E = Br->getDesc().getNumOperands(); I < E; ++I) {
    MachineOperand &MO = Br->getOperand(I);
    if (!MO.isReg()) {
      assert(MO.isMBB() && "MBB operand expected.");
      break;
    }
    MIB.addReg(MO.getReg());
  }
  MIB.addMBB(MBBOpnd);
  if (Br->hasDelaySlot()) {
    // Bundle the instruction in the delay slot to the newly created branch
    // and erase the original branch.
    assert(Br->isBundledWithSucc());
    MachineBasicBlock::instr_iterator II = Br.getInstrIterator();
    MIBundleBuilder(&*MIB).append((++II)->removeFromBundle());
  }
  Br->eraseFromParent();
}


// Expand branch instructions to long branches.
// TODO: This function has to be fixed for beqz16 and bnez16, because it
// currently assumes that all branches have 16-bit offsets, and will produce
// wrong code if branches whose allowed offsets are [-128, -126, ..., 126]
// are present.
void MYRISCVXLongBranch::expandToLongBranch(MBBInfo &I) {
  MachineBasicBlock::iterator Pos;
  MachineBasicBlock *MBB = I.Br->getParent(), *TgtMBB = getTargetMBB(*I.Br);
  DebugLoc DL = I.Br->getDebugLoc();
  const BasicBlock *BB = MBB->getBasicBlock();
  MachineFunction::iterator FallThroughMBB = ++MachineFunction::iterator(MBB);
  MachineBasicBlock *LongBrMBB = MFp->CreateMachineBasicBlock(BB);
  const MYRISCVXSubtarget &Subtarget =
      static_cast<const MYRISCVXSubtarget &>(MFp->getSubtarget());
  const MYRISCVXInstrInfo *TII =
      static_cast<const MYRISCVXInstrInfo *>(Subtarget.getInstrInfo());
  MFp->insert(FallThroughMBB, LongBrMBB);

  MBB->replaceSuccessor(TgtMBB, LongBrMBB);
  if (IsPIC) {
    // MachineBasicBlock *BalTgtMBB = MFp->CreateMachineBasicBlock(BB);
    // MFp->insert(FallThroughMBB, BalTgtMBB);
    // LongBrMBB->addSuccessor(BalTgtMBB);
    // BalTgtMBB->addSuccessor(TgtMBB);
    // unsigned BalOp = MYRISCVX::BAL;
    // // $longbr:
    // // addiu $sp, $sp, -8
    // // st $lr, 0($sp)
    // // lui $at, %hi($tgt - $baltgt)
    // // addiu $lr, $lr, %lo($tgt - $baltgt)
    // // bal $baltgt
    // // nop
    // // $baltgt:
    // // addu $at, $lr, $at
    // // addiu $sp, $sp, 8
    // // ld $lr, 0($sp)
    // // jr $at
    // // nop
    // // $fallthrough:
    // //
    // Pos = LongBrMBB->begin();
    // BuildMI(*LongBrMBB, Pos, DL, TII->get(MYRISCVX::ADDI), MYRISCVX::SP)
    //     .addReg(MYRISCVX::SP).addImm(-8);
    // BuildMI(*LongBrMBB, Pos, DL, TII->get(MYRISCVX::ST)).addReg(MYRISCVX::LR)
    //     .addReg(MYRISCVX::SP).addImm(0);
    // // LUi and ADDiu instructions create 32-bit offset of the target basic
    // // block from the target of BAL instruction. We cannot use immediate
    // // value for this offset because it cannot be determined accurately when
    // // the program has inline assembly statements. We therefore use the
    // // relocation expressions %hi($tgt-$baltgt) and %lo($tgt-$baltgt) which
    // // are resolved during the fixup, so the values will always be correct.
    // //
    // // Since we cannot create %hi($tgt-$baltgt) and %lo($tgt-$baltgt)
    // // expressions at this point (it is possible only at the MC layer),
    // // we replace LUi and ADDiu with pseudo instructions
    // // LONG_BRANCH_LUi and LONG_BRANCH_ADDiu, and add both basic
    // // blocks as operands to these instructions. When lowering these pseudo
    // // instructions to LUi and ADDiu in the MC layer, we will create
    // // %hi($tgt-$baltgt) and %lo($tgt-$baltgt) expressions and add them as
    // // operands to lowered instructions.
    // BuildMI(*LongBrMBB, Pos, DL, TII->get(MYRISCVX::LONG_BRANCH_LUi), MYRISCVX::AT)
    //     .addMBB(TgtMBB).addMBB(BalTgtMBB);
    // BuildMI(*LongBrMBB, Pos, DL, TII->get(MYRISCVX::LONG_BRANCH_ADDiu), MYRISCVX::AT)
    //     .addReg(MYRISCVX::AT).addMBB(TgtMBB).addMBB(BalTgtMBB);
    // MIBundleBuilder(*LongBrMBB, Pos)
    //     .append(BuildMI(*MFp, DL, TII->get(BalOp)).addMBB(BalTgtMBB));
    // Pos = BalTgtMBB->begin();
    // BuildMI(*BalTgtMBB, Pos, DL, TII->get(MYRISCVX::ADDu), MYRISCVX::AT)
    //     .addReg(MYRISCVX::LR).addReg(MYRISCVX::AT);
    // BuildMI(*BalTgtMBB, Pos, DL, TII->get(MYRISCVX::LD), MYRISCVX::LR)
    //     .addReg(MYRISCVX::SP).addImm(0);
    // BuildMI(*BalTgtMBB, Pos, DL, TII->get(MYRISCVX::ADDI), MYRISCVX::SP)
    //     .addReg(MYRISCVX::SP).addImm(8);
    // MIBundleBuilder(*BalTgtMBB, Pos)
    //     .append(BuildMI(*MFp, DL, TII->get(MYRISCVX::JR)).addReg(MYRISCVX::AT))
    //     .append(BuildMI(*MFp, DL, TII->get(MYRISCVX::NOP)));
    // assert(LongBrMBB->size() + BalTgtMBB->size() == LongBranchSeqSize);
  } else {
    // $longbr:
    // jmp $tgt
    // nop
    // $fallthrough:
    //
    Pos = LongBrMBB->begin();
    LongBrMBB->addSuccessor(TgtMBB);
    MIBundleBuilder(*LongBrMBB, Pos)
        .append(BuildMI(*MFp, DL, TII->get(MYRISCVX::JAL)).addMBB(TgtMBB))
        .append(BuildMI(*MFp, DL, TII->get(MYRISCVX::ADD)));
    assert(LongBrMBB->size() == LongBranchSeqSize);
  }
  if (I.Br->isUnconditionalBranch()) {
    // Change branch destination.
    assert(I.Br->getDesc().getNumOperands() == 1);
    I.Br->RemoveOperand(0);
    I.Br->addOperand(MachineOperand::CreateMBB(LongBrMBB));
  } else
    // Change branch destination and reverse condition.
    replaceBranch(*MBB, I.Br, DL, &*FallThroughMBB);
}


static void emitGPDisp(MachineFunction &F, const MYRISCVXInstrInfo *TII) {
  MachineBasicBlock &MBB = F.front();
  MachineBasicBlock::iterator I = MBB.begin();
  DebugLoc DL = MBB.findDebugLoc(MBB.begin());
  BuildMI(MBB, I, DL, TII->get(MYRISCVX::LUI), MYRISCVX::A0)
      .addExternalSymbol("_gp_disp", MYRISCVXII::MO_ABS_HI);
  BuildMI(MBB, I, DL, TII->get(MYRISCVX::ADDI), MYRISCVX::A0)
      .addReg(MYRISCVX::A0).addExternalSymbol("_gp_disp", MYRISCVXII::MO_ABS_LO);
  MBB.removeLiveIn(MYRISCVX::A0);
}


bool MYRISCVXLongBranch::runOnMachineFunction(MachineFunction &F) {
  const MYRISCVXSubtarget &STI =
      static_cast<const MYRISCVXSubtarget &>(F.getSubtarget());
  const MYRISCVXInstrInfo *TII =
      static_cast<const MYRISCVXInstrInfo *>(STI.getInstrInfo());
  LongBranchSeqSize =
      !IsPIC ? 2 : 10;
  // if (!STI.enableLongBranchPass())
  //   return false;
  if (IsPIC && static_cast<const MYRISCVXTargetMachine &>(TM).getABI().IsO32() &&
      F.getInfo<MYRISCVXFunctionInfo>()->globalBaseRegSet())
    emitGPDisp(F, TII);
  MFp = &F;
  initMBBInfo();
  SmallVectorImpl<MBBInfo>::iterator I, E = MBBInfos.end();
  bool EverMadeChange = false, MadeChange = true;
  while (MadeChange) {
    MadeChange = false;
    for (I = MBBInfos.begin(); I != E; ++I) {
      // Skip if this MBB doesn't have a branch or the branch has already been
      // converted to a long branch.
      if (!I->Br || I->HasLongBranch)
        continue;
      int ShVal = 4;
      int64_t Offset = computeOffset(I->Br) / ShVal;
      // Check if offset fits into 16-bit immediate field of branches.
      if (!ForceLongBranch && isInt<16>(Offset))
        continue;
      I->HasLongBranch = true;
      I->Size += LongBranchSeqSize * 4;
      ++LongBranches;
      EverMadeChange = MadeChange = true;
    }
  }
  if (!EverMadeChange)
    return true;
  // Compute basic block addresses.
  if (IsPIC) {
    uint64_t Address = 0;
    for (I = MBBInfos.begin(); I != E; Address += I->Size, ++I)
      I->Address = Address;
  }
  // Do the expansion.
  for (I = MBBInfos.begin(); I != E; ++I)
    if (I->HasLongBranch)
      expandToLongBranch(*I);
  MFp->RenumberBlocks();
  return true;
}
