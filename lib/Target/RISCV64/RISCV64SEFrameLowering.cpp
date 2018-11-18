//===- RISCV64SEFrameLowering.cpp - RISCV6432/64 Frame Information --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RISCV6432/64 implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "RISCV64SEFrameLowering.h"
#include "MCTargetDesc/RISCV64ABIInfo.h"
#include "RISCV64MachineFunction.h"
#include "RISCV64RegisterInfo.h"
#include "RISCV64SEInstrInfo.h"
#include "RISCV64Subtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCDwarf.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>

using namespace llvm;

static std::pair<unsigned, unsigned> getMFHiLoOpc(unsigned Src) {
  if (RISCV64::ACC64RegClass.contains(Src))
    return std::make_pair((unsigned)RISCV64::PseudoMFHI,
                          (unsigned)RISCV64::PseudoMFLO);

  if (RISCV64::ACC64DSPRegClass.contains(Src))
    return std::make_pair((unsigned)RISCV64::MFHI_DSP, (unsigned)RISCV64::MFLO_DSP);

  if (RISCV64::ACC128RegClass.contains(Src))
    return std::make_pair((unsigned)RISCV64::PseudoMFHI64,
                          (unsigned)RISCV64::PseudoMFLO64);

  return std::make_pair(0, 0);
}

namespace {

/// Helper class to expand pseudos.
class ExpandPseudo {
public:
  ExpandPseudo(MachineFunction &MF);
  bool expand();

private:
  using Iter = MachineBasicBlock::iterator;

  bool expandInstr(MachineBasicBlock &MBB, Iter I);
  void expandLoadCCond(MachineBasicBlock &MBB, Iter I);
  void expandStoreCCond(MachineBasicBlock &MBB, Iter I);
  void expandLoadACC(MachineBasicBlock &MBB, Iter I, unsigned RegSize);
  void expandStoreACC(MachineBasicBlock &MBB, Iter I, unsigned MFHiOpc,
                      unsigned MFLoOpc, unsigned RegSize);
  bool expandCopy(MachineBasicBlock &MBB, Iter I);
  bool expandCopyACC(MachineBasicBlock &MBB, Iter I, unsigned MFHiOpc,
                     unsigned MFLoOpc);
  bool expandBuildPairF64(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator I, bool FP64) const;
  bool expandExtractElementF64(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I, bool FP64) const;

  MachineFunction &MF;
  MachineRegisterInfo &MRI;
  const RISCV64Subtarget &Subtarget;
  const RISCV64SEInstrInfo &TII;
  const RISCV64RegisterInfo &RegInfo;
};

} // end anonymous namespace

ExpandPseudo::ExpandPseudo(MachineFunction &MF_)
    : MF(MF_), MRI(MF.getRegInfo()),
      Subtarget(static_cast<const RISCV64Subtarget &>(MF.getSubtarget())),
      TII(*static_cast<const RISCV64SEInstrInfo *>(Subtarget.getInstrInfo())),
      RegInfo(*Subtarget.getRegisterInfo()) {}

bool ExpandPseudo::expand() {
  bool Expanded = false;

  for (auto &MBB : MF) {
    for (Iter I = MBB.begin(), End = MBB.end(); I != End;)
      Expanded |= expandInstr(MBB, I++);
  }

  return Expanded;
}

bool ExpandPseudo::expandInstr(MachineBasicBlock &MBB, Iter I) {
  switch(I->getOpcode()) {
  case RISCV64::LOAD_CCOND_DSP:
    expandLoadCCond(MBB, I);
    break;
  case RISCV64::STORE_CCOND_DSP:
    expandStoreCCond(MBB, I);
    break;
  case RISCV64::LOAD_ACC64:
  case RISCV64::LOAD_ACC64DSP:
    expandLoadACC(MBB, I, 4);
    break;
  case RISCV64::LOAD_ACC128:
    expandLoadACC(MBB, I, 8);
    break;
  case RISCV64::STORE_ACC64:
    expandStoreACC(MBB, I, RISCV64::PseudoMFHI, RISCV64::PseudoMFLO, 4);
    break;
  case RISCV64::STORE_ACC64DSP:
    expandStoreACC(MBB, I, RISCV64::MFHI_DSP, RISCV64::MFLO_DSP, 4);
    break;
  case RISCV64::STORE_ACC128:
    expandStoreACC(MBB, I, RISCV64::PseudoMFHI64, RISCV64::PseudoMFLO64, 8);
    break;
  case RISCV64::BuildPairF64:
    if (expandBuildPairF64(MBB, I, false))
      MBB.erase(I);
    return false;
  case RISCV64::BuildPairF64_64:
    if (expandBuildPairF64(MBB, I, true))
      MBB.erase(I);
    return false;
  case RISCV64::ExtractElementF64:
    if (expandExtractElementF64(MBB, I, false))
      MBB.erase(I);
    return false;
  case RISCV64::ExtractElementF64_64:
    if (expandExtractElementF64(MBB, I, true))
      MBB.erase(I);
    return false;
  case TargetOpcode::COPY:
    if (!expandCopy(MBB, I))
      return false;
    break;
  default:
    return false;
  }

  MBB.erase(I);
  return true;
}

void ExpandPseudo::expandLoadCCond(MachineBasicBlock &MBB, Iter I) {
  //  load $vr, FI
  //  copy ccond, $vr

  assert(I->getOperand(0).isReg() && I->getOperand(1).isFI());

  const TargetRegisterClass *RC = RegInfo.intRegClass(4);
  unsigned VR = MRI.createVirtualRegister(RC);
  unsigned Dst = I->getOperand(0).getReg(), FI = I->getOperand(1).getIndex();

  TII.loadRegFromStack(MBB, I, VR, FI, RC, &RegInfo, 0);
  BuildMI(MBB, I, I->getDebugLoc(), TII.get(TargetOpcode::COPY), Dst)
    .addReg(VR, RegState::Kill);
}

void ExpandPseudo::expandStoreCCond(MachineBasicBlock &MBB, Iter I) {
  //  copy $vr, ccond
  //  store $vr, FI

  assert(I->getOperand(0).isReg() && I->getOperand(1).isFI());

  const TargetRegisterClass *RC = RegInfo.intRegClass(4);
  unsigned VR = MRI.createVirtualRegister(RC);
  unsigned Src = I->getOperand(0).getReg(), FI = I->getOperand(1).getIndex();

  BuildMI(MBB, I, I->getDebugLoc(), TII.get(TargetOpcode::COPY), VR)
    .addReg(Src, getKillRegState(I->getOperand(0).isKill()));
  TII.storeRegToStack(MBB, I, VR, true, FI, RC, &RegInfo, 0);
}

void ExpandPseudo::expandLoadACC(MachineBasicBlock &MBB, Iter I,
                                 unsigned RegSize) {
  //  load $vr0, FI
  //  copy lo, $vr0
  //  load $vr1, FI + 4
  //  copy hi, $vr1

  assert(I->getOperand(0).isReg() && I->getOperand(1).isFI());

  const TargetRegisterClass *RC = RegInfo.intRegClass(RegSize);
  unsigned VR0 = MRI.createVirtualRegister(RC);
  unsigned VR1 = MRI.createVirtualRegister(RC);
  unsigned Dst = I->getOperand(0).getReg(), FI = I->getOperand(1).getIndex();
  unsigned Lo = RegInfo.getSubReg(Dst, RISCV64::sub_lo);
  unsigned Hi = RegInfo.getSubReg(Dst, RISCV64::sub_hi);
  DebugLoc DL = I->getDebugLoc();
  const MCInstrDesc &Desc = TII.get(TargetOpcode::COPY);

  TII.loadRegFromStack(MBB, I, VR0, FI, RC, &RegInfo, 0);
  BuildMI(MBB, I, DL, Desc, Lo).addReg(VR0, RegState::Kill);
  TII.loadRegFromStack(MBB, I, VR1, FI, RC, &RegInfo, RegSize);
  BuildMI(MBB, I, DL, Desc, Hi).addReg(VR1, RegState::Kill);
}

void ExpandPseudo::expandStoreACC(MachineBasicBlock &MBB, Iter I,
                                  unsigned MFHiOpc, unsigned MFLoOpc,
                                  unsigned RegSize) {
  //  mflo $vr0, src
  //  store $vr0, FI
  //  mfhi $vr1, src
  //  store $vr1, FI + 4

  assert(I->getOperand(0).isReg() && I->getOperand(1).isFI());

  const TargetRegisterClass *RC = RegInfo.intRegClass(RegSize);
  unsigned VR0 = MRI.createVirtualRegister(RC);
  unsigned VR1 = MRI.createVirtualRegister(RC);
  unsigned Src = I->getOperand(0).getReg(), FI = I->getOperand(1).getIndex();
  unsigned SrcKill = getKillRegState(I->getOperand(0).isKill());
  DebugLoc DL = I->getDebugLoc();

  BuildMI(MBB, I, DL, TII.get(MFLoOpc), VR0).addReg(Src);
  TII.storeRegToStack(MBB, I, VR0, true, FI, RC, &RegInfo, 0);
  BuildMI(MBB, I, DL, TII.get(MFHiOpc), VR1).addReg(Src, SrcKill);
  TII.storeRegToStack(MBB, I, VR1, true, FI, RC, &RegInfo, RegSize);
}

bool ExpandPseudo::expandCopy(MachineBasicBlock &MBB, Iter I) {
  unsigned Src = I->getOperand(1).getReg();
  std::pair<unsigned, unsigned> Opcodes = getMFHiLoOpc(Src);

  if (!Opcodes.first)
    return false;

  return expandCopyACC(MBB, I, Opcodes.first, Opcodes.second);
}

bool ExpandPseudo::expandCopyACC(MachineBasicBlock &MBB, Iter I,
                                 unsigned MFHiOpc, unsigned MFLoOpc) {
  //  mflo $vr0, src
  //  copy dst_lo, $vr0
  //  mfhi $vr1, src
  //  copy dst_hi, $vr1

  unsigned Dst = I->getOperand(0).getReg(), Src = I->getOperand(1).getReg();
  const TargetRegisterClass *DstRC = RegInfo.getMinimalPhysRegClass(Dst);
  unsigned VRegSize = RegInfo.getRegSizeInBits(*DstRC) / 16;
  const TargetRegisterClass *RC = RegInfo.intRegClass(VRegSize);
  unsigned VR0 = MRI.createVirtualRegister(RC);
  unsigned VR1 = MRI.createVirtualRegister(RC);
  unsigned SrcKill = getKillRegState(I->getOperand(1).isKill());
  unsigned DstLo = RegInfo.getSubReg(Dst, RISCV64::sub_lo);
  unsigned DstHi = RegInfo.getSubReg(Dst, RISCV64::sub_hi);
  DebugLoc DL = I->getDebugLoc();

  BuildMI(MBB, I, DL, TII.get(MFLoOpc), VR0).addReg(Src);
  BuildMI(MBB, I, DL, TII.get(TargetOpcode::COPY), DstLo)
    .addReg(VR0, RegState::Kill);
  BuildMI(MBB, I, DL, TII.get(MFHiOpc), VR1).addReg(Src, SrcKill);
  BuildMI(MBB, I, DL, TII.get(TargetOpcode::COPY), DstHi)
    .addReg(VR1, RegState::Kill);
  return true;
}

/// This method expands the same instruction that RISCV64SEInstrInfo::
/// expandBuildPairF64 does, for the case when ABI is fpxx and mthc1 is not
/// available and the case where the ABI is FP64A. It is implemented here
/// because frame indexes are eliminated before RISCV64SEInstrInfo::
/// expandBuildPairF64 is called.
bool ExpandPseudo::expandBuildPairF64(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator I,
                                      bool FP64) const {
  // For fpxx and when mthc1 is not available, use:
  //   spill + reload via ldc1
  //
  // The case where dmtc1 is available doesn't need to be handled here
  // because it never creates a BuildPairF64 node.
  //
  // The FP64A ABI (fp64 with nooddspreg) must also use a spill/reload sequence
  // for odd-numbered double precision values (because the lower 32-bits is
  // transferred with mtc1 which is redirected to the upper half of the even
  // register). Unfortunately, we have to make this decision before register
  // allocation so for now we use a spill/reload sequence for all
  // double-precision values in regardless of being an odd/even register.
  //
  // For the cases that should be covered here RISCV64SEISelDAGToDAG adds $sp as
  // implicit operand, so other passes (like ShrinkWrapping) are aware that
  // stack is used.
  if (I->getNumOperands() == 4 && I->getOperand(3).isReg()
      && I->getOperand(3).getReg() == RISCV64::SP) {
    unsigned DstReg = I->getOperand(0).getReg();
    unsigned LoReg = I->getOperand(1).getReg();
    unsigned HiReg = I->getOperand(2).getReg();

    // It should be impossible to have FGR64 on MIPS-II or MIPS32r1 (which are
    // the cases where mthc1 is not available). 64-bit architectures and
    // MIPS32r2 or later can use FGR64 though.
    assert(Subtarget.isGP64bit() || Subtarget.hasMTHC1() ||
           !Subtarget.isFP64bit());

    const TargetRegisterClass *RC = &RISCV64::GPR32RegClass;
    const TargetRegisterClass *RC2 =
        FP64 ? &RISCV64::FGR64RegClass : &RISCV64::AFGR64RegClass;

    // We re-use the same spill slot each time so that the stack frame doesn't
    // grow too much in functions with a large number of moves.
    int FI = MF.getInfo<RISCV64FunctionInfo>()->getMoveF64ViaSpillFI(RC2);
    if (!Subtarget.isLittle())
      std::swap(LoReg, HiReg);
    TII.storeRegToStack(MBB, I, LoReg, I->getOperand(1).isKill(), FI, RC,
                        &RegInfo, 0);
    TII.storeRegToStack(MBB, I, HiReg, I->getOperand(2).isKill(), FI, RC,
                        &RegInfo, 4);
    TII.loadRegFromStack(MBB, I, DstReg, FI, RC2, &RegInfo, 0);
    return true;
  }

  return false;
}

/// This method expands the same instruction that RISCV64SEInstrInfo::
/// expandExtractElementF64 does, for the case when ABI is fpxx and mfhc1 is not
/// available and the case where the ABI is FP64A. It is implemented here
/// because frame indexes are eliminated before RISCV64SEInstrInfo::
/// expandExtractElementF64 is called.
bool ExpandPseudo::expandExtractElementF64(MachineBasicBlock &MBB,
                                           MachineBasicBlock::iterator I,
                                           bool FP64) const {
  const MachineOperand &Op1 = I->getOperand(1);
  const MachineOperand &Op2 = I->getOperand(2);

  if ((Op1.isReg() && Op1.isUndef()) || (Op2.isReg() && Op2.isUndef())) {
    unsigned DstReg = I->getOperand(0).getReg();
    BuildMI(MBB, I, I->getDebugLoc(), TII.get(RISCV64::IMPLICIT_DEF), DstReg);
    return true;
  }

  // For fpxx and when mfhc1 is not available, use:
  //   spill + reload via ldc1
  //
  // The case where dmfc1 is available doesn't need to be handled here
  // because it never creates a ExtractElementF64 node.
  //
  // The FP64A ABI (fp64 with nooddspreg) must also use a spill/reload sequence
  // for odd-numbered double precision values (because the lower 32-bits is
  // transferred with mfc1 which is redirected to the upper half of the even
  // register). Unfortunately, we have to make this decision before register
  // allocation so for now we use a spill/reload sequence for all
  // double-precision values in regardless of being an odd/even register.
  //
  // For the cases that should be covered here RISCV64SEISelDAGToDAG adds $sp as
  // implicit operand, so other passes (like ShrinkWrapping) are aware that
  // stack is used.
  if (I->getNumOperands() == 4 && I->getOperand(3).isReg()
      && I->getOperand(3).getReg() == RISCV64::SP) {
    unsigned DstReg = I->getOperand(0).getReg();
    unsigned SrcReg = Op1.getReg();
    unsigned N = Op2.getImm();
    int64_t Offset = 4 * (Subtarget.isLittle() ? N : (1 - N));

    // It should be impossible to have FGR64 on MIPS-II or MIPS32r1 (which are
    // the cases where mfhc1 is not available). 64-bit architectures and
    // MIPS32r2 or later can use FGR64 though.
    assert(Subtarget.isGP64bit() || Subtarget.hasMTHC1() ||
           !Subtarget.isFP64bit());

    const TargetRegisterClass *RC =
        FP64 ? &RISCV64::FGR64RegClass : &RISCV64::AFGR64RegClass;
    const TargetRegisterClass *RC2 = &RISCV64::GPR32RegClass;

    // We re-use the same spill slot each time so that the stack frame doesn't
    // grow too much in functions with a large number of moves.
    int FI = MF.getInfo<RISCV64FunctionInfo>()->getMoveF64ViaSpillFI(RC);
    TII.storeRegToStack(MBB, I, SrcReg, Op1.isKill(), FI, RC, &RegInfo, 0);
    TII.loadRegFromStack(MBB, I, DstReg, FI, RC2, &RegInfo, Offset);
    return true;
  }

  return false;
}

RISCV64SEFrameLowering::RISCV64SEFrameLowering(const RISCV64Subtarget &STI)
    : RISCV64FrameLowering(STI, STI.getStackAlignment()) {}

void RISCV64SEFrameLowering::emitPrologue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {
  MachineFrameInfo &MFI    = MF.getFrameInfo();
  RISCV64FunctionInfo *RISCV64FI = MF.getInfo<RISCV64FunctionInfo>();

  const RISCV64SEInstrInfo &TII =
      *static_cast<const RISCV64SEInstrInfo *>(STI.getInstrInfo());
  const RISCV64RegisterInfo &RegInfo =
      *static_cast<const RISCV64RegisterInfo *>(STI.getRegisterInfo());

  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc dl;
  RISCV64ABIInfo ABI = STI.getABI();
  unsigned SP = ABI.GetStackPtr();
  unsigned FP = ABI.GetFramePtr();
  unsigned ZERO = ABI.GetNullPtr();
  unsigned MOVE = ABI.GetGPRMoveOp();
  unsigned ADDiu = ABI.GetPtrAddiuOp();
  unsigned AND = ABI.IsN64() ? RISCV64::AND64 : RISCV64::AND;

  const TargetRegisterClass *RC = ABI.ArePtrs64bit() ?
        &RISCV64::GPR64RegClass : &RISCV64::GPR32RegClass;

  // First, compute final stack size.
  uint64_t StackSize = MFI.getStackSize();

  // No need to allocate space on the stack.
  if (StackSize == 0 && !MFI.adjustsStack()) return;

  MachineModuleInfo &MMI = MF.getMMI();
  const MCRegisterInfo *MRI = MMI.getContext().getRegisterInfo();

  // Adjust stack.
  TII.adjustStackPtr(SP, -StackSize, MBB, MBBI);

  // emit ".cfi_def_cfa_offset StackSize"
  unsigned CFIIndex = MF.addFrameInst(
      MCCFIInstruction::createDefCfaOffset(nullptr, -StackSize));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  if (MF.getFunction().hasFnAttribute("interrupt"))
    emitInterruptPrologueStub(MF, MBB);

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();

  if (!CSI.empty()) {
    // Find the instruction past the last instruction that saves a callee-saved
    // register to the stack.
    for (unsigned i = 0; i < CSI.size(); ++i)
      ++MBBI;

    // Iterate over list of callee-saved registers and emit .cfi_offset
    // directives.
    for (std::vector<CalleeSavedInfo>::const_iterator I = CSI.begin(),
           E = CSI.end(); I != E; ++I) {
      int64_t Offset = MFI.getObjectOffset(I->getFrameIdx());
      unsigned Reg = I->getReg();

      // If Reg is a double precision register, emit two cfa_offsets,
      // one for each of the paired single precision registers.
      if (RISCV64::AFGR64RegClass.contains(Reg)) {
        unsigned Reg0 =
            MRI->getDwarfRegNum(RegInfo.getSubReg(Reg, RISCV64::sub_lo), true);
        unsigned Reg1 =
            MRI->getDwarfRegNum(RegInfo.getSubReg(Reg, RISCV64::sub_hi), true);

        if (!STI.isLittle())
          std::swap(Reg0, Reg1);

        unsigned CFIIndex = MF.addFrameInst(
            MCCFIInstruction::createOffset(nullptr, Reg0, Offset));
        BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
            .addCFIIndex(CFIIndex);

        CFIIndex = MF.addFrameInst(
            MCCFIInstruction::createOffset(nullptr, Reg1, Offset + 4));
        BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
            .addCFIIndex(CFIIndex);
      } else if (RISCV64::FGR64RegClass.contains(Reg)) {
        unsigned Reg0 = MRI->getDwarfRegNum(Reg, true);
        unsigned Reg1 = MRI->getDwarfRegNum(Reg, true) + 1;

        if (!STI.isLittle())
          std::swap(Reg0, Reg1);

        unsigned CFIIndex = MF.addFrameInst(
          MCCFIInstruction::createOffset(nullptr, Reg0, Offset));
        BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
            .addCFIIndex(CFIIndex);

        CFIIndex = MF.addFrameInst(
          MCCFIInstruction::createOffset(nullptr, Reg1, Offset + 4));
        BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
            .addCFIIndex(CFIIndex);
      } else {
        // Reg is either in GPR32 or FGR32.
        unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createOffset(
            nullptr, MRI->getDwarfRegNum(Reg, true), Offset));
        BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
            .addCFIIndex(CFIIndex);
      }
    }
  }

  if (RISCV64FI->callsEhReturn()) {
    // Insert instructions that spill eh data registers.
    for (int I = 0; I < 4; ++I) {
      if (!MBB.isLiveIn(ABI.GetEhDataReg(I)))
        MBB.addLiveIn(ABI.GetEhDataReg(I));
      TII.storeRegToStackSlot(MBB, MBBI, ABI.GetEhDataReg(I), false,
                              RISCV64FI->getEhDataRegFI(I), RC, &RegInfo);
    }

    // Emit .cfi_offset directives for eh data registers.
    for (int I = 0; I < 4; ++I) {
      int64_t Offset = MFI.getObjectOffset(RISCV64FI->getEhDataRegFI(I));
      unsigned Reg = MRI->getDwarfRegNum(ABI.GetEhDataReg(I), true);
      unsigned CFIIndex = MF.addFrameInst(
          MCCFIInstruction::createOffset(nullptr, Reg, Offset));
      BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
          .addCFIIndex(CFIIndex);
    }
  }

  // if framepointer enabled, set it to point to the stack pointer.
  if (hasFP(MF)) {
    // Insert instruction "move $fp, $sp" at this location.
    BuildMI(MBB, MBBI, dl, TII.get(MOVE), FP).addReg(SP).addReg(ZERO)
      .setMIFlag(MachineInstr::FrameSetup);

    // emit ".cfi_def_cfa_register $fp"
    unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createDefCfaRegister(
        nullptr, MRI->getDwarfRegNum(FP, true)));
    BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
        .addCFIIndex(CFIIndex);

    if (RegInfo.needsStackRealignment(MF)) {
      // addiu $Reg, $zero, -MaxAlignment
      // andi $sp, $sp, $Reg
      unsigned VR = MF.getRegInfo().createVirtualRegister(RC);
      assert(isInt<16>(MFI.getMaxAlignment()) &&
             "Function's alignment size requirement is not supported.");
      int MaxAlign = -(int)MFI.getMaxAlignment();

      BuildMI(MBB, MBBI, dl, TII.get(ADDiu), VR).addReg(ZERO) .addImm(MaxAlign);
      BuildMI(MBB, MBBI, dl, TII.get(AND), SP).addReg(SP).addReg(VR);

      if (hasBP(MF)) {
        // move $s7, $sp
        unsigned BP = STI.isABI_N64() ? RISCV64::S7_64 : RISCV64::S7;
        BuildMI(MBB, MBBI, dl, TII.get(MOVE), BP)
          .addReg(SP)
          .addReg(ZERO);
      }
    }
  }
}

void RISCV64SEFrameLowering::emitInterruptPrologueStub(
    MachineFunction &MF, MachineBasicBlock &MBB) const {
  RISCV64FunctionInfo *RISCV64FI = MF.getInfo<RISCV64FunctionInfo>();
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();

  // Report an error the target doesn't support RISCV6432r2 or later.
  // The epilogue relies on the use of the "ehb" to clear execution
  // hazards. Pre R2 RISCV64 relies on an implementation defined number
  // of "ssnop"s to clear the execution hazard. Support for ssnop hazard
  // clearing is not provided so reject that configuration.
  if (!STI.hasRISCV6432r2())
    report_fatal_error(
        "\"interrupt\" attribute is not supported on pre-MIPS32R2 or "
        "MIPS16 targets.");

  // The GP register contains the "user" value, so we cannot perform
  // any gp relative loads until we restore the "kernel" or "system" gp
  // value. Until support is written we shall only accept the static
  // relocation model.
  if ((STI.getRelocationModel() != Reloc::Static))
    report_fatal_error("\"interrupt\" attribute is only supported for the "
                       "static relocation model on MIPS at the present time.");

  if (!STI.isABI_O32() || STI.hasRISCV6464())
    report_fatal_error("\"interrupt\" attribute is only supported for the "
                       "O32 ABI on MIPS32R2+ at the present time.");

  // Perform ISR handling like GCC
  StringRef IntKind =
      MF.getFunction().getFnAttribute("interrupt").getValueAsString();
  const TargetRegisterClass *PtrRC = &RISCV64::GPR32RegClass;

  // EIC interrupt handling needs to read the Cause register to disable
  // interrupts.
  if (IntKind == "eic") {
    // Coprocessor registers are always live per se.
    MBB.addLiveIn(RISCV64::COP013);
    BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::MFC0), RISCV64::K0)
        .addReg(RISCV64::COP013)
        .addImm(0)
        .setMIFlag(MachineInstr::FrameSetup);

    BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::EXT), RISCV64::K0)
        .addReg(RISCV64::K0)
        .addImm(10)
        .addImm(6)
        .setMIFlag(MachineInstr::FrameSetup);
  }

  // Fetch and spill EPC
  MBB.addLiveIn(RISCV64::COP014);
  BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::MFC0), RISCV64::K1)
      .addReg(RISCV64::COP014)
      .addImm(0)
      .setMIFlag(MachineInstr::FrameSetup);

  STI.getInstrInfo()->storeRegToStack(MBB, MBBI, RISCV64::K1, false,
                                      RISCV64FI->getISRRegFI(0), PtrRC,
                                      STI.getRegisterInfo(), 0);

  // Fetch and Spill Status
  MBB.addLiveIn(RISCV64::COP012);
  BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::MFC0), RISCV64::K1)
      .addReg(RISCV64::COP012)
      .addImm(0)
      .setMIFlag(MachineInstr::FrameSetup);

  STI.getInstrInfo()->storeRegToStack(MBB, MBBI, RISCV64::K1, false,
                                      RISCV64FI->getISRRegFI(1), PtrRC,
                                      STI.getRegisterInfo(), 0);

  // Build the configuration for disabling lower priority interrupts. Non EIC
  // interrupts need to be masked off with zero, EIC from the Cause register.
  unsigned InsPosition = 8;
  unsigned InsSize = 0;
  unsigned SrcReg = RISCV64::ZERO;

  // If the interrupt we're tied to is the EIC, switch the source for the
  // masking off interrupts to the cause register.
  if (IntKind == "eic") {
    SrcReg = RISCV64::K0;
    InsPosition = 10;
    InsSize = 6;
  } else
    InsSize = StringSwitch<unsigned>(IntKind)
                  .Case("sw0", 1)
                  .Case("sw1", 2)
                  .Case("hw0", 3)
                  .Case("hw1", 4)
                  .Case("hw2", 5)
                  .Case("hw3", 6)
                  .Case("hw4", 7)
                  .Case("hw5", 8)
                  .Default(0);
  assert(InsSize != 0 && "Unknown interrupt type!");

  BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::INS), RISCV64::K1)
      .addReg(SrcReg)
      .addImm(InsPosition)
      .addImm(InsSize)
      .addReg(RISCV64::K1)
      .setMIFlag(MachineInstr::FrameSetup);

  // Mask off KSU, ERL, EXL
  BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::INS), RISCV64::K1)
      .addReg(RISCV64::ZERO)
      .addImm(1)
      .addImm(4)
      .addReg(RISCV64::K1)
      .setMIFlag(MachineInstr::FrameSetup);

  // Disable the FPU as we are not spilling those register sets.
  if (!STI.useSoftFloat())
    BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::INS), RISCV64::K1)
        .addReg(RISCV64::ZERO)
        .addImm(29)
        .addImm(1)
        .addReg(RISCV64::K1)
        .setMIFlag(MachineInstr::FrameSetup);

  // Set the new status
  BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::MTC0), RISCV64::COP012)
      .addReg(RISCV64::K1)
      .addImm(0)
      .setMIFlag(MachineInstr::FrameSetup);
}

void RISCV64SEFrameLowering::emitEpilogue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.getFirstTerminator();
  MachineFrameInfo &MFI            = MF.getFrameInfo();
  RISCV64FunctionInfo *RISCV64FI = MF.getInfo<RISCV64FunctionInfo>();

  const RISCV64SEInstrInfo &TII =
      *static_cast<const RISCV64SEInstrInfo *>(STI.getInstrInfo());
  const RISCV64RegisterInfo &RegInfo =
      *static_cast<const RISCV64RegisterInfo *>(STI.getRegisterInfo());

  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  RISCV64ABIInfo ABI = STI.getABI();
  unsigned SP = ABI.GetStackPtr();
  unsigned FP = ABI.GetFramePtr();
  unsigned ZERO = ABI.GetNullPtr();
  unsigned MOVE = ABI.GetGPRMoveOp();

  // if framepointer enabled, restore the stack pointer.
  if (hasFP(MF)) {
    // Find the first instruction that restores a callee-saved register.
    MachineBasicBlock::iterator I = MBBI;

    for (unsigned i = 0; i < MFI.getCalleeSavedInfo().size(); ++i)
      --I;

    // Insert instruction "move $sp, $fp" at this location.
    BuildMI(MBB, I, DL, TII.get(MOVE), SP).addReg(FP).addReg(ZERO);
  }

  if (RISCV64FI->callsEhReturn()) {
    const TargetRegisterClass *RC =
        ABI.ArePtrs64bit() ? &RISCV64::GPR64RegClass : &RISCV64::GPR32RegClass;

    // Find first instruction that restores a callee-saved register.
    MachineBasicBlock::iterator I = MBBI;
    for (unsigned i = 0; i < MFI.getCalleeSavedInfo().size(); ++i)
      --I;

    // Insert instructions that restore eh data registers.
    for (int J = 0; J < 4; ++J) {
      TII.loadRegFromStackSlot(MBB, I, ABI.GetEhDataReg(J),
                               RISCV64FI->getEhDataRegFI(J), RC, &RegInfo);
    }
  }

  if (MF.getFunction().hasFnAttribute("interrupt"))
    emitInterruptEpilogueStub(MF, MBB);

  // Get the number of bytes from FrameInfo
  uint64_t StackSize = MFI.getStackSize();

  if (!StackSize)
    return;

  // Adjust stack.
  TII.adjustStackPtr(SP, StackSize, MBB, MBBI);
}

void RISCV64SEFrameLowering::emitInterruptEpilogueStub(
    MachineFunction &MF, MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  RISCV64FunctionInfo *RISCV64FI = MF.getInfo<RISCV64FunctionInfo>();
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();

  // Perform ISR handling like GCC
  const TargetRegisterClass *PtrRC = &RISCV64::GPR32RegClass;

  // Disable Interrupts.
  BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::DI), RISCV64::ZERO);
  BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::EHB));

  // Restore EPC
  STI.getInstrInfo()->loadRegFromStackSlot(MBB, MBBI, RISCV64::K1,
                                           RISCV64FI->getISRRegFI(0), PtrRC,
                                           STI.getRegisterInfo());
  BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::MTC0), RISCV64::COP014)
      .addReg(RISCV64::K1)
      .addImm(0);

  // Restore Status
  STI.getInstrInfo()->loadRegFromStackSlot(MBB, MBBI, RISCV64::K1,
                                           RISCV64FI->getISRRegFI(1), PtrRC,
                                           STI.getRegisterInfo());
  BuildMI(MBB, MBBI, DL, STI.getInstrInfo()->get(RISCV64::MTC0), RISCV64::COP012)
      .addReg(RISCV64::K1)
      .addImm(0);
}

int RISCV64SEFrameLowering::getFrameIndexReference(const MachineFunction &MF,
                                                int FI,
                                                unsigned &FrameReg) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  RISCV64ABIInfo ABI = STI.getABI();

  if (MFI.isFixedObjectIndex(FI))
    FrameReg = hasFP(MF) ? ABI.GetFramePtr() : ABI.GetStackPtr();
  else
    FrameReg = hasBP(MF) ? ABI.GetBasePtr() : ABI.GetStackPtr();

  return MFI.getObjectOffset(FI) + MFI.getStackSize() -
         getOffsetOfLocalArea() + MFI.getOffsetAdjustment();
}

bool RISCV64SEFrameLowering::
spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI,
                          const std::vector<CalleeSavedInfo> &CSI,
                          const TargetRegisterInfo *TRI) const {
  MachineFunction *MF = MBB.getParent();
  const TargetInstrInfo &TII = *STI.getInstrInfo();

  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
    // Add the callee-saved register as live-in. Do not add if the register is
    // RA and return address is taken, because it has already been added in
    // method RISCV64TargetLowering::lowerRETURNADDR.
    // It's killed at the spill, unless the register is RA and return address
    // is taken.
    unsigned Reg = CSI[i].getReg();
    bool IsRAAndRetAddrIsTaken = (Reg == RISCV64::RA || Reg == RISCV64::RA_64)
        && MF->getFrameInfo().isReturnAddressTaken();
    if (!IsRAAndRetAddrIsTaken)
      MBB.addLiveIn(Reg);

    // ISRs require HI/LO to be spilled into kernel registers to be then
    // spilled to the stack frame.
    bool IsLOHI = (Reg == RISCV64::LO0 || Reg == RISCV64::LO0_64 ||
                   Reg == RISCV64::HI0 || Reg == RISCV64::HI0_64);
    const Function &Func = MBB.getParent()->getFunction();
    if (IsLOHI && Func.hasFnAttribute("interrupt")) {
      DebugLoc DL = MI->getDebugLoc();

      unsigned Op = 0;
      if (!STI.getABI().ArePtrs64bit()) {
        Op = (Reg == RISCV64::HI0) ? RISCV64::MFHI : RISCV64::MFLO;
        Reg = RISCV64::K0;
      } else {
        Op = (Reg == RISCV64::HI0) ? RISCV64::MFHI64 : RISCV64::MFLO64;
        Reg = RISCV64::K0_64;
      }
      BuildMI(MBB, MI, DL, TII.get(Op), RISCV64::K0)
          .setMIFlag(MachineInstr::FrameSetup);
    }

    // Insert the spill to the stack frame.
    bool IsKill = !IsRAAndRetAddrIsTaken;
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.storeRegToStackSlot(MBB, MI, Reg, IsKill,
                            CSI[i].getFrameIdx(), RC, TRI);
  }

  return true;
}

bool
RISCV64SEFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  // Reserve call frame if the size of the maximum call frame fits into 16-bit
  // immediate field and there are no variable sized objects on the stack.
  // Make sure the second register scavenger spill slot can be accessed with one
  // instruction.
  return isInt<16>(MFI.getMaxCallFrameSize() + getStackAlignment()) &&
    !MFI.hasVarSizedObjects();
}

/// Mark \p Reg and all registers aliasing it in the bitset.
static void setAliasRegs(MachineFunction &MF, BitVector &SavedRegs,
                         unsigned Reg) {
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  for (MCRegAliasIterator AI(Reg, TRI, true); AI.isValid(); ++AI)
    SavedRegs.set(*AI);
}

void RISCV64SEFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                               BitVector &SavedRegs,
                                               RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  RISCV64FunctionInfo *RISCV64FI = MF.getInfo<RISCV64FunctionInfo>();
  RISCV64ABIInfo ABI = STI.getABI();
  unsigned FP = ABI.GetFramePtr();
  unsigned BP = ABI.IsN64() ? RISCV64::S7_64 : RISCV64::S7;

  // Mark $fp as used if function has dedicated frame pointer.
  if (hasFP(MF))
    setAliasRegs(MF, SavedRegs, FP);
  // Mark $s7 as used if function has dedicated base pointer.
  if (hasBP(MF))
    setAliasRegs(MF, SavedRegs, BP);

  // Create spill slots for eh data registers if function calls eh_return.
  if (RISCV64FI->callsEhReturn())
    RISCV64FI->createEhDataRegsFI();

  // Create spill slots for Coprocessor 0 registers if function is an ISR.
  if (RISCV64FI->isISR())
    RISCV64FI->createISRRegFI();

  // Expand pseudo instructions which load, store or copy accumulators.
  // Add an emergency spill slot if a pseudo was expanded.
  if (ExpandPseudo(MF).expand()) {
    // The spill slot should be half the size of the accumulator. If target have
    // general-purpose registers 64 bits wide, it should be 64-bit, otherwise
    // it should be 32-bit.
    const TargetRegisterClass &RC = STI.isGP64bit() ?
      RISCV64::GPR64RegClass : RISCV64::GPR32RegClass;
    int FI = MF.getFrameInfo().CreateStackObject(TRI->getSpillSize(RC),
                                                 TRI->getSpillAlignment(RC),
                                                 false);
    RS->addScavengingFrameIndex(FI);
  }

  // Set scavenging frame index if necessary.
  uint64_t MaxSPOffset = estimateStackSize(MF);

  // MSA has a minimum offset of 10 bits signed. If there is a variable
  // sized object on the stack, the estimation cannot account for it.
  if (isIntN(STI.hasMSA() ? 10 : 16, MaxSPOffset) &&
      !MF.getFrameInfo().hasVarSizedObjects())
    return;

  const TargetRegisterClass &RC =
      ABI.ArePtrs64bit() ? RISCV64::GPR64RegClass : RISCV64::GPR32RegClass;
  int FI = MF.getFrameInfo().CreateStackObject(TRI->getSpillSize(RC),
                                               TRI->getSpillAlignment(RC),
                                               false);
  RS->addScavengingFrameIndex(FI);
}

const RISCV64FrameLowering *
llvm::createRISCV64SEFrameLowering(const RISCV64Subtarget &ST) {
  return new RISCV64SEFrameLowering(ST);
}
