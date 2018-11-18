//===-- RISCV64SEInstrInfo.cpp - RISCV6432/64 Instruction Information -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RISCV6432/64 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "RISCV64SEInstrInfo.h"
#include "InstPrinter/RISCV64InstPrinter.h"
#include "RISCV64AnalyzeImmediate.h"
#include "RISCV64MachineFunction.h"
#include "RISCV64TargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

static unsigned getUnconditionalBranch(const RISCV64Subtarget &STI) {
  if (STI.inMicroRISCV64Mode())
    return STI.isPositionIndependent() ? RISCV64::B_MM : RISCV64::J_MM;
  return STI.isPositionIndependent() ? RISCV64::B : RISCV64::J;
}

RISCV64SEInstrInfo::RISCV64SEInstrInfo(const RISCV64Subtarget &STI)
    : RISCV64InstrInfo(STI, getUnconditionalBranch(STI)), RI() {}

const RISCV64RegisterInfo &RISCV64SEInstrInfo::getRegisterInfo() const {
  return RI;
}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned RISCV64SEInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                              int &FrameIndex) const {
  unsigned Opc = MI.getOpcode();

  if ((Opc == RISCV64::LW)   || (Opc == RISCV64::LD)   ||
      (Opc == RISCV64::LWC1) || (Opc == RISCV64::LDC1) || (Opc == RISCV64::LDC164)) {
    if ((MI.getOperand(1).isFI()) &&  // is a stack slot
        (MI.getOperand(2).isImm()) && // the imm is zero
        (isZeroImm(MI.getOperand(2)))) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }

  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned RISCV64SEInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  unsigned Opc = MI.getOpcode();

  if ((Opc == RISCV64::SW)   || (Opc == RISCV64::SD)   ||
      (Opc == RISCV64::SWC1) || (Opc == RISCV64::SDC1) || (Opc == RISCV64::SDC164)) {
    if ((MI.getOperand(1).isFI()) &&  // is a stack slot
        (MI.getOperand(2).isImm()) && // the imm is zero
        (isZeroImm(MI.getOperand(2)))) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }
  return 0;
}

void RISCV64SEInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  const DebugLoc &DL, unsigned DestReg,
                                  unsigned SrcReg, bool KillSrc) const {
  unsigned Opc = 0, ZeroReg = 0;
  bool isMicroRISCV64 = Subtarget.inMicroRISCV64Mode();

  if (RISCV64::GPR32RegClass.contains(DestReg)) { // Copy to CPU Reg.
    if (RISCV64::GPR32RegClass.contains(SrcReg)) {
      if (isMicroRISCV64)
        Opc = RISCV64::MOVE16_MM;
      else
        Opc = RISCV64::OR, ZeroReg = RISCV64::ZERO;
    } else if (RISCV64::CCRRegClass.contains(SrcReg))
      Opc = RISCV64::CFC1;
    else if (RISCV64::FGR32RegClass.contains(SrcReg))
      Opc = RISCV64::MFC1;
    else if (RISCV64::HI32RegClass.contains(SrcReg)) {
      Opc = isMicroRISCV64 ? RISCV64::MFHI16_MM : RISCV64::MFHI;
      SrcReg = 0;
    } else if (RISCV64::LO32RegClass.contains(SrcReg)) {
      Opc = isMicroRISCV64 ? RISCV64::MFLO16_MM : RISCV64::MFLO;
      SrcReg = 0;
    } else if (RISCV64::HI32DSPRegClass.contains(SrcReg))
      Opc = RISCV64::MFHI_DSP;
    else if (RISCV64::LO32DSPRegClass.contains(SrcReg))
      Opc = RISCV64::MFLO_DSP;
    else if (RISCV64::DSPCCRegClass.contains(SrcReg)) {
      BuildMI(MBB, I, DL, get(RISCV64::RDDSP), DestReg).addImm(1 << 4)
        .addReg(SrcReg, RegState::Implicit | getKillRegState(KillSrc));
      return;
    }
    else if (RISCV64::MSACtrlRegClass.contains(SrcReg))
      Opc = RISCV64::CFCMSA;
  }
  else if (RISCV64::GPR32RegClass.contains(SrcReg)) { // Copy from CPU Reg.
    if (RISCV64::CCRRegClass.contains(DestReg))
      Opc = RISCV64::CTC1;
    else if (RISCV64::FGR32RegClass.contains(DestReg))
      Opc = RISCV64::MTC1;
    else if (RISCV64::HI32RegClass.contains(DestReg))
      Opc = RISCV64::MTHI, DestReg = 0;
    else if (RISCV64::LO32RegClass.contains(DestReg))
      Opc = RISCV64::MTLO, DestReg = 0;
    else if (RISCV64::HI32DSPRegClass.contains(DestReg))
      Opc = RISCV64::MTHI_DSP;
    else if (RISCV64::LO32DSPRegClass.contains(DestReg))
      Opc = RISCV64::MTLO_DSP;
    else if (RISCV64::DSPCCRegClass.contains(DestReg)) {
      BuildMI(MBB, I, DL, get(RISCV64::WRDSP))
        .addReg(SrcReg, getKillRegState(KillSrc)).addImm(1 << 4)
        .addReg(DestReg, RegState::ImplicitDefine);
      return;
    } else if (RISCV64::MSACtrlRegClass.contains(DestReg)) {
      BuildMI(MBB, I, DL, get(RISCV64::CTCMSA))
          .addReg(DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
      return;
    }
  }
  else if (RISCV64::FGR32RegClass.contains(DestReg, SrcReg))
    Opc = RISCV64::FMOV_S;
  else if (RISCV64::AFGR64RegClass.contains(DestReg, SrcReg))
    Opc = RISCV64::FMOV_D32;
  else if (RISCV64::FGR64RegClass.contains(DestReg, SrcReg))
    Opc = RISCV64::FMOV_D64;
  else if (RISCV64::GPR64RegClass.contains(DestReg)) { // Copy to CPU64 Reg.
    if (RISCV64::GPR64RegClass.contains(SrcReg))
      Opc = RISCV64::OR64, ZeroReg = RISCV64::ZERO_64;
    else if (RISCV64::HI64RegClass.contains(SrcReg))
      Opc = RISCV64::MFHI64, SrcReg = 0;
    else if (RISCV64::LO64RegClass.contains(SrcReg))
      Opc = RISCV64::MFLO64, SrcReg = 0;
    else if (RISCV64::FGR64RegClass.contains(SrcReg))
      Opc = RISCV64::DMFC1;
  }
  else if (RISCV64::GPR64RegClass.contains(SrcReg)) { // Copy from CPU64 Reg.
    if (RISCV64::HI64RegClass.contains(DestReg))
      Opc = RISCV64::MTHI64, DestReg = 0;
    else if (RISCV64::LO64RegClass.contains(DestReg))
      Opc = RISCV64::MTLO64, DestReg = 0;
    else if (RISCV64::FGR64RegClass.contains(DestReg))
      Opc = RISCV64::DMTC1;
  }
  else if (RISCV64::MSA128BRegClass.contains(DestReg)) { // Copy to MSA reg
    if (RISCV64::MSA128BRegClass.contains(SrcReg))
      Opc = RISCV64::MOVE_V;
  }

  assert(Opc && "Cannot copy registers");

  MachineInstrBuilder MIB = BuildMI(MBB, I, DL, get(Opc));

  if (DestReg)
    MIB.addReg(DestReg, RegState::Define);

  if (SrcReg)
    MIB.addReg(SrcReg, getKillRegState(KillSrc));

  if (ZeroReg)
    MIB.addReg(ZeroReg);
}

static bool isORCopyInst(const MachineInstr &MI) {
  switch (MI.getOpcode()) {
  default:
    break;
  case RISCV64::OR_MM:
  case RISCV64::OR:
    if (MI.getOperand(2).getReg() == RISCV64::ZERO)
      return true;
    break;
  case RISCV64::OR64:
    if (MI.getOperand(2).getReg() == RISCV64::ZERO_64)
      return true;
    break;
  }
  return false;
}

/// If @MI is WRDSP/RRDSP instruction return true with @isWrite set to true
/// if it is WRDSP instruction.
static bool isReadOrWriteToDSPReg(const MachineInstr &MI, bool &isWrite) {
  switch (MI.getOpcode()) {
  default:
   return false;
  case RISCV64::WRDSP:
  case RISCV64::WRDSP_MM:
    isWrite = true;
    break;
  case RISCV64::RDDSP:
  case RISCV64::RDDSP_MM:
    isWrite = false;
    break;
  }
  return true;
}

/// We check for the common case of 'or', as it's MIPS' preferred instruction
/// for GPRs but we have to check the operands to ensure that is the case.
/// Other move instructions for MIPS are directly identifiable.
bool RISCV64SEInstrInfo::isCopyInstrImpl(const MachineInstr &MI,
                                      const MachineOperand *&Src,
                                      const MachineOperand *&Dest) const {
  bool isDSPControlWrite = false;
  // Condition is made to match the creation of WRDSP/RDDSP copy instruction
  // from copyPhysReg function.
  if (isReadOrWriteToDSPReg(MI, isDSPControlWrite)) {
    if (!MI.getOperand(1).isImm() || MI.getOperand(1).getImm() != (1<<4))
      return false;
    else if (isDSPControlWrite) {
      Src = &MI.getOperand(0);
      Dest = &MI.getOperand(2);
    } else {
      Dest = &MI.getOperand(0);
      Src = &MI.getOperand(2);
    }
    return true;
  } else if (MI.isMoveReg() || isORCopyInst(MI)) {
    Dest = &MI.getOperand(0);
    Src = &MI.getOperand(1);
    return true;
  }
  return false;
}

void RISCV64SEInstrInfo::
storeRegToStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                unsigned SrcReg, bool isKill, int FI,
                const TargetRegisterClass *RC, const TargetRegisterInfo *TRI,
                int64_t Offset) const {
  DebugLoc DL;
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);

  unsigned Opc = 0;

  if (RISCV64::GPR32RegClass.hasSubClassEq(RC))
    Opc = RISCV64::SW;
  else if (RISCV64::GPR64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::SD;
  else if (RISCV64::ACC64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::STORE_ACC64;
  else if (RISCV64::ACC64DSPRegClass.hasSubClassEq(RC))
    Opc = RISCV64::STORE_ACC64DSP;
  else if (RISCV64::ACC128RegClass.hasSubClassEq(RC))
    Opc = RISCV64::STORE_ACC128;
  else if (RISCV64::DSPCCRegClass.hasSubClassEq(RC))
    Opc = RISCV64::STORE_CCOND_DSP;
  else if (RISCV64::FGR32RegClass.hasSubClassEq(RC))
    Opc = RISCV64::SWC1;
  else if (RISCV64::AFGR64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::SDC1;
  else if (RISCV64::FGR64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::SDC164;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v16i8))
    Opc = RISCV64::ST_B;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v8i16) ||
           TRI->isTypeLegalForClass(*RC, MVT::v8f16))
    Opc = RISCV64::ST_H;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v4i32) ||
           TRI->isTypeLegalForClass(*RC, MVT::v4f32))
    Opc = RISCV64::ST_W;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v2i64) ||
           TRI->isTypeLegalForClass(*RC, MVT::v2f64))
    Opc = RISCV64::ST_D;
  else if (RISCV64::LO32RegClass.hasSubClassEq(RC))
    Opc = RISCV64::SW;
  else if (RISCV64::LO64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::SD;
  else if (RISCV64::HI32RegClass.hasSubClassEq(RC))
    Opc = RISCV64::SW;
  else if (RISCV64::HI64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::SD;
  else if (RISCV64::DSPRRegClass.hasSubClassEq(RC))
    Opc = RISCV64::SWDSP;

  // Hi, Lo are normally caller save but they are callee save
  // for interrupt handling.
  const Function &Func = MBB.getParent()->getFunction();
  if (Func.hasFnAttribute("interrupt")) {
    if (RISCV64::HI32RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(RISCV64::MFHI), RISCV64::K0);
      SrcReg = RISCV64::K0;
    } else if (RISCV64::HI64RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(RISCV64::MFHI64), RISCV64::K0_64);
      SrcReg = RISCV64::K0_64;
    } else if (RISCV64::LO32RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(RISCV64::MFLO), RISCV64::K0);
      SrcReg = RISCV64::K0;
    } else if (RISCV64::LO64RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(RISCV64::MFLO64), RISCV64::K0_64);
      SrcReg = RISCV64::K0_64;
    }
  }

  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill))
    .addFrameIndex(FI).addImm(Offset).addMemOperand(MMO);
}

void RISCV64SEInstrInfo::
loadRegFromStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                 unsigned DestReg, int FI, const TargetRegisterClass *RC,
                 const TargetRegisterInfo *TRI, int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
  unsigned Opc = 0;

  const Function &Func = MBB.getParent()->getFunction();
  bool ReqIndirectLoad = Func.hasFnAttribute("interrupt") &&
                         (DestReg == RISCV64::LO0 || DestReg == RISCV64::LO0_64 ||
                          DestReg == RISCV64::HI0 || DestReg == RISCV64::HI0_64);

  if (RISCV64::GPR32RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LW;
  else if (RISCV64::GPR64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LD;
  else if (RISCV64::ACC64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LOAD_ACC64;
  else if (RISCV64::ACC64DSPRegClass.hasSubClassEq(RC))
    Opc = RISCV64::LOAD_ACC64DSP;
  else if (RISCV64::ACC128RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LOAD_ACC128;
  else if (RISCV64::DSPCCRegClass.hasSubClassEq(RC))
    Opc = RISCV64::LOAD_CCOND_DSP;
  else if (RISCV64::FGR32RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LWC1;
  else if (RISCV64::AFGR64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LDC1;
  else if (RISCV64::FGR64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LDC164;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v16i8))
    Opc = RISCV64::LD_B;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v8i16) ||
           TRI->isTypeLegalForClass(*RC, MVT::v8f16))
    Opc = RISCV64::LD_H;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v4i32) ||
           TRI->isTypeLegalForClass(*RC, MVT::v4f32))
    Opc = RISCV64::LD_W;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v2i64) ||
           TRI->isTypeLegalForClass(*RC, MVT::v2f64))
    Opc = RISCV64::LD_D;
  else if (RISCV64::HI32RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LW;
  else if (RISCV64::HI64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LD;
  else if (RISCV64::LO32RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LW;
  else if (RISCV64::LO64RegClass.hasSubClassEq(RC))
    Opc = RISCV64::LD;
  else if (RISCV64::DSPRRegClass.hasSubClassEq(RC))
    Opc = RISCV64::LWDSP;

  assert(Opc && "Register class not handled!");

  if (!ReqIndirectLoad)
    BuildMI(MBB, I, DL, get(Opc), DestReg)
        .addFrameIndex(FI)
        .addImm(Offset)
        .addMemOperand(MMO);
  else {
    // Load HI/LO through K0. Notably the DestReg is encoded into the
    // instruction itself.
    unsigned Reg = RISCV64::K0;
    unsigned LdOp = RISCV64::MTLO;
    if (DestReg == RISCV64::HI0)
      LdOp = RISCV64::MTHI;

    if (Subtarget.getABI().ArePtrs64bit()) {
      Reg = RISCV64::K0_64;
      if (DestReg == RISCV64::HI0_64)
        LdOp = RISCV64::MTHI64;
      else
        LdOp = RISCV64::MTLO64;
    }

    BuildMI(MBB, I, DL, get(Opc), Reg)
        .addFrameIndex(FI)
        .addImm(Offset)
        .addMemOperand(MMO);
    BuildMI(MBB, I, DL, get(LdOp)).addReg(Reg);
  }
}

bool RISCV64SEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();
  bool isMicroRISCV64 = Subtarget.inMicroRISCV64Mode();
  unsigned Opc;

  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case RISCV64::RetRA:
    expandRetRA(MBB, MI);
    break;
  case RISCV64::ERet:
    expandERet(MBB, MI);
    break;
  case RISCV64::PseudoMFHI:
    expandPseudoMFHiLo(MBB, MI, RISCV64::MFHI);
    break;
  case RISCV64::PseudoMFHI_MM:
    expandPseudoMFHiLo(MBB, MI, RISCV64::MFHI16_MM);
    break;
  case RISCV64::PseudoMFLO:
    expandPseudoMFHiLo(MBB, MI, RISCV64::MFLO);
    break;
  case RISCV64::PseudoMFLO_MM:
    expandPseudoMFHiLo(MBB, MI, RISCV64::MFLO16_MM);
    break;
  case RISCV64::PseudoMFHI64:
    expandPseudoMFHiLo(MBB, MI, RISCV64::MFHI64);
    break;
  case RISCV64::PseudoMFLO64:
    expandPseudoMFHiLo(MBB, MI, RISCV64::MFLO64);
    break;
  case RISCV64::PseudoMTLOHI:
    expandPseudoMTLoHi(MBB, MI, RISCV64::MTLO, RISCV64::MTHI, false);
    break;
  case RISCV64::PseudoMTLOHI64:
    expandPseudoMTLoHi(MBB, MI, RISCV64::MTLO64, RISCV64::MTHI64, false);
    break;
  case RISCV64::PseudoMTLOHI_DSP:
    expandPseudoMTLoHi(MBB, MI, RISCV64::MTLO_DSP, RISCV64::MTHI_DSP, true);
    break;
  case RISCV64::PseudoCVT_S_W:
    expandCvtFPInt(MBB, MI, RISCV64::CVT_S_W, RISCV64::MTC1, false);
    break;
  case RISCV64::PseudoCVT_D32_W:
    Opc = isMicroRISCV64 ? RISCV64::CVT_D32_W_MM : RISCV64::CVT_D32_W;
    expandCvtFPInt(MBB, MI, Opc, RISCV64::MTC1, false);
    break;
  case RISCV64::PseudoCVT_S_L:
    expandCvtFPInt(MBB, MI, RISCV64::CVT_S_L, RISCV64::DMTC1, true);
    break;
  case RISCV64::PseudoCVT_D64_W:
    Opc = isMicroRISCV64 ? RISCV64::CVT_D64_W_MM : RISCV64::CVT_D64_W;
    expandCvtFPInt(MBB, MI, Opc, RISCV64::MTC1, true);
    break;
  case RISCV64::PseudoCVT_D64_L:
    expandCvtFPInt(MBB, MI, RISCV64::CVT_D64_L, RISCV64::DMTC1, true);
    break;
  case RISCV64::BuildPairF64:
    expandBuildPairF64(MBB, MI, isMicroRISCV64, false);
    break;
  case RISCV64::BuildPairF64_64:
    expandBuildPairF64(MBB, MI, isMicroRISCV64, true);
    break;
  case RISCV64::ExtractElementF64:
    expandExtractElementF64(MBB, MI, isMicroRISCV64, false);
    break;
  case RISCV64::ExtractElementF64_64:
    expandExtractElementF64(MBB, MI, isMicroRISCV64, true);
    break;
  case RISCV64::MIPSeh_return32:
  case RISCV64::MIPSeh_return64:
    expandEhReturn(MBB, MI);
    break;
  }

  MBB.erase(MI);
  return true;
}

/// getOppositeBranchOpc - Return the inverse of the specified
/// opcode, e.g. turning BEQ to BNE.
unsigned RISCV64SEInstrInfo::getOppositeBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:           llvm_unreachable("Illegal opcode!");
  case RISCV64::BEQ:    return RISCV64::BNE;
  case RISCV64::BEQ_MM: return RISCV64::BNE_MM;
  case RISCV64::BNE:    return RISCV64::BEQ;
  case RISCV64::BNE_MM: return RISCV64::BEQ_MM;
  case RISCV64::BGTZ:   return RISCV64::BLEZ;
  case RISCV64::BGEZ:   return RISCV64::BLTZ;
  case RISCV64::BLTZ:   return RISCV64::BGEZ;
  case RISCV64::BLEZ:   return RISCV64::BGTZ;
  case RISCV64::BGTZ_MM:   return RISCV64::BLEZ_MM;
  case RISCV64::BGEZ_MM:   return RISCV64::BLTZ_MM;
  case RISCV64::BLTZ_MM:   return RISCV64::BGEZ_MM;
  case RISCV64::BLEZ_MM:   return RISCV64::BGTZ_MM;
  case RISCV64::BEQ64:  return RISCV64::BNE64;
  case RISCV64::BNE64:  return RISCV64::BEQ64;
  case RISCV64::BGTZ64: return RISCV64::BLEZ64;
  case RISCV64::BGEZ64: return RISCV64::BLTZ64;
  case RISCV64::BLTZ64: return RISCV64::BGEZ64;
  case RISCV64::BLEZ64: return RISCV64::BGTZ64;
  case RISCV64::BC1T:   return RISCV64::BC1F;
  case RISCV64::BC1F:   return RISCV64::BC1T;
  case RISCV64::BC1T_MM:   return RISCV64::BC1F_MM;
  case RISCV64::BC1F_MM:   return RISCV64::BC1T_MM;
  case RISCV64::BEQZ16_MM: return RISCV64::BNEZ16_MM;
  case RISCV64::BNEZ16_MM: return RISCV64::BEQZ16_MM;
  case RISCV64::BEQZC_MM:  return RISCV64::BNEZC_MM;
  case RISCV64::BNEZC_MM:  return RISCV64::BEQZC_MM;
  case RISCV64::BEQZC:  return RISCV64::BNEZC;
  case RISCV64::BNEZC:  return RISCV64::BEQZC;
  case RISCV64::BLEZC:  return RISCV64::BGTZC;
  case RISCV64::BGEZC:  return RISCV64::BLTZC;
  case RISCV64::BGEC:   return RISCV64::BLTC;
  case RISCV64::BGTZC:  return RISCV64::BLEZC;
  case RISCV64::BLTZC:  return RISCV64::BGEZC;
  case RISCV64::BLTC:   return RISCV64::BGEC;
  case RISCV64::BGEUC:  return RISCV64::BLTUC;
  case RISCV64::BLTUC:  return RISCV64::BGEUC;
  case RISCV64::BEQC:   return RISCV64::BNEC;
  case RISCV64::BNEC:   return RISCV64::BEQC;
  case RISCV64::BC1EQZ: return RISCV64::BC1NEZ;
  case RISCV64::BC1NEZ: return RISCV64::BC1EQZ;
  case RISCV64::BEQZC_MMR6:  return RISCV64::BNEZC_MMR6;
  case RISCV64::BNEZC_MMR6:  return RISCV64::BEQZC_MMR6;
  case RISCV64::BLEZC_MMR6:  return RISCV64::BGTZC_MMR6;
  case RISCV64::BGEZC_MMR6:  return RISCV64::BLTZC_MMR6;
  case RISCV64::BGEC_MMR6:   return RISCV64::BLTC_MMR6;
  case RISCV64::BGTZC_MMR6:  return RISCV64::BLEZC_MMR6;
  case RISCV64::BLTZC_MMR6:  return RISCV64::BGEZC_MMR6;
  case RISCV64::BLTC_MMR6:   return RISCV64::BGEC_MMR6;
  case RISCV64::BGEUC_MMR6:  return RISCV64::BLTUC_MMR6;
  case RISCV64::BLTUC_MMR6:  return RISCV64::BGEUC_MMR6;
  case RISCV64::BEQC_MMR6:   return RISCV64::BNEC_MMR6;
  case RISCV64::BNEC_MMR6:   return RISCV64::BEQC_MMR6;
  case RISCV64::BC1EQZC_MMR6: return RISCV64::BC1NEZC_MMR6;
  case RISCV64::BC1NEZC_MMR6: return RISCV64::BC1EQZC_MMR6;
  case RISCV64::BEQZC64:  return RISCV64::BNEZC64;
  case RISCV64::BNEZC64:  return RISCV64::BEQZC64;
  case RISCV64::BEQC64:   return RISCV64::BNEC64;
  case RISCV64::BNEC64:   return RISCV64::BEQC64;
  case RISCV64::BGEC64:   return RISCV64::BLTC64;
  case RISCV64::BGEUC64:  return RISCV64::BLTUC64;
  case RISCV64::BLTC64:   return RISCV64::BGEC64;
  case RISCV64::BLTUC64:  return RISCV64::BGEUC64;
  case RISCV64::BGTZC64:  return RISCV64::BLEZC64;
  case RISCV64::BGEZC64:  return RISCV64::BLTZC64;
  case RISCV64::BLTZC64:  return RISCV64::BGEZC64;
  case RISCV64::BLEZC64:  return RISCV64::BGTZC64;
  case RISCV64::BBIT0:  return RISCV64::BBIT1;
  case RISCV64::BBIT1:  return RISCV64::BBIT0;
  case RISCV64::BBIT032:  return RISCV64::BBIT132;
  case RISCV64::BBIT132:  return RISCV64::BBIT032;
  case RISCV64::BZ_B:   return RISCV64::BNZ_B;
  case RISCV64::BZ_H:   return RISCV64::BNZ_H;
  case RISCV64::BZ_W:   return RISCV64::BNZ_W;
  case RISCV64::BZ_D:   return RISCV64::BNZ_D;
  case RISCV64::BZ_V:   return RISCV64::BNZ_V;
  case RISCV64::BNZ_B:  return RISCV64::BZ_B;
  case RISCV64::BNZ_H:  return RISCV64::BZ_H;
  case RISCV64::BNZ_W:  return RISCV64::BZ_W;
  case RISCV64::BNZ_D:  return RISCV64::BZ_D;
  case RISCV64::BNZ_V:  return RISCV64::BZ_V;
  }
}

/// Adjust SP by Amount bytes.
void RISCV64SEInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  RISCV64ABIInfo ABI = Subtarget.getABI();
  DebugLoc DL;
  unsigned ADDiu = ABI.GetPtrAddiuOp();

  if (Amount == 0)
    return;

  if (isInt<16>(Amount)) {
    // addi sp, sp, amount
    BuildMI(MBB, I, DL, get(ADDiu), SP).addReg(SP).addImm(Amount);
  } else {
    // For numbers which are not 16bit integers we synthesize Amount inline
    // then add or subtract it from sp.
    unsigned Opc = ABI.GetPtrAdduOp();
    if (Amount < 0) {
      Opc = ABI.GetPtrSubuOp();
      Amount = -Amount;
    }
    unsigned Reg = loadImmediate(Amount, MBB, I, DL, nullptr);
    BuildMI(MBB, I, DL, get(Opc), SP).addReg(SP).addReg(Reg, RegState::Kill);
  }
}

/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
unsigned RISCV64SEInstrInfo::loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator II,
                                        const DebugLoc &DL,
                                        unsigned *NewImm) const {
  RISCV64AnalyzeImmediate AnalyzeImm;
  const RISCV64Subtarget &STI = Subtarget;
  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  unsigned Size = STI.isABI_N64() ? 64 : 32;
  unsigned LUi = STI.isABI_N64() ? RISCV64::LUi64 : RISCV64::LUi;
  unsigned ZEROReg = STI.isABI_N64() ? RISCV64::ZERO_64 : RISCV64::ZERO;
  const TargetRegisterClass *RC = STI.isABI_N64() ?
    &RISCV64::GPR64RegClass : &RISCV64::GPR32RegClass;
  bool LastInstrIsADDiu = NewImm;

  const RISCV64AnalyzeImmediate::InstSeq &Seq =
    AnalyzeImm.Analyze(Imm, Size, LastInstrIsADDiu);
  RISCV64AnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();

  assert(Seq.size() && (!LastInstrIsADDiu || (Seq.size() > 1)));

  // The first instruction can be a LUi, which is different from other
  // instructions (ADDiu, ORI and SLL) in that it does not have a register
  // operand.
  unsigned Reg = RegInfo.createVirtualRegister(RC);

  if (Inst->Opc == LUi)
    BuildMI(MBB, II, DL, get(LUi), Reg).addImm(SignExtend64<16>(Inst->ImmOpnd));
  else
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(ZEROReg)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  // Build the remaining instructions in Seq.
  for (++Inst; Inst != Seq.end() - LastInstrIsADDiu; ++Inst)
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(Reg, RegState::Kill)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  if (LastInstrIsADDiu)
    *NewImm = Inst->ImmOpnd;

  return Reg;
}

unsigned RISCV64SEInstrInfo::getAnalyzableBrOpc(unsigned Opc) const {
  return (Opc == RISCV64::BEQ    || Opc == RISCV64::BEQ_MM || Opc == RISCV64::BNE    ||
          Opc == RISCV64::BNE_MM || Opc == RISCV64::BGTZ   || Opc == RISCV64::BGEZ   ||
          Opc == RISCV64::BLTZ   || Opc == RISCV64::BLEZ   || Opc == RISCV64::BEQ64  ||
          Opc == RISCV64::BNE64  || Opc == RISCV64::BGTZ64 || Opc == RISCV64::BGEZ64 ||
          Opc == RISCV64::BLTZ64 || Opc == RISCV64::BLEZ64 || Opc == RISCV64::BC1T   ||
          Opc == RISCV64::BC1F   || Opc == RISCV64::B      || Opc == RISCV64::J      ||
          Opc == RISCV64::J_MM   || Opc == RISCV64::B_MM   || Opc == RISCV64::BEQZC_MM ||
          Opc == RISCV64::BNEZC_MM || Opc == RISCV64::BEQC || Opc == RISCV64::BNEC   ||
          Opc == RISCV64::BLTC   || Opc == RISCV64::BGEC   || Opc == RISCV64::BLTUC  ||
          Opc == RISCV64::BGEUC  || Opc == RISCV64::BGTZC  || Opc == RISCV64::BLEZC  ||
          Opc == RISCV64::BGEZC  || Opc == RISCV64::BLTZC  || Opc == RISCV64::BEQZC  ||
          Opc == RISCV64::BNEZC  || Opc == RISCV64::BEQZC64 || Opc == RISCV64::BNEZC64 ||
          Opc == RISCV64::BEQC64 || Opc == RISCV64::BNEC64 || Opc == RISCV64::BGEC64 ||
          Opc == RISCV64::BGEUC64 || Opc == RISCV64::BLTC64 || Opc == RISCV64::BLTUC64 ||
          Opc == RISCV64::BGTZC64 || Opc == RISCV64::BGEZC64 ||
          Opc == RISCV64::BLTZC64 || Opc == RISCV64::BLEZC64 || Opc == RISCV64::BC ||
          Opc == RISCV64::BBIT0 || Opc == RISCV64::BBIT1 || Opc == RISCV64::BBIT032 ||
          Opc == RISCV64::BBIT132 ||  Opc == RISCV64::BC_MMR6 ||
          Opc == RISCV64::BEQC_MMR6 || Opc == RISCV64::BNEC_MMR6 ||
          Opc == RISCV64::BLTC_MMR6 || Opc == RISCV64::BGEC_MMR6 ||
          Opc == RISCV64::BLTUC_MMR6 || Opc == RISCV64::BGEUC_MMR6 ||
          Opc == RISCV64::BGTZC_MMR6 || Opc == RISCV64::BLEZC_MMR6 ||
          Opc == RISCV64::BGEZC_MMR6 || Opc == RISCV64::BLTZC_MMR6 ||
          Opc == RISCV64::BEQZC_MMR6 || Opc == RISCV64::BNEZC_MMR6) ? Opc : 0;
}

void RISCV64SEInstrInfo::expandRetRA(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I) const {

  MachineInstrBuilder MIB;
  if (Subtarget.isGP64bit())
    MIB = BuildMI(MBB, I, I->getDebugLoc(), get(RISCV64::PseudoReturn64))
              .addReg(RISCV64::RA_64, RegState::Undef);
  else
    MIB = BuildMI(MBB, I, I->getDebugLoc(), get(RISCV64::PseudoReturn))
              .addReg(RISCV64::RA, RegState::Undef);

  // Retain any imp-use flags.
  for (auto & MO : I->operands()) {
    if (MO.isImplicit())
      MIB.add(MO);
  }
}

void RISCV64SEInstrInfo::expandERet(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(RISCV64::ERET));
}

std::pair<bool, bool>
RISCV64SEInstrInfo::compareOpndSize(unsigned Opc,
                                 const MachineFunction &MF) const {
  const MCInstrDesc &Desc = get(Opc);
  assert(Desc.NumOperands == 2 && "Unary instruction expected.");
  const RISCV64RegisterInfo *RI = &getRegisterInfo();
  unsigned DstRegSize = RI->getRegSizeInBits(*getRegClass(Desc, 0, RI, MF));
  unsigned SrcRegSize = RI->getRegSizeInBits(*getRegClass(Desc, 1, RI, MF));

  return std::make_pair(DstRegSize > SrcRegSize, DstRegSize < SrcRegSize);
}

void RISCV64SEInstrInfo::expandPseudoMFHiLo(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned NewOpc) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(NewOpc), I->getOperand(0).getReg());
}

void RISCV64SEInstrInfo::expandPseudoMTLoHi(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned LoOpc,
                                         unsigned HiOpc,
                                         bool HasExplicitDef) const {
  // Expand
  //  lo_hi pseudomtlohi $gpr0, $gpr1
  // to these two instructions:
  //  mtlo $gpr0
  //  mthi $gpr1

  DebugLoc DL = I->getDebugLoc();
  const MachineOperand &SrcLo = I->getOperand(1), &SrcHi = I->getOperand(2);
  MachineInstrBuilder LoInst = BuildMI(MBB, I, DL, get(LoOpc));
  MachineInstrBuilder HiInst = BuildMI(MBB, I, DL, get(HiOpc));

  // Add lo/hi registers if the mtlo/hi instructions created have explicit
  // def registers.
  if (HasExplicitDef) {
    unsigned DstReg = I->getOperand(0).getReg();
    unsigned DstLo = getRegisterInfo().getSubReg(DstReg, RISCV64::sub_lo);
    unsigned DstHi = getRegisterInfo().getSubReg(DstReg, RISCV64::sub_hi);
    LoInst.addReg(DstLo, RegState::Define);
    HiInst.addReg(DstHi, RegState::Define);
  }

  LoInst.addReg(SrcLo.getReg(), getKillRegState(SrcLo.isKill()));
  HiInst.addReg(SrcHi.getReg(), getKillRegState(SrcHi.isKill()));
}

void RISCV64SEInstrInfo::expandCvtFPInt(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I,
                                     unsigned CvtOpc, unsigned MovOpc,
                                     bool IsI64) const {
  const MCInstrDesc &CvtDesc = get(CvtOpc), &MovDesc = get(MovOpc);
  const MachineOperand &Dst = I->getOperand(0), &Src = I->getOperand(1);
  unsigned DstReg = Dst.getReg(), SrcReg = Src.getReg(), TmpReg = DstReg;
  unsigned KillSrc =  getKillRegState(Src.isKill());
  DebugLoc DL = I->getDebugLoc();
  bool DstIsLarger, SrcIsLarger;

  std::tie(DstIsLarger, SrcIsLarger) =
      compareOpndSize(CvtOpc, *MBB.getParent());

  if (DstIsLarger)
    TmpReg = getRegisterInfo().getSubReg(DstReg, RISCV64::sub_lo);

  if (SrcIsLarger)
    DstReg = getRegisterInfo().getSubReg(DstReg, RISCV64::sub_lo);

  BuildMI(MBB, I, DL, MovDesc, TmpReg).addReg(SrcReg, KillSrc);
  BuildMI(MBB, I, DL, CvtDesc, DstReg).addReg(TmpReg, RegState::Kill);
}

void RISCV64SEInstrInfo::expandExtractElementF64(MachineBasicBlock &MBB,
                                              MachineBasicBlock::iterator I,
                                              bool isMicroRISCV64,
                                              bool FP64) const {
  unsigned DstReg = I->getOperand(0).getReg();
  unsigned SrcReg = I->getOperand(1).getReg();
  unsigned N = I->getOperand(2).getImm();
  DebugLoc dl = I->getDebugLoc();

  assert(N < 2 && "Invalid immediate");
  unsigned SubIdx = N ? RISCV64::sub_hi : RISCV64::sub_lo;
  unsigned SubReg = getRegisterInfo().getSubReg(SrcReg, SubIdx);

  // FPXX on MIPS-II or MIPS32r1 should have been handled with a spill/reload
  // in RISCV64SEFrameLowering.cpp.
  assert(!(Subtarget.isABI_FPXX() && !Subtarget.hasRISCV6432r2()));

  // FP64A (FP64 with nooddspreg) should have been handled with a spill/reload
  // in RISCV64SEFrameLowering.cpp.
  assert(!(Subtarget.isFP64bit() && !Subtarget.useOddSPReg()));

  if (SubIdx == RISCV64::sub_hi && Subtarget.hasMTHC1()) {
    // FIXME: Strictly speaking MFHC1 only reads the top 32-bits however, we
    //        claim to read the whole 64-bits as part of a white lie used to
    //        temporarily work around a widespread bug in the -mfp64 support.
    //        The problem is that none of the 32-bit fpu ops mention the fact
    //        that they clobber the upper 32-bits of the 64-bit FPR. Fixing that
    //        requires a major overhaul of the FPU implementation which can't
    //        be done right now due to time constraints.
    //        MFHC1 is one of two instructions that are affected since they are
    //        the only instructions that don't read the lower 32-bits.
    //        We therefore pretend that it reads the bottom 32-bits to
    //        artificially create a dependency and prevent the scheduler
    //        changing the behaviour of the code.
    BuildMI(MBB, I, dl,
            get(isMicroRISCV64 ? (FP64 ? RISCV64::MFHC1_D64_MM : RISCV64::MFHC1_D32_MM)
                            : (FP64 ? RISCV64::MFHC1_D64 : RISCV64::MFHC1_D32)),
            DstReg)
        .addReg(SrcReg);
  } else
    BuildMI(MBB, I, dl, get(RISCV64::MFC1), DstReg).addReg(SubReg);
}

void RISCV64SEInstrInfo::expandBuildPairF64(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         bool isMicroRISCV64, bool FP64) const {
  unsigned DstReg = I->getOperand(0).getReg();
  unsigned LoReg = I->getOperand(1).getReg(), HiReg = I->getOperand(2).getReg();
  const MCInstrDesc& Mtc1Tdd = get(RISCV64::MTC1);
  DebugLoc dl = I->getDebugLoc();
  const TargetRegisterInfo &TRI = getRegisterInfo();

  // When mthc1 is available, use:
  //   mtc1 Lo, $fp
  //   mthc1 Hi, $fp
  //
  // Otherwise, for O32 FPXX ABI:
  //   spill + reload via ldc1
  // This case is handled by the frame lowering code.
  //
  // Otherwise, for FP32:
  //   mtc1 Lo, $fp
  //   mtc1 Hi, $fp + 1
  //
  // The case where dmtc1 is available doesn't need to be handled here
  // because it never creates a BuildPairF64 node.

  // FPXX on MIPS-II or MIPS32r1 should have been handled with a spill/reload
  // in RISCV64SEFrameLowering.cpp.
  assert(!(Subtarget.isABI_FPXX() && !Subtarget.hasRISCV6432r2()));

  // FP64A (FP64 with nooddspreg) should have been handled with a spill/reload
  // in RISCV64SEFrameLowering.cpp.
  assert(!(Subtarget.isFP64bit() && !Subtarget.useOddSPReg()));

  BuildMI(MBB, I, dl, Mtc1Tdd, TRI.getSubReg(DstReg, RISCV64::sub_lo))
    .addReg(LoReg);

  if (Subtarget.hasMTHC1()) {
    // FIXME: The .addReg(DstReg) is a white lie used to temporarily work
    //        around a widespread bug in the -mfp64 support.
    //        The problem is that none of the 32-bit fpu ops mention the fact
    //        that they clobber the upper 32-bits of the 64-bit FPR. Fixing that
    //        requires a major overhaul of the FPU implementation which can't
    //        be done right now due to time constraints.
    //        MTHC1 is one of two instructions that are affected since they are
    //        the only instructions that don't read the lower 32-bits.
    //        We therefore pretend that it reads the bottom 32-bits to
    //        artificially create a dependency and prevent the scheduler
    //        changing the behaviour of the code.
    BuildMI(MBB, I, dl,
            get(isMicroRISCV64 ? (FP64 ? RISCV64::MTHC1_D64_MM : RISCV64::MTHC1_D32_MM)
                            : (FP64 ? RISCV64::MTHC1_D64 : RISCV64::MTHC1_D32)),
            DstReg)
        .addReg(DstReg)
        .addReg(HiReg);
  } else if (Subtarget.isABI_FPXX())
    llvm_unreachable("BuildPairF64 not expanded in frame lowering code!");
  else
    BuildMI(MBB, I, dl, Mtc1Tdd, TRI.getSubReg(DstReg, RISCV64::sub_hi))
      .addReg(HiReg);
}

void RISCV64SEInstrInfo::expandEhReturn(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  // This pseudo instruction is generated as part of the lowering of
  // ISD::EH_RETURN. We convert it to a stack increment by OffsetReg, and
  // indirect jump to TargetReg
  RISCV64ABIInfo ABI = Subtarget.getABI();
  unsigned ADDU = ABI.GetPtrAdduOp();
  unsigned SP = Subtarget.isGP64bit() ? RISCV64::SP_64 : RISCV64::SP;
  unsigned RA = Subtarget.isGP64bit() ? RISCV64::RA_64 : RISCV64::RA;
  unsigned T9 = Subtarget.isGP64bit() ? RISCV64::T9_64 : RISCV64::T9;
  unsigned ZERO = Subtarget.isGP64bit() ? RISCV64::ZERO_64 : RISCV64::ZERO;
  unsigned OffsetReg = I->getOperand(0).getReg();
  unsigned TargetReg = I->getOperand(1).getReg();

  // addu $ra, $v0, $zero
  // addu $sp, $sp, $v1
  // jr   $ra (via RetRA)
  const TargetMachine &TM = MBB.getParent()->getTarget();
  if (TM.isPositionIndependent())
    BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), T9)
        .addReg(TargetReg)
        .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), RA)
      .addReg(TargetReg)
      .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), SP).addReg(SP).addReg(OffsetReg);
  expandRetRA(MBB, I);
}

const RISCV64InstrInfo *llvm::createRISCV64SEInstrInfo(const RISCV64Subtarget &STI) {
  return new RISCV64SEInstrInfo(STI);
}
