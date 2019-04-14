//===-- MYRISCVXMCInstLower.cpp - Convert MYRISCVX MachineInstr to MCInst ---------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower MYRISCVX MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXMCInstLower.h"
#include "MYRISCVXAsmPrinter.h"
#include "MYRISCVXInstrInfo.h"
#include "MCTargetDesc/MYRISCVXBaseInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"

using namespace llvm;
MYRISCVXMCInstLower::MYRISCVXMCInstLower(MYRISCVXAsmPrinter &asmprinter)
    : AsmPrinter(asmprinter) {}
void MYRISCVXMCInstLower::Initialize(MCContext* C) {
  Ctx = C;
}


MCOperand MYRISCVXMCInstLower::LowerSymbolOperand(const MachineOperand &MO,
                                                  MachineOperandType MOTy,
                                                  unsigned Offset) const {
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_None;
  MYRISCVXMCExpr::MYRISCVXExprKind TargetKind = MYRISCVXMCExpr::CEK_None;
  const MCSymbol *Symbol;

  switch(MO.getTargetFlags()) {
    default:                   llvm_unreachable("Invalid target flag!");
    case MYRISCVXII::MO_NO_FLAG:
      break;

      // MYRISCVX_GPREL is for llc -march=MYRISCVX -relocation-model=static -MYRISCVX-islinux-
      //  format=false (global var in .sdata).
    case MYRISCVXII::MO_GPREL:
      TargetKind = MYRISCVXMCExpr::CEK_GPREL;
      break;

    case MYRISCVXII::MO_GOT:
      TargetKind = MYRISCVXMCExpr::CEK_GOT;
      break;
    case MYRISCVXII::MO_GOT_CALL:
      TargetKind = MYRISCVXMCExpr::CEK_GOT_CALL;
      break;

      // ABS_HI and ABS_LO is for llc -march=MYRISCVX -relocation-model=static (global
      //  var in .data).
    case MYRISCVXII::MO_ABS_HI:
      TargetKind = MYRISCVXMCExpr::CEK_ABS_HI;
      break;
    case MYRISCVXII::MO_ABS_LO:
      TargetKind = MYRISCVXMCExpr::CEK_ABS_LO;
      break;
    case MYRISCVXII::MO_GOT_HI16:
      TargetKind = MYRISCVXMCExpr::CEK_GOT_HI16;
      break;
    case MYRISCVXII::MO_GOT_LO16:
      TargetKind = MYRISCVXMCExpr::CEK_GOT_LO16;
      break;
    case MYRISCVXII::MO_TLSGD:
      TargetKind = MYRISCVXMCExpr::CEK_TLSGD;
      break;
    case MYRISCVXII::MO_TLSLDM:
      TargetKind = MYRISCVXMCExpr::CEK_TLSLDM;
      break;
    case MYRISCVXII::MO_DTP_HI:
      TargetKind = MYRISCVXMCExpr::CEK_DTP_HI;
      break;
    case MYRISCVXII::MO_DTP_LO:
      TargetKind = MYRISCVXMCExpr::CEK_DTP_LO;
      break;
    case MYRISCVXII::MO_GOTTPREL:
      TargetKind = MYRISCVXMCExpr::CEK_GOTTPREL;
      break;
    case MYRISCVXII::MO_TP_HI:
      TargetKind = MYRISCVXMCExpr::CEK_TP_HI;
      break;
    case MYRISCVXII::MO_TP_LO:
      TargetKind = MYRISCVXMCExpr::CEK_TP_LO;
      break;
  }

  switch (MOTy) {
    case MachineOperand::MO_ExternalSymbol:
      Symbol = AsmPrinter.GetExternalSymbolSymbol(MO.getSymbolName());
      Offset += MO.getOffset();
      break;
    case MachineOperand::MO_GlobalAddress:
      Symbol = AsmPrinter.getSymbol(MO.getGlobal());
      Offset += MO.getOffset();
      break;
    case MachineOperand::MO_MachineBasicBlock:
      Symbol = MO.getMBB()->getSymbol();
      break;

    case MachineOperand::MO_BlockAddress:
      Symbol = AsmPrinter.GetBlockAddressSymbol(MO.getBlockAddress());
      Offset += MO.getOffset();
      break;

    case MachineOperand::MO_JumpTableIndex:
      Symbol = AsmPrinter.GetJTISymbol(MO.getIndex());
      break;

    default:
    llvm_unreachable("<unknown operand type>");
  }

  const MCExpr *Expr = MCSymbolRefExpr::create(Symbol, Kind, *Ctx);

  if (Offset) {
    // Assume offset is never negative.
    assert(Offset > 0);
    Expr = MCBinaryExpr::createAdd(Expr, MCConstantExpr::create(Offset, *Ctx),
                                   *Ctx);
  }

  if (TargetKind != MYRISCVXMCExpr::CEK_None)
    Expr = MYRISCVXMCExpr::create(TargetKind, Expr, *Ctx);

  return MCOperand::createExpr(Expr);

}


static void CreateMCInst(MCInst& Inst, unsigned Opc, const MCOperand& Opnd0,
                         const MCOperand& Opnd1,
                         const MCOperand& Opnd2 = MCOperand()) {
  Inst.setOpcode(Opc);
  Inst.addOperand(Opnd0);
  Inst.addOperand(Opnd1);
  if (Opnd2.isValid())
    Inst.addOperand(Opnd2);
}


// Lower ".cpload $reg" to
// "lui $gp, %hi(_gp_disp)"
// "addiu $gp, $gp, %lo(_gp_disp)"
// "addu $gp, $gp, $tp"
void MYRISCVXMCInstLower::LowerCPLOAD(SmallVector<MCInst, 4>& MCInsts) {
  MCOperand GPReg = MCOperand::createReg(MYRISCVX::GP);
  MCOperand TPReg = MCOperand::createReg(MYRISCVX::TP);
  StringRef SymName("_gp_disp");
  const MCSymbol *Sym = Ctx->getOrCreateSymbol(SymName);
  const MYRISCVXMCExpr *MCSym;

  MCSym = MYRISCVXMCExpr::create(Sym, MYRISCVXMCExpr::CEK_ABS_HI, *Ctx);
  MCOperand SymHi = MCOperand::createExpr(MCSym);
  MCSym = MYRISCVXMCExpr::create(Sym, MYRISCVXMCExpr::CEK_ABS_LO, *Ctx);
  MCOperand SymLo = MCOperand::createExpr(MCSym);

  MCInsts.resize(3);

  CreateMCInst(MCInsts[0], MYRISCVX::LUI, GPReg, SymHi);
  CreateMCInst(MCInsts[1], MYRISCVX::ORI, GPReg, GPReg, SymLo);
  CreateMCInst(MCInsts[2], MYRISCVX::ADD, GPReg, GPReg, TPReg);
}


#ifdef ENABLE_GPRESTORE
// Lower ".cprestore offset" to "st $gp, offset($sp)".
void MYRISCVXMCInstLower::LowerCPRESTORE(int64_t Offset,
                                         SmallVector<MCInst, 4>& MCInsts) {
  assert(isInt<32>(Offset) && (Offset >= 0) &&
         "Imm operand of .cprestore must be a non-negative 32-bit value.");

  MCOperand SPReg = MCOperand::createReg(MYRISCVX::SP), BaseReg = SPReg;
  MCOperand GPReg = MCOperand::createReg(MYRISCVX::GP);
  MCOperand ZEROReg = MCOperand::createReg(MYRISCVX::ZERO);

  if (!isInt<16>(Offset)) {
    unsigned Hi = ((Offset + 0x8000) >> 16) & 0xffff;
    Offset &= 0xffff;
    MCOperand ATReg = MCOperand::createReg(MYRISCVX::TP);
    BaseReg = ATReg;

    // lui   at,hi
    // add   at,at,sp
    MCInsts.resize(2);
    CreateMCInst(MCInsts[0], MYRISCVX::LUI, ATReg, ZEROReg, MCOperand::createImm(Hi));
    CreateMCInst(MCInsts[1], MYRISCVX::ADD, ATReg, ATReg, SPReg);
  }

  MCInst St;
  CreateMCInst(St, MYRISCVX::SW, GPReg, BaseReg, MCOperand::createImm(Offset));
  MCInsts.push_back(St);
}
#endif


//@LowerOperand {
MCOperand MYRISCVXMCInstLower::LowerOperand(const MachineOperand& MO,
                                            unsigned offset) const {
  MachineOperandType MOTy = MO.getType();
  switch (MOTy) {
    //@2
    default:
      printf("%d\n", MOTy);
      llvm_unreachable("unknown operand type");
    case MachineOperand::MO_Register:
      // Ignore all implicit register operands.
      if (MO.isImplicit()) break;
      return MCOperand::createReg(MO.getReg());
    case MachineOperand::MO_Immediate:
      return MCOperand::createImm(MO.getImm() + offset);
    case MachineOperand::MO_MachineBasicBlock:
    case MachineOperand::MO_JumpTableIndex:
    case MachineOperand::MO_BlockAddress:
    case MachineOperand::MO_GlobalAddress:
    case MachineOperand::MO_ExternalSymbol:
      return LowerSymbolOperand(MO, MOTy, offset);
    case MachineOperand::MO_RegisterMask:
      break;
  }
  return MCOperand();
}


void MYRISCVXMCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  if (lowerLongBranch(MI, OutMI))
    return;

  OutMI.setOpcode(MI->getOpcode());
  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MO);
    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}


MCOperand MYRISCVXMCInstLower::createSub(MachineBasicBlock *BB1,
                                         MachineBasicBlock *BB2,
                                         MYRISCVXMCExpr::MYRISCVXExprKind Kind) const {
  const MCSymbolRefExpr *Sym1 = MCSymbolRefExpr::create(BB1->getSymbol(), *Ctx);
  const MCSymbolRefExpr *Sym2 = MCSymbolRefExpr::create(BB2->getSymbol(), *Ctx);
  const MCBinaryExpr *Sub = MCBinaryExpr::createSub(Sym1, Sym2, *Ctx);
  return MCOperand::createExpr(MYRISCVXMCExpr::create(Kind, Sub, *Ctx));
}


void MYRISCVXMCInstLower::
lowerLongBranchLUI(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(MYRISCVX::LUI);
  // Lower register operand.
  OutMI.addOperand(LowerOperand(MI->getOperand(0)));
  // Create %hi($tgt-$baltgt).
  OutMI.addOperand(createSub(MI->getOperand(1).getMBB(),
                             MI->getOperand(2).getMBB(),
                             MYRISCVXMCExpr::CEK_ABS_HI));
}


void MYRISCVXMCInstLower::
lowerLongBranchADDI(const MachineInstr *MI, MCInst &OutMI, int Opcode,
                     MYRISCVXMCExpr::MYRISCVXExprKind Kind) const {
  OutMI.setOpcode(Opcode);
  // Lower two register operands.
  for (unsigned I = 0, E = 2; I != E; ++I) {
    const MachineOperand &MO = MI->getOperand(I);
    OutMI.addOperand(LowerOperand(MO));
  }
  // Create %lo($tgt-$baltgt) or %hi($tgt-$baltgt).
  OutMI.addOperand(createSub(MI->getOperand(2).getMBB(),
                             MI->getOperand(3).getMBB(), Kind));
}


bool MYRISCVXMCInstLower::lowerLongBranch(const MachineInstr *MI,
                                          MCInst &OutMI) const {
  switch (MI->getOpcode()) {
    default:
      return false;
    case MYRISCVX::LONG_BRANCH_LUI:
      lowerLongBranchLUI(MI, OutMI);
      return true;
    case MYRISCVX::LONG_BRANCH_ADDI:
      lowerLongBranchADDI(MI, OutMI, MYRISCVX::ADDI,
                           MYRISCVXMCExpr::CEK_ABS_LO);
      return true;
  }
}
