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
  }

  switch (MOTy) {
  case MachineOperand::MO_GlobalAddress:
    Symbol = AsmPrinter.getSymbol(MO.getGlobal());
    Offset += MO.getOffset();
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
    case MachineOperand::MO_GlobalAddress:
      return LowerSymbolOperand(MO, MOTy, offset);
    case MachineOperand::MO_RegisterMask:
      break;
  }
  return MCOperand();
}


void MYRISCVXMCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(MI->getOpcode());
  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MO);
    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}
