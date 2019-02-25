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
    default: llvm_unreachable("unknown operand type");
    case MachineOperand::MO_Register:
      // Ignore all implicit register operands.
      if (MO.isImplicit()) break;
      return MCOperand::createReg(MO.getReg());
    case MachineOperand::MO_Immediate:
      return MCOperand::createImm(MO.getImm() + offset);
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
