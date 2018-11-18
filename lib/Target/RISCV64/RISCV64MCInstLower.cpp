//===- RISCV64MCInstLower.cpp - Convert RISCV64 MachineInstr to MCInst ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower RISCV64 MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "RISCV64MCInstLower.h"
#include "MCTargetDesc/RISCV64BaseInfo.h"
#include "MCTargetDesc/RISCV64MCExpr.h"
#include "RISCV64AsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>

using namespace llvm;

RISCV64MCInstLower::RISCV64MCInstLower(RISCV64AsmPrinter &asmprinter)
  : AsmPrinter(asmprinter) {}

void RISCV64MCInstLower::Initialize(MCContext *C) {
  Ctx = C;
}

MCOperand RISCV64MCInstLower::LowerSymbolOperand(const MachineOperand &MO,
                                              MachineOperandType MOTy,
                                              unsigned Offset) const {
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_None;
  RISCV64MCExpr::RISCV64ExprKind TargetKind = RISCV64MCExpr::MEK_None;
  bool IsGpOff = false;
  const MCSymbol *Symbol;

  switch(MO.getTargetFlags()) {
  default:
    llvm_unreachable("Invalid target flag!");
  case RISCV64II::MO_NO_FLAG:
    break;
  case RISCV64II::MO_GPREL:
    TargetKind = RISCV64MCExpr::MEK_GPREL;
    break;
  case RISCV64II::MO_GOT_CALL:
    TargetKind = RISCV64MCExpr::MEK_GOT_CALL;
    break;
  case RISCV64II::MO_GOT:
    TargetKind = RISCV64MCExpr::MEK_GOT;
    break;
  case RISCV64II::MO_ABS_HI:
    TargetKind = RISCV64MCExpr::MEK_HI;
    break;
  case RISCV64II::MO_ABS_LO:
    TargetKind = RISCV64MCExpr::MEK_LO;
    break;
  case RISCV64II::MO_TLSGD:
    TargetKind = RISCV64MCExpr::MEK_TLSGD;
    break;
  case RISCV64II::MO_TLSLDM:
    TargetKind = RISCV64MCExpr::MEK_TLSLDM;
    break;
  case RISCV64II::MO_DTPREL_HI:
    TargetKind = RISCV64MCExpr::MEK_DTPREL_HI;
    break;
  case RISCV64II::MO_DTPREL_LO:
    TargetKind = RISCV64MCExpr::MEK_DTPREL_LO;
    break;
  case RISCV64II::MO_GOTTPREL:
    TargetKind = RISCV64MCExpr::MEK_GOTTPREL;
    break;
  case RISCV64II::MO_TPREL_HI:
    TargetKind = RISCV64MCExpr::MEK_TPREL_HI;
    break;
  case RISCV64II::MO_TPREL_LO:
    TargetKind = RISCV64MCExpr::MEK_TPREL_LO;
    break;
  case RISCV64II::MO_GPOFF_HI:
    TargetKind = RISCV64MCExpr::MEK_HI;
    IsGpOff = true;
    break;
  case RISCV64II::MO_GPOFF_LO:
    TargetKind = RISCV64MCExpr::MEK_LO;
    IsGpOff = true;
    break;
  case RISCV64II::MO_GOT_DISP:
    TargetKind = RISCV64MCExpr::MEK_GOT_DISP;
    break;
  case RISCV64II::MO_GOT_HI16:
    TargetKind = RISCV64MCExpr::MEK_GOT_HI16;
    break;
  case RISCV64II::MO_GOT_LO16:
    TargetKind = RISCV64MCExpr::MEK_GOT_LO16;
    break;
  case RISCV64II::MO_GOT_PAGE:
    TargetKind = RISCV64MCExpr::MEK_GOT_PAGE;
    break;
  case RISCV64II::MO_GOT_OFST:
    TargetKind = RISCV64MCExpr::MEK_GOT_OFST;
    break;
  case RISCV64II::MO_HIGHER:
    TargetKind = RISCV64MCExpr::MEK_HIGHER;
    break;
  case RISCV64II::MO_HIGHEST:
    TargetKind = RISCV64MCExpr::MEK_HIGHEST;
    break;
  case RISCV64II::MO_CALL_HI16:
    TargetKind = RISCV64MCExpr::MEK_CALL_HI16;
    break;
  case RISCV64II::MO_CALL_LO16:
    TargetKind = RISCV64MCExpr::MEK_CALL_LO16;
    break;
  }

  switch (MOTy) {
  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    break;

  case MachineOperand::MO_GlobalAddress:
    Symbol = AsmPrinter.getSymbol(MO.getGlobal());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_BlockAddress:
    Symbol = AsmPrinter.GetBlockAddressSymbol(MO.getBlockAddress());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_ExternalSymbol:
    Symbol = AsmPrinter.GetExternalSymbolSymbol(MO.getSymbolName());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_MCSymbol:
    Symbol = MO.getMCSymbol();
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_JumpTableIndex:
    Symbol = AsmPrinter.GetJTISymbol(MO.getIndex());
    break;

  case MachineOperand::MO_ConstantPoolIndex:
    Symbol = AsmPrinter.GetCPISymbol(MO.getIndex());
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

  if (IsGpOff)
    Expr = RISCV64MCExpr::createGpOff(TargetKind, Expr, *Ctx);
  else if (TargetKind != RISCV64MCExpr::MEK_None)
    Expr = RISCV64MCExpr::create(TargetKind, Expr, *Ctx);

  return MCOperand::createExpr(Expr);
}

MCOperand RISCV64MCInstLower::LowerOperand(const MachineOperand &MO,
                                        unsigned offset) const {
  MachineOperandType MOTy = MO.getType();

  switch (MOTy) {
  default: llvm_unreachable("unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit()) break;
    return MCOperand::createReg(MO.getReg());
  case MachineOperand::MO_Immediate:
    return MCOperand::createImm(MO.getImm() + offset);
  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_MCSymbol:
  case MachineOperand::MO_JumpTableIndex:
  case MachineOperand::MO_ConstantPoolIndex:
  case MachineOperand::MO_BlockAddress:
    return LowerSymbolOperand(MO, MOTy, offset);
  case MachineOperand::MO_RegisterMask:
    break;
 }

  return MCOperand();
}

MCOperand RISCV64MCInstLower::createSub(MachineBasicBlock *BB1,
                                     MachineBasicBlock *BB2,
                                     RISCV64MCExpr::RISCV64ExprKind Kind) const {
  const MCSymbolRefExpr *Sym1 = MCSymbolRefExpr::create(BB1->getSymbol(), *Ctx);
  const MCSymbolRefExpr *Sym2 = MCSymbolRefExpr::create(BB2->getSymbol(), *Ctx);
  const MCBinaryExpr *Sub = MCBinaryExpr::createSub(Sym1, Sym2, *Ctx);

  return MCOperand::createExpr(RISCV64MCExpr::create(Kind, Sub, *Ctx));
}

void RISCV64MCInstLower::
lowerLongBranchLUi(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(RISCV64::LUi);

  // Lower register operand.
  OutMI.addOperand(LowerOperand(MI->getOperand(0)));

  RISCV64MCExpr::RISCV64ExprKind Kind;
  unsigned TargetFlags = MI->getOperand(1).getTargetFlags();
  switch (TargetFlags) {
  case RISCV64II::MO_HIGHEST:
    Kind = RISCV64MCExpr::MEK_HIGHEST;
    break;
  case RISCV64II::MO_HIGHER:
    Kind = RISCV64MCExpr::MEK_HIGHER;
    break;
  case RISCV64II::MO_ABS_HI:
    Kind = RISCV64MCExpr::MEK_HI;
    break;
  case RISCV64II::MO_ABS_LO:
    Kind = RISCV64MCExpr::MEK_LO;
    break;
  default:
    report_fatal_error("Unexpected flags for lowerLongBranchLUi");
  }

  if (MI->getNumOperands() == 2) {
    const MCExpr *Expr =
        MCSymbolRefExpr::create(MI->getOperand(1).getMBB()->getSymbol(), *Ctx);
    const RISCV64MCExpr *RISCV64Expr = RISCV64MCExpr::create(Kind, Expr, *Ctx);
    OutMI.addOperand(MCOperand::createExpr(RISCV64Expr));
  } else if (MI->getNumOperands() == 3) {
    // Create %hi($tgt-$baltgt).
    OutMI.addOperand(createSub(MI->getOperand(1).getMBB(),
                               MI->getOperand(2).getMBB(), Kind));
  }
}

void RISCV64MCInstLower::lowerLongBranchADDiu(const MachineInstr *MI,
                                           MCInst &OutMI, int Opcode) const {
  OutMI.setOpcode(Opcode);

  RISCV64MCExpr::RISCV64ExprKind Kind;
  unsigned TargetFlags = MI->getOperand(2).getTargetFlags();
  switch (TargetFlags) {
  case RISCV64II::MO_HIGHEST:
    Kind = RISCV64MCExpr::MEK_HIGHEST;
    break;
  case RISCV64II::MO_HIGHER:
    Kind = RISCV64MCExpr::MEK_HIGHER;
    break;
  case RISCV64II::MO_ABS_HI:
    Kind = RISCV64MCExpr::MEK_HI;
    break;
  case RISCV64II::MO_ABS_LO:
    Kind = RISCV64MCExpr::MEK_LO;
    break;
  default:
    report_fatal_error("Unexpected flags for lowerLongBranchADDiu");
  }

  // Lower two register operands.
  for (unsigned I = 0, E = 2; I != E; ++I) {
    const MachineOperand &MO = MI->getOperand(I);
    OutMI.addOperand(LowerOperand(MO));
  }

  if (MI->getNumOperands() == 3) {
    // Lower register operand.
    const MCExpr *Expr =
        MCSymbolRefExpr::create(MI->getOperand(2).getMBB()->getSymbol(), *Ctx);
    const RISCV64MCExpr *RISCV64Expr = RISCV64MCExpr::create(Kind, Expr, *Ctx);
    OutMI.addOperand(MCOperand::createExpr(RISCV64Expr));
  } else if (MI->getNumOperands() == 4) {
    // Create %lo($tgt-$baltgt) or %hi($tgt-$baltgt).
    OutMI.addOperand(createSub(MI->getOperand(2).getMBB(),
                               MI->getOperand(3).getMBB(), Kind));
  }
}

bool RISCV64MCInstLower::lowerLongBranch(const MachineInstr *MI,
                                      MCInst &OutMI) const {
  switch (MI->getOpcode()) {
  default:
    return false;
  case RISCV64::LONG_BRANCH_LUi:
  case RISCV64::LONG_BRANCH_LUi2Op:
  case RISCV64::LONG_BRANCH_LUi2Op_64:
    lowerLongBranchLUi(MI, OutMI);
    return true;
  case RISCV64::LONG_BRANCH_ADDiu:
  case RISCV64::LONG_BRANCH_ADDiu2Op:
    lowerLongBranchADDiu(MI, OutMI, RISCV64::ADDiu);
    return true;
  case RISCV64::LONG_BRANCH_DADDiu:
  case RISCV64::LONG_BRANCH_DADDiu2Op:
    lowerLongBranchADDiu(MI, OutMI, RISCV64::DADDiu);
    return true;
  }
}

void RISCV64MCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
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
