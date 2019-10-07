//===-- MYRISCVXMCExpr.cpp - MYRISCVX specific MC expression classes --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVX.h"

#include "MYRISCVXMCExpr.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCObjectStreamer.h"
#include "llvm/MC/MCSymbolELF.h"

using namespace llvm;

#define DEBUG_TYPE "MYRISCVXmcexpr"

const MYRISCVXMCExpr *MYRISCVXMCExpr::create(MYRISCVXMCExpr::MYRISCVXExprKind Kind,
                                             const MCExpr *Expr, MCContext &Ctx) {
  return new (Ctx) MYRISCVXMCExpr(Kind, Expr);
}

const MYRISCVXMCExpr *MYRISCVXMCExpr::create(const MCSymbol *Symbol, MYRISCVXMCExpr::MYRISCVXExprKind Kind,
                                             MCContext &Ctx) {
  const MCSymbolRefExpr *MCSym =
      MCSymbolRefExpr::create(Symbol, MCSymbolRefExpr::VK_None, Ctx);
  return new (Ctx) MYRISCVXMCExpr(Kind, MCSym);
}

const MYRISCVXMCExpr *MYRISCVXMCExpr::createGpOff(MYRISCVXMCExpr::MYRISCVXExprKind Kind,
                                                  const MCExpr *Expr, MCContext &Ctx) {
  return create(Kind, create(CEK_None, create(CEK_GPREL, Expr, Ctx), Ctx), Ctx);
}

void MYRISCVXMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  int64_t AbsVal;

  switch (Kind) {
    case CEK_None:
    case CEK_Special:
      llvm_unreachable("CEK_None and CEK_Special are invalid");
      break;
    case CEK_CALL_HI20:
      OS << "%call_hi";
      break;
    case CEK_CALL_LO12:
      OS << "%call_lo";
      break;
    case CEK_DTP_HI:
      OS << "%dtp_hi";
      break;
    case CEK_DTP_LO:
      OS << "%dtp_lo";
      break;
    case CEK_GOT:
      OS << "%got";
      break;
    case CEK_GOTTPREL:
      OS << "%gottprel";
      break;
    case CEK_GOT_CALL:
      OS << "%call16";
      break;
    case CEK_GOT_DISP:
      OS << "%got_disp";
      break;
    case CEK_GOT_HI20:
      OS << "%got_hi";
      break;
    case CEK_GOT_LO12:
      OS << "%got_lo";
      break;
    case CEK_GPREL:
      OS << "%gp_rel";
      break;
    case CEK_ABS_HI:
      OS << "%hi";
      break;
    case CEK_ABS_LO:
      OS << "%lo";
      break;
    case CEK_TLSGD:
      OS << "%tlsgd";
      break;
    case CEK_TLSLDM:
      OS << "%tlsldm";
      break;
    case CEK_TP_HI:
      OS << "%tp_hi";
      break;
    case CEK_TP_LO:
      OS << "%tp_lo";
      break;
  }

  OS << '(';
  if (Expr->evaluateAsAbsolute(AbsVal))
    OS << AbsVal;
  else
    Expr->print(OS, MAI, true);
  OS << ')';
}

bool
MYRISCVXMCExpr::evaluateAsRelocatableImpl(MCValue &Res,
                                          const MCAsmLayout *Layout,
                                          const MCFixup *Fixup) const {
  return getSubExpr()->evaluateAsRelocatable(Res, Layout, Fixup);
}

void MYRISCVXMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

void MYRISCVXMCExpr::fixELFSymbolsInTLSFixups(MCAssembler &Asm) const {
  switch (getKind()) {
    default:
      llvm_unreachable("Invalid ELF Symbol in TLS kind");
    case CEK_None:
    case CEK_Special:
      llvm_unreachable("CEK_None and CEK_Special are invalid");
      break;
    case CEK_CALL_HI20:
    case CEK_CALL_LO12:
      break;
  }
}

bool MYRISCVXMCExpr::isGpOff(MYRISCVXExprKind &Kind) const {
  if (const MYRISCVXMCExpr *S1 = dyn_cast<const MYRISCVXMCExpr>(getSubExpr())) {
    if (const MYRISCVXMCExpr *S2 = dyn_cast<const MYRISCVXMCExpr>(S1->getSubExpr())) {
      if (S1->getKind() == CEK_None && S2->getKind() == CEK_GPREL) {
        Kind = getKind();
        return true;
      }
    }
  }
  return false;
}
