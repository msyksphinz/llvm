//===- RISCV64MCExpr.h - RISCV64 specific MC expression classes -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MIPS_MCTARGETDESC_MIPSMCEXPR_H
#define LLVM_LIB_TARGET_MIPS_MCTARGETDESC_MIPSMCEXPR_H

#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

class RISCV64MCExpr : public MCTargetExpr {
public:
  enum RISCV64ExprKind {
    MEK_None,
    MEK_CALL_HI16,
    MEK_CALL_LO16,
    MEK_DTPREL_HI,
    MEK_DTPREL_LO,
    MEK_GOT,
    MEK_GOTTPREL,
    MEK_GOT_CALL,
    MEK_GOT_DISP,
    MEK_GOT_HI16,
    MEK_GOT_LO16,
    MEK_GOT_OFST,
    MEK_GOT_PAGE,
    MEK_GPREL,
    MEK_HI,
    MEK_HIGHER,
    MEK_HIGHEST,
    MEK_LO,
    MEK_NEG,
    MEK_PCREL_HI16,
    MEK_PCREL_LO16,
    MEK_TLSGD,
    MEK_TLSLDM,
    MEK_TPREL_HI,
    MEK_TPREL_LO,
    MEK_Special,
  };

private:
  const RISCV64ExprKind Kind;
  const MCExpr *Expr;

  explicit RISCV64MCExpr(RISCV64ExprKind Kind, const MCExpr *Expr)
      : Kind(Kind), Expr(Expr) {}

public:
  static const RISCV64MCExpr *create(RISCV64ExprKind Kind, const MCExpr *Expr,
                                  MCContext &Ctx);
  static const RISCV64MCExpr *createGpOff(RISCV64ExprKind Kind, const MCExpr *Expr,
                                       MCContext &Ctx);

  /// Get the kind of this expression.
  RISCV64ExprKind getKind() const { return Kind; }

  /// Get the child of this expression.
  const MCExpr *getSubExpr() const { return Expr; }

  void printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const override;
  bool evaluateAsRelocatableImpl(MCValue &Res, const MCAsmLayout *Layout,
                                 const MCFixup *Fixup) const override;
  void visitUsedExpr(MCStreamer &Streamer) const override;

  MCFragment *findAssociatedFragment() const override {
    return getSubExpr()->findAssociatedFragment();
  }

  void fixELFSymbolsInTLSFixups(MCAssembler &Asm) const override;

  static bool classof(const MCExpr *E) {
    return E->getKind() == MCExpr::Target;
  }

  bool isGpOff(RISCV64ExprKind &Kind) const;
  bool isGpOff() const {
    RISCV64ExprKind Kind;
    return isGpOff(Kind);
  }
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MIPS_MCTARGETDESC_MIPSMCEXPR_H
