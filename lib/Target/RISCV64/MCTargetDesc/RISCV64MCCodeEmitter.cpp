//===-- RISCV64MCCodeEmitter.cpp - Convert RISCV64 Code to Machine Code ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the RISCV64MCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "RISCV64MCCodeEmitter.h"
#include "MCTargetDesc/RISCV64FixupKinds.h"
#include "MCTargetDesc/RISCV64MCExpr.h"
#include "MCTargetDesc/RISCV64MCTargetDesc.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <cassert>
#include <cstdint>

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "RISCV64GenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

namespace llvm {

MCCodeEmitter *createRISCV64MCCodeEmitterEB(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx) {
  return new RISCV64MCCodeEmitter(MCII, Ctx, false);
}

MCCodeEmitter *createRISCV64MCCodeEmitterEL(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx) {
  return new RISCV64MCCodeEmitter(MCII, Ctx, true);
}

} // end namespace llvm

// If the D<shift> instruction has a shift amount that is greater
// than 31 (checked in calling routine), lower it to a D<shift>32 instruction
static void LowerLargeShift(MCInst& Inst) {
  assert(Inst.getNumOperands() == 3 && "Invalid no. of operands for shift!");
  assert(Inst.getOperand(2).isImm());

  int64_t Shift = Inst.getOperand(2).getImm();
  if (Shift <= 31)
    return; // Do nothing
  Shift -= 32;

  // saminus32
  Inst.getOperand(2).setImm(Shift);

  switch (Inst.getOpcode()) {
  default:
    // Calling function is not synchronized
    llvm_unreachable("Unexpected shift instruction");
  case RISCV64::DSLL:
    Inst.setOpcode(RISCV64::DSLL32);
    return;
  case RISCV64::DSRL:
    Inst.setOpcode(RISCV64::DSRL32);
    return;
  case RISCV64::DSRA:
    Inst.setOpcode(RISCV64::DSRA32);
    return;
  case RISCV64::DROTR:
    Inst.setOpcode(RISCV64::DROTR32);
    return;
  }
}

// Fix a bad compact branch encoding for beqc/bnec.
void RISCV64MCCodeEmitter::LowerCompactBranch(MCInst& Inst) const {
  // Encoding may be illegal !(rs < rt), but this situation is
  // easily fixed.
  unsigned RegOp0 = Inst.getOperand(0).getReg();
  unsigned RegOp1 = Inst.getOperand(1).getReg();

  unsigned Reg0 =  Ctx.getRegisterInfo()->getEncodingValue(RegOp0);
  unsigned Reg1 =  Ctx.getRegisterInfo()->getEncodingValue(RegOp1);

  if (Inst.getOpcode() == RISCV64::BNEC || Inst.getOpcode() == RISCV64::BEQC ||
      Inst.getOpcode() == RISCV64::BNEC64 || Inst.getOpcode() == RISCV64::BEQC64) {
    assert(Reg0 != Reg1 && "Instruction has bad operands ($rs == $rt)!");
    if (Reg0 < Reg1)
      return;
  } else if (Inst.getOpcode() == RISCV64::BNVC || Inst.getOpcode() == RISCV64::BOVC) {
    if (Reg0 >= Reg1)
      return;
  } else if (Inst.getOpcode() == RISCV64::BNVC_MMR6 ||
             Inst.getOpcode() == RISCV64::BOVC_MMR6) {
    if (Reg1 >= Reg0)
      return;
  } else
    llvm_unreachable("Cannot rewrite unknown branch!");

  Inst.getOperand(0).setReg(RegOp1);
  Inst.getOperand(1).setReg(RegOp0);
}

bool RISCV64MCCodeEmitter::isMicroRISCV64(const MCSubtargetInfo &STI) const {
  return STI.getFeatureBits()[RISCV64::FeatureMicroRISCV64];
}

bool RISCV64MCCodeEmitter::isRISCV6432r6(const MCSubtargetInfo &STI) const {
  return STI.getFeatureBits()[RISCV64::FeatureRISCV6432r6];
}

void RISCV64MCCodeEmitter::EmitByte(unsigned char C, raw_ostream &OS) const {
  OS << (char)C;
}

void RISCV64MCCodeEmitter::EmitInstruction(uint64_t Val, unsigned Size,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &OS) const {
  // Output the instruction encoding in little endian byte order.
  // Little-endian byte ordering:
  //   mips32r2:   4 | 3 | 2 | 1
  //   microMIPS:  2 | 1 | 4 | 3
  if (IsLittleEndian && Size == 4 && isMicroRISCV64(STI)) {
    EmitInstruction(Val >> 16, 2, STI, OS);
    EmitInstruction(Val, 2, STI, OS);
  } else {
    for (unsigned i = 0; i < Size; ++i) {
      unsigned Shift = IsLittleEndian ? i * 8 : (Size - 1 - i) * 8;
      EmitByte((Val >> Shift) & 0xff, OS);
    }
  }
}

/// encodeInstruction - Emit the instruction.
/// Size the instruction with Desc.getSize().
void RISCV64MCCodeEmitter::
encodeInstruction(const MCInst &MI, raw_ostream &OS,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const
{
  // Non-pseudo instructions that get changed for direct object
  // only based on operand values.
  // If this list of instructions get much longer we will move
  // the check to a function call. Until then, this is more efficient.
  MCInst TmpInst = MI;
  switch (MI.getOpcode()) {
  // If shift amount is >= 32 it the inst needs to be lowered further
  case RISCV64::DSLL:
  case RISCV64::DSRL:
  case RISCV64::DSRA:
  case RISCV64::DROTR:
    LowerLargeShift(TmpInst);
    break;
  // Compact branches, enforce encoding restrictions.
  case RISCV64::BEQC:
  case RISCV64::BNEC:
  case RISCV64::BEQC64:
  case RISCV64::BNEC64:
  case RISCV64::BOVC:
  case RISCV64::BOVC_MMR6:
  case RISCV64::BNVC:
  case RISCV64::BNVC_MMR6:
    LowerCompactBranch(TmpInst);
  }

  unsigned long N = Fixups.size();
  uint32_t Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);

  // Check for unimplemented opcodes.
  // Unfortunately in MIPS both NOP and SLL will come in with Binary == 0
  // so we have to special check for them.
  unsigned Opcode = TmpInst.getOpcode();
  if ((Opcode != RISCV64::NOP) && (Opcode != RISCV64::SLL) &&
      (Opcode != RISCV64::SLL_MM) && (Opcode != RISCV64::SLL_MMR6) && !Binary)
    llvm_unreachable("unimplemented opcode in encodeInstruction()");

  int NewOpcode = -1;
  if (isMicroRISCV64(STI)) {
    if (isRISCV6432r6(STI)) {
      NewOpcode = RISCV64::RISCV64R62MicroRISCV64R6(Opcode, RISCV64::Arch_micromipsr6);
      if (NewOpcode == -1)
        NewOpcode = RISCV64::Std2MicroRISCV64R6(Opcode, RISCV64::Arch_micromipsr6);
    }
    else
      NewOpcode = RISCV64::Std2MicroRISCV64(Opcode, RISCV64::Arch_micromips);

    // Check whether it is Dsp instruction.
    if (NewOpcode == -1)
      NewOpcode = RISCV64::Dsp2MicroRISCV64(Opcode, RISCV64::Arch_mmdsp);

    if (NewOpcode != -1) {
      if (Fixups.size() > N)
        Fixups.pop_back();

      Opcode = NewOpcode;
      TmpInst.setOpcode (NewOpcode);
      Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
    }

    if (((MI.getOpcode() == RISCV64::MOVEP_MM) ||
         (MI.getOpcode() == RISCV64::MOVEP_MMR6))) {
      unsigned RegPair = getMovePRegPairOpValue(MI, 0, Fixups, STI);
      Binary = (Binary & 0xFFFFFC7F) | (RegPair << 7);
    }
  }

  const MCInstrDesc &Desc = MCII.get(TmpInst.getOpcode());

  // Get byte count of instruction
  unsigned Size = Desc.getSize();
  if (!Size)
    llvm_unreachable("Desc.getSize() returns 0");

  EmitInstruction(Binary, Size, STI, OS);
}

/// getBranchTargetOpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTargetOpValue(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTargetOpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(RISCV64::fixup_RISCV64_PC16)));
  return 0;
}

/// getBranchTargetOpValue1SImm16 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTargetOpValue1SImm16(const MCInst &MI, unsigned OpNo,
                              SmallVectorImpl<MCFixup> &Fixups,
                              const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(RISCV64::fixup_RISCV64_PC16)));
  return 0;
}

/// getBranchTargetOpValueMMR6 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTargetOpValueMMR6(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm())
    return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMMR6 expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-2, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(RISCV64::fixup_RISCV64_PC16)));
  return 0;
}

/// getBranchTargetOpValueLsl2MMR6 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTargetOpValueLsl2MMR6(const MCInst &MI, unsigned OpNo,
                               SmallVectorImpl<MCFixup> &Fixups,
                               const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm())
    return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueLsl2MMR6 expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(RISCV64::fixup_RISCV64_PC16)));
  return 0;
}

/// getBranchTarget7OpValueMM - Return binary encoding of the microMIPS branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTarget7OpValueMM(const MCInst &MI, unsigned OpNo,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMM expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(RISCV64::fixup_MICROMIPS_PC7_S1)));
  return 0;
}

/// getBranchTargetOpValueMMPC10 - Return binary encoding of the microMIPS
/// 10-bit branch target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTargetOpValueMMPC10(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValuePC10 expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                   MCFixupKind(RISCV64::fixup_MICROMIPS_PC10_S1)));
  return 0;
}

/// getBranchTargetOpValue - Return binary encoding of the microMIPS branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTargetOpValueMM(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMM expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                   MCFixupKind(RISCV64::
                               fixup_MICROMIPS_PC16_S1)));
  return 0;
}

/// getBranchTarget21OpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTarget21OpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTarget21OpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(RISCV64::fixup_MIPS_PC21_S2)));
  return 0;
}

/// getBranchTarget21OpValueMM - Return binary encoding of the branch
/// target operand for microMIPS. If the machine operand requires
/// relocation, record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTarget21OpValueMM(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
    "getBranchTarget21OpValueMM expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(RISCV64::fixup_MICROMIPS_PC21_S1)));
  return 0;
}

/// getBranchTarget26OpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getBranchTarget26OpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTarget26OpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(RISCV64::fixup_MIPS_PC26_S2)));
  return 0;
}

/// getBranchTarget26OpValueMM - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::getBranchTarget26OpValueMM(
    const MCInst &MI, unsigned OpNo, SmallVectorImpl<MCFixup> &Fixups,
    const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm())
    return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTarget26OpValueMM expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(RISCV64::fixup_MICROMIPS_PC26_S1)));
  return 0;
}

/// getJumpOffset16OpValue - Return binary encoding of the jump
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getJumpOffset16OpValue(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isImm()) return MO.getImm();

  assert(MO.isExpr() &&
         "getJumpOffset16OpValue expects only expressions or an immediate");

   // TODO: Push fixup.
   return 0;
}

/// getJumpTargetOpValue - Return binary encoding of the jump
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getJumpTargetOpValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm()>>2;

  assert(MO.isExpr() &&
         "getJumpTargetOpValue expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(RISCV64::fixup_RISCV64_26)));
  return 0;
}

unsigned RISCV64MCCodeEmitter::
getJumpTargetOpValueMM(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getJumpTargetOpValueMM expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(RISCV64::fixup_MICROMIPS_26_S1)));
  return 0;
}

unsigned RISCV64MCCodeEmitter::
getUImm5Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 2'.
    unsigned Res = getMachineOpValue(MI, MO, Fixups, STI);
    assert((Res & 3) == 0);
    return Res >> 2;
  }

  assert(MO.isExpr() &&
         "getUImm5Lsl2Encoding expects only expressions or an immediate");

  return 0;
}

unsigned RISCV64MCCodeEmitter::
getSImm3Lsa2Value(const MCInst &MI, unsigned OpNo,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    int Value = MO.getImm();
    return Value >> 2;
  }

  return 0;
}

unsigned RISCV64MCCodeEmitter::
getUImm6Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    unsigned Value = MO.getImm();
    return Value >> 2;
  }

  return 0;
}

unsigned RISCV64MCCodeEmitter::
getSImm9AddiuspValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    unsigned Binary = (MO.getImm() >> 2) & 0x0000ffff;
    return (((Binary & 0x8000) >> 7) | (Binary & 0x00ff));
  }

  return 0;
}

unsigned RISCV64MCCodeEmitter::
getExprOpValue(const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups,
               const MCSubtargetInfo &STI) const {
  int64_t Res;

  if (Expr->evaluateAsAbsolute(Res))
    return Res;

  MCExpr::ExprKind Kind = Expr->getKind();
  if (Kind == MCExpr::Constant) {
    return cast<MCConstantExpr>(Expr)->getValue();
  }

  if (Kind == MCExpr::Binary) {
    unsigned Res = getExprOpValue(cast<MCBinaryExpr>(Expr)->getLHS(), Fixups, STI);
    Res += getExprOpValue(cast<MCBinaryExpr>(Expr)->getRHS(), Fixups, STI);
    return Res;
  }

  if (Kind == MCExpr::Target) {
    const RISCV64MCExpr *RISCV64Expr = cast<RISCV64MCExpr>(Expr);

    RISCV64::Fixups FixupKind = RISCV64::Fixups(0);
    switch (RISCV64Expr->getKind()) {
    case RISCV64MCExpr::MEK_None:
    case RISCV64MCExpr::MEK_Special:
      llvm_unreachable("Unhandled fixup kind!");
      break;
    case RISCV64MCExpr::MEK_CALL_HI16:
      FixupKind = RISCV64::fixup_RISCV64_CALL_HI16;
      break;
    case RISCV64MCExpr::MEK_CALL_LO16:
      FixupKind = RISCV64::fixup_RISCV64_CALL_LO16;
      break;
    case RISCV64MCExpr::MEK_DTPREL_HI:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_TLS_DTPREL_HI16
                                   : RISCV64::fixup_RISCV64_DTPREL_HI;
      break;
    case RISCV64MCExpr::MEK_DTPREL_LO:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_TLS_DTPREL_LO16
                                   : RISCV64::fixup_RISCV64_DTPREL_LO;
      break;
    case RISCV64MCExpr::MEK_GOTTPREL:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_GOTTPREL
                                   : RISCV64::fixup_RISCV64_GOTTPREL;
      break;
    case RISCV64MCExpr::MEK_GOT:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_GOT16
                                   : RISCV64::fixup_RISCV64_GOT;
      break;
    case RISCV64MCExpr::MEK_GOT_CALL:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_CALL16
                                   : RISCV64::fixup_RISCV64_CALL16;
      break;
    case RISCV64MCExpr::MEK_GOT_DISP:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_GOT_DISP
                                   : RISCV64::fixup_RISCV64_GOT_DISP;
      break;
    case RISCV64MCExpr::MEK_GOT_HI16:
      FixupKind = RISCV64::fixup_RISCV64_GOT_HI16;
      break;
    case RISCV64MCExpr::MEK_GOT_LO16:
      FixupKind = RISCV64::fixup_RISCV64_GOT_LO16;
      break;
    case RISCV64MCExpr::MEK_GOT_PAGE:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_GOT_PAGE
                                   : RISCV64::fixup_RISCV64_GOT_PAGE;
      break;
    case RISCV64MCExpr::MEK_GOT_OFST:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_GOT_OFST
                                   : RISCV64::fixup_RISCV64_GOT_OFST;
      break;
    case RISCV64MCExpr::MEK_GPREL:
      FixupKind = RISCV64::fixup_RISCV64_GPREL16;
      break;
    case RISCV64MCExpr::MEK_LO:
      // Check for %lo(%neg(%gp_rel(X)))
      if (RISCV64Expr->isGpOff())
        FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_GPOFF_LO
                                     : RISCV64::fixup_RISCV64_GPOFF_LO;
      else
        FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_LO16
                                     : RISCV64::fixup_RISCV64_LO16;
      break;
    case RISCV64MCExpr::MEK_HIGHEST:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_HIGHEST
                                   : RISCV64::fixup_RISCV64_HIGHEST;
      break;
    case RISCV64MCExpr::MEK_HIGHER:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_HIGHER
                                   : RISCV64::fixup_RISCV64_HIGHER;
      break;
    case RISCV64MCExpr::MEK_HI:
      // Check for %hi(%neg(%gp_rel(X)))
      if (RISCV64Expr->isGpOff())
        FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_GPOFF_HI
                                     : RISCV64::fixup_RISCV64_GPOFF_HI;
      else
        FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_HI16
                                     : RISCV64::fixup_RISCV64_HI16;
      break;
    case RISCV64MCExpr::MEK_PCREL_HI16:
      FixupKind = RISCV64::fixup_MIPS_PCHI16;
      break;
    case RISCV64MCExpr::MEK_PCREL_LO16:
      FixupKind = RISCV64::fixup_MIPS_PCLO16;
      break;
    case RISCV64MCExpr::MEK_TLSGD:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_TLS_GD
                                   : RISCV64::fixup_RISCV64_TLSGD;
      break;
    case RISCV64MCExpr::MEK_TLSLDM:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_TLS_LDM
                                   : RISCV64::fixup_RISCV64_TLSLDM;
      break;
    case RISCV64MCExpr::MEK_TPREL_HI:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_TLS_TPREL_HI16
                                   : RISCV64::fixup_RISCV64_TPREL_HI;
      break;
    case RISCV64MCExpr::MEK_TPREL_LO:
      FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_TLS_TPREL_LO16
                                   : RISCV64::fixup_RISCV64_TPREL_LO;
      break;
    case RISCV64MCExpr::MEK_NEG:
      FixupKind =
          isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_SUB : RISCV64::fixup_RISCV64_SUB;
      break;
    }
    Fixups.push_back(MCFixup::create(0, RISCV64Expr, MCFixupKind(FixupKind)));
    return 0;
  }

  if (Kind == MCExpr::SymbolRef) {
    RISCV64::Fixups FixupKind = RISCV64::Fixups(0);

    switch(cast<MCSymbolRefExpr>(Expr)->getKind()) {
    default: llvm_unreachable("Unknown fixup kind!");
      break;
    case MCSymbolRefExpr::VK_None:
      FixupKind = RISCV64::fixup_RISCV64_32; // FIXME: This is ok for O32/N32 but not N64.
      break;
    } // switch

    Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
    return 0;
  }
  return 0;
}

/// getMachineOpValue - Return binary encoding of operand. If the machine
/// operand requires relocation, record the relocation and return zero.
unsigned RISCV64MCCodeEmitter::
getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  if (MO.isReg()) {
    unsigned Reg = MO.getReg();
    unsigned RegNo = Ctx.getRegisterInfo()->getEncodingValue(Reg);
    return RegNo;
  } else if (MO.isImm()) {
    return static_cast<unsigned>(MO.getImm());
  } else if (MO.isFPImm()) {
    return static_cast<unsigned>(APFloat(MO.getFPImm())
        .bitcastToAPInt().getHiBits(32).getLimitedValue());
  }
  // MO must be an Expr.
  assert(MO.isExpr());
  return getExprOpValue(MO.getExpr(),Fixups, STI);
}

/// Return binary encoding of memory related operand.
/// If the offset operand requires relocation, record the relocation.
template <unsigned ShiftAmount>
unsigned RISCV64MCCodeEmitter::getMemEncoding(const MCInst &MI, unsigned OpNo,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 15-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),Fixups, STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  // Apply the scale factor if there is one.
  OffBits >>= ShiftAmount;

  return (OffBits & 0xFFFF) | RegBits;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMImm4(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 6-4, offset is encoded in bits 3-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),
                                       Fixups, STI) << 4;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI);

  return (OffBits & 0xF) | RegBits;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMImm4Lsl1(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 6-4, offset is encoded in bits 3-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),
                                       Fixups, STI) << 4;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 1;

  return (OffBits & 0xF) | RegBits;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMImm4Lsl2(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 6-4, offset is encoded in bits 3-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),
                                       Fixups, STI) << 4;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 2;

  return (OffBits & 0xF) | RegBits;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMSPImm5Lsl2(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  // Register is encoded in bits 9-5, offset is encoded in bits 4-0.
  assert(MI.getOperand(OpNo).isReg() &&
         (MI.getOperand(OpNo).getReg() == RISCV64::SP ||
         MI.getOperand(OpNo).getReg() == RISCV64::SP_64) &&
         "Unexpected base register!");
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 2;

  return OffBits & 0x1F;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMGPImm7Lsl2(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  // Register is encoded in bits 9-7, offset is encoded in bits 6-0.
  assert(MI.getOperand(OpNo).isReg() &&
         MI.getOperand(OpNo).getReg() == RISCV64::GP &&
         "Unexpected base register!");

  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 2;

  return OffBits & 0x7F;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMImm9(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 8-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups,
                                       STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo + 1), Fixups, STI);

  return (OffBits & 0x1FF) | RegBits;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMImm11(const MCInst &MI, unsigned OpNo,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 10-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups,
                                       STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0x07FF) | RegBits;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMImm12(const MCInst &MI, unsigned OpNo,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  // opNum can be invalid if instruction had reglist as operand.
  // MemOperand is always last operand of instruction (base + offset).
  switch (MI.getOpcode()) {
  default:
    break;
  case RISCV64::SWM32_MM:
  case RISCV64::LWM32_MM:
    OpNo = MI.getNumOperands() - 2;
    break;
  }

  // Base register is encoded in bits 20-16, offset is encoded in bits 11-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0x0FFF) | RegBits;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMImm16(const MCInst &MI, unsigned OpNo,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 15-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups,
                                       STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0xFFFF) | RegBits;
}

unsigned RISCV64MCCodeEmitter::
getMemEncodingMMImm4sp(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  // opNum can be invalid if instruction had reglist as operand
  // MemOperand is always last operand of instruction (base + offset)
  switch (MI.getOpcode()) {
  default:
    break;
  case RISCV64::SWM16_MM:
  case RISCV64::SWM16_MMR6:
  case RISCV64::LWM16_MM:
  case RISCV64::LWM16_MMR6:
    OpNo = MI.getNumOperands() - 2;
    break;
  }

  // Offset is encoded in bits 4-0.
  assert(MI.getOperand(OpNo).isReg());
  // Base register is always SP - thus it is not encoded.
  assert(MI.getOperand(OpNo+1).isImm());
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return ((OffBits >> 2) & 0x0F);
}

// FIXME: should be called getMSBEncoding
//
unsigned
RISCV64MCCodeEmitter::getSizeInsEncoding(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo-1).isImm());
  assert(MI.getOperand(OpNo).isImm());
  unsigned Position = getMachineOpValue(MI, MI.getOperand(OpNo-1), Fixups, STI);
  unsigned Size = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);

  return Position + Size - 1;
}

template <unsigned Bits, int Offset>
unsigned
RISCV64MCCodeEmitter::getUImmWithOffsetEncoding(const MCInst &MI, unsigned OpNo,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  unsigned Value = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
  Value -= Offset;
  return Value;
}

unsigned
RISCV64MCCodeEmitter::getSimm19Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 2'.
    unsigned Res = getMachineOpValue(MI, MO, Fixups, STI);
    assert((Res & 3) == 0);
    return Res >> 2;
  }

  assert(MO.isExpr() &&
         "getSimm19Lsl2Encoding expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  RISCV64::Fixups FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_PC19_S2
                                            : RISCV64::fixup_MIPS_PC19_S2;
  Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
  return 0;
}

unsigned
RISCV64MCCodeEmitter::getSimm18Lsl3Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 3'.
    unsigned Res = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
    assert((Res & 7) == 0);
    return Res >> 3;
  }

  assert(MO.isExpr() &&
         "getSimm18Lsl2Encoding expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  RISCV64::Fixups FixupKind = isMicroRISCV64(STI) ? RISCV64::fixup_MICROMIPS_PC18_S3
                                            : RISCV64::fixup_MIPS_PC18_S3;
  Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
  return 0;
}

unsigned
RISCV64MCCodeEmitter::getUImm3Mod8Encoding(const MCInst &MI, unsigned OpNo,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  const MCOperand &MO = MI.getOperand(OpNo);
  return MO.getImm() % 8;
}

unsigned
RISCV64MCCodeEmitter::getUImm4AndValue(const MCInst &MI, unsigned OpNo,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  const MCOperand &MO = MI.getOperand(OpNo);
  unsigned Value = MO.getImm();
  switch (Value) {
    case 128:   return 0x0;
    case 1:     return 0x1;
    case 2:     return 0x2;
    case 3:     return 0x3;
    case 4:     return 0x4;
    case 7:     return 0x5;
    case 8:     return 0x6;
    case 15:    return 0x7;
    case 16:    return 0x8;
    case 31:    return 0x9;
    case 32:    return 0xa;
    case 63:    return 0xb;
    case 64:    return 0xc;
    case 255:   return 0xd;
    case 32768: return 0xe;
    case 65535: return 0xf;
  }
  llvm_unreachable("Unexpected value");
}

unsigned
RISCV64MCCodeEmitter::getRegisterListOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  unsigned res = 0;

  // Register list operand is always first operand of instruction and it is
  // placed before memory operand (register + imm).

  for (unsigned I = OpNo, E = MI.getNumOperands() - 2; I < E; ++I) {
    unsigned Reg = MI.getOperand(I).getReg();
    unsigned RegNo = Ctx.getRegisterInfo()->getEncodingValue(Reg);
    if (RegNo != 31)
      res++;
    else
      res |= 0x10;
  }
  return res;
}

unsigned
RISCV64MCCodeEmitter::getRegisterListOpValue16(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  return (MI.getNumOperands() - 4);
}

unsigned
RISCV64MCCodeEmitter::getMovePRegPairOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  unsigned res = 0;

  if (MI.getOperand(0).getReg() == RISCV64::A1 &&
      MI.getOperand(1).getReg() == RISCV64::A2)
    res = 0;
  else if (MI.getOperand(0).getReg() == RISCV64::A1 &&
           MI.getOperand(1).getReg() == RISCV64::A3)
    res = 1;
  else if (MI.getOperand(0).getReg() == RISCV64::A2 &&
           MI.getOperand(1).getReg() == RISCV64::A3)
    res = 2;
  else if (MI.getOperand(0).getReg() == RISCV64::A0 &&
           MI.getOperand(1).getReg() == RISCV64::S5)
    res = 3;
  else if (MI.getOperand(0).getReg() == RISCV64::A0 &&
           MI.getOperand(1).getReg() == RISCV64::S6)
    res = 4;
  else if (MI.getOperand(0).getReg() == RISCV64::A0 &&
           MI.getOperand(1).getReg() == RISCV64::A1)
    res = 5;
  else if (MI.getOperand(0).getReg() == RISCV64::A0 &&
           MI.getOperand(1).getReg() == RISCV64::A2)
    res = 6;
  else if (MI.getOperand(0).getReg() == RISCV64::A0 &&
           MI.getOperand(1).getReg() == RISCV64::A3)
    res = 7;

  return res;
}

unsigned
RISCV64MCCodeEmitter::getMovePRegSingleOpValue(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  assert(((OpNo == 2) || (OpNo == 3)) &&
         "Unexpected OpNo for movep operand encoding!");

  MCOperand Op = MI.getOperand(OpNo);
  assert(Op.isReg() && "Operand of movep is not a register!");
  switch (Op.getReg()) {
  default:
    llvm_unreachable("Unknown register for movep!");
  case RISCV64::ZERO:  return 0;
  case RISCV64::S1:    return 1;
  case RISCV64::V0:    return 2;
  case RISCV64::V1:    return 3;
  case RISCV64::S0:    return 4;
  case RISCV64::S2:    return 5;
  case RISCV64::S3:    return 6;
  case RISCV64::S4:    return 7;
  }
}

unsigned
RISCV64MCCodeEmitter::getSimm23Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  assert(MO.isImm() && "getSimm23Lsl2Encoding expects only an immediate");
  // The immediate is encoded as 'immediate >> 2'.
  unsigned Res = static_cast<unsigned>(MO.getImm());
  assert((Res & 3) == 0);
  return Res >> 2;
}

#include "RISCV64GenMCCodeEmitter.inc"
