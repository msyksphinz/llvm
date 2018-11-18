//===-- RISCV64TargetStreamer.cpp - RISCV64 Target Streamer Methods -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides RISCV64 specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "RISCV64TargetStreamer.h"
#include "InstPrinter/RISCV64InstPrinter.h"
#include "MCTargetDesc/RISCV64ABIInfo.h"
#include "RISCV64ELFStreamer.h"
#include "RISCV64MCExpr.h"
#include "RISCV64MCTargetDesc.h"
#include "RISCV64TargetObjectFile.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

namespace {
static cl::opt<bool> RoundSectionSizes(
    "mips-round-section-sizes", cl::init(false),
    cl::desc("Round section sizes up to the section alignment"), cl::Hidden);
} // end anonymous namespace

RISCV64TargetStreamer::RISCV64TargetStreamer(MCStreamer &S)
    : MCTargetStreamer(S), ModuleDirectiveAllowed(true) {
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;
}
void RISCV64TargetStreamer::emitDirectiveSetMicroRISCV64() {}
void RISCV64TargetStreamer::emitDirectiveSetNoMicroRISCV64() {}
void RISCV64TargetStreamer::setUsesMicroRISCV64() {}
void RISCV64TargetStreamer::emitDirectiveSetRISCV6416() {}
void RISCV64TargetStreamer::emitDirectiveSetNoRISCV6416() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetReorder() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetNoReorder() {}
void RISCV64TargetStreamer::emitDirectiveSetMacro() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetNoMacro() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetMsa() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetNoMsa() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetMt() {}
void RISCV64TargetStreamer::emitDirectiveSetNoMt() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetCRC() {}
void RISCV64TargetStreamer::emitDirectiveSetNoCRC() {}
void RISCV64TargetStreamer::emitDirectiveSetVirt() {}
void RISCV64TargetStreamer::emitDirectiveSetNoVirt() {}
void RISCV64TargetStreamer::emitDirectiveSetGINV() {}
void RISCV64TargetStreamer::emitDirectiveSetNoGINV() {}
void RISCV64TargetStreamer::emitDirectiveSetAt() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetAtWithArg(unsigned RegNo) {
  forbidModuleDirective();
}
void RISCV64TargetStreamer::emitDirectiveSetNoAt() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveEnd(StringRef Name) {}
void RISCV64TargetStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {}
void RISCV64TargetStreamer::emitDirectiveAbiCalls() {}
void RISCV64TargetStreamer::emitDirectiveNaN2008() {}
void RISCV64TargetStreamer::emitDirectiveNaNLegacy() {}
void RISCV64TargetStreamer::emitDirectiveOptionPic0() {}
void RISCV64TargetStreamer::emitDirectiveOptionPic2() {}
void RISCV64TargetStreamer::emitDirectiveInsn() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                   unsigned ReturnReg) {}
void RISCV64TargetStreamer::emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff) {}
void RISCV64TargetStreamer::emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff) {
}
void RISCV64TargetStreamer::emitDirectiveSetArch(StringRef Arch) {
  forbidModuleDirective();
}
void RISCV64TargetStreamer::emitDirectiveSetRISCV640() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV641() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV642() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV643() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV644() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV645() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6432() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6432R2() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6432R3() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6432R5() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6432R6() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6464() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6464R2() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6464R3() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6464R5() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetRISCV6464R6() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetPop() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetPush() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetSoftFloat() {
  forbidModuleDirective();
}
void RISCV64TargetStreamer::emitDirectiveSetHardFloat() {
  forbidModuleDirective();
}
void RISCV64TargetStreamer::emitDirectiveSetDsp() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetDspr2() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetNoDsp() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveCpLoad(unsigned RegNo) {}
bool RISCV64TargetStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  forbidModuleDirective();
  return true;
}
void RISCV64TargetStreamer::emitDirectiveCpsetup(unsigned RegNo, int RegOrOffset,
                                              const MCSymbol &Sym, bool IsReg) {
}
void RISCV64TargetStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                               bool SaveLocationIsRegister) {}

void RISCV64TargetStreamer::emitDirectiveModuleFP() {}

void RISCV64TargetStreamer::emitDirectiveModuleOddSPReg() {
  if (!ABIFlagsSection.OddSPReg && !ABIFlagsSection.Is32BitABI)
    report_fatal_error("+nooddspreg is only valid for O32");
}
void RISCV64TargetStreamer::emitDirectiveModuleSoftFloat() {}
void RISCV64TargetStreamer::emitDirectiveModuleHardFloat() {}
void RISCV64TargetStreamer::emitDirectiveModuleMT() {}
void RISCV64TargetStreamer::emitDirectiveModuleCRC() {}
void RISCV64TargetStreamer::emitDirectiveModuleNoCRC() {}
void RISCV64TargetStreamer::emitDirectiveModuleVirt() {}
void RISCV64TargetStreamer::emitDirectiveModuleNoVirt() {}
void RISCV64TargetStreamer::emitDirectiveModuleGINV() {}
void RISCV64TargetStreamer::emitDirectiveModuleNoGINV() {}
void RISCV64TargetStreamer::emitDirectiveSetFp(
    RISCV64ABIFlagsSection::FpABIKind Value) {
  forbidModuleDirective();
}
void RISCV64TargetStreamer::emitDirectiveSetOddSPReg() { forbidModuleDirective(); }
void RISCV64TargetStreamer::emitDirectiveSetNoOddSPReg() {
  forbidModuleDirective();
}

void RISCV64TargetStreamer::emitR(unsigned Opcode, unsigned Reg0, SMLoc IDLoc,
                               const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void RISCV64TargetStreamer::emitRX(unsigned Opcode, unsigned Reg0, MCOperand Op1,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(Op1);
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void RISCV64TargetStreamer::emitRI(unsigned Opcode, unsigned Reg0, int32_t Imm,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRX(Opcode, Reg0, MCOperand::createImm(Imm), IDLoc, STI);
}

void RISCV64TargetStreamer::emitRR(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRX(Opcode, Reg0, MCOperand::createReg(Reg1), IDLoc, STI);
}

void RISCV64TargetStreamer::emitII(unsigned Opcode, int16_t Imm1, int16_t Imm2,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createImm(Imm1));
  TmpInst.addOperand(MCOperand::createImm(Imm2));
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void RISCV64TargetStreamer::emitRRX(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 MCOperand Op2, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(MCOperand::createReg(Reg1));
  TmpInst.addOperand(Op2);
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void RISCV64TargetStreamer::emitRRR(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 unsigned Reg2, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  emitRRX(Opcode, Reg0, Reg1, MCOperand::createReg(Reg2), IDLoc, STI);
}

void RISCV64TargetStreamer::emitRRI(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 int16_t Imm, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  emitRRX(Opcode, Reg0, Reg1, MCOperand::createImm(Imm), IDLoc, STI);
}

void RISCV64TargetStreamer::emitRRIII(unsigned Opcode, unsigned Reg0,
                                   unsigned Reg1, int16_t Imm0, int16_t Imm1,
                                   int16_t Imm2, SMLoc IDLoc,
                                   const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(MCOperand::createReg(Reg1));
  TmpInst.addOperand(MCOperand::createImm(Imm0));
  TmpInst.addOperand(MCOperand::createImm(Imm1));
  TmpInst.addOperand(MCOperand::createImm(Imm2));
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void RISCV64TargetStreamer::emitAddu(unsigned DstReg, unsigned SrcReg,
                                  unsigned TrgReg, bool Is64Bit,
                                  const MCSubtargetInfo *STI) {
  emitRRR(Is64Bit ? RISCV64::DADDu : RISCV64::ADDu, DstReg, SrcReg, TrgReg, SMLoc(),
          STI);
}

void RISCV64TargetStreamer::emitDSLL(unsigned DstReg, unsigned SrcReg,
                                  int16_t ShiftAmount, SMLoc IDLoc,
                                  const MCSubtargetInfo *STI) {
  if (ShiftAmount >= 32) {
    emitRRI(RISCV64::DSLL32, DstReg, SrcReg, ShiftAmount - 32, IDLoc, STI);
    return;
  }

  emitRRI(RISCV64::DSLL, DstReg, SrcReg, ShiftAmount, IDLoc, STI);
}

void RISCV64TargetStreamer::emitEmptyDelaySlot(bool hasShortDelaySlot, SMLoc IDLoc,
                                            const MCSubtargetInfo *STI) {
  if (hasShortDelaySlot)
    emitRR(RISCV64::MOVE16_MM, RISCV64::ZERO, RISCV64::ZERO, IDLoc, STI);
  else
    emitRRI(RISCV64::SLL, RISCV64::ZERO, RISCV64::ZERO, 0, IDLoc, STI);
}

void RISCV64TargetStreamer::emitNop(SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRRI(RISCV64::SLL, RISCV64::ZERO, RISCV64::ZERO, 0, IDLoc, STI);
}

/// Emit the $gp restore operation for .cprestore.
void RISCV64TargetStreamer::emitGPRestore(int Offset, SMLoc IDLoc,
                                       const MCSubtargetInfo *STI) {
  emitLoadWithImmOffset(RISCV64::LW, RISCV64::GP, RISCV64::SP, Offset, RISCV64::GP, IDLoc,
                        STI);
}

/// Emit a store instruction with an immediate offset.
void RISCV64TargetStreamer::emitStoreWithImmOffset(
    unsigned Opcode, unsigned SrcReg, unsigned BaseReg, int64_t Offset,
    function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  if (isInt<16>(Offset)) {
    emitRRI(Opcode, SrcReg, BaseReg, Offset, IDLoc, STI);
    return;
  }

  // sw $8, offset($8) => lui $at, %hi(offset)
  //                      add $at, $at, $8
  //                      sw $8, %lo(offset)($at)

  unsigned ATReg = GetATReg();
  if (!ATReg)
    return;

  unsigned LoOffset = Offset & 0x0000ffff;
  unsigned HiOffset = (Offset & 0xffff0000) >> 16;

  // If msb of LoOffset is 1(negative number) we must increment HiOffset
  // to account for the sign-extension of the low part.
  if (LoOffset & 0x8000)
    HiOffset++;

  // Generate the base address in ATReg.
  emitRI(RISCV64::LUi, ATReg, HiOffset, IDLoc, STI);
  if (BaseReg != RISCV64::ZERO)
    emitRRR(RISCV64::ADDu, ATReg, ATReg, BaseReg, IDLoc, STI);
  // Emit the store with the adjusted base and offset.
  emitRRI(Opcode, SrcReg, ATReg, LoOffset, IDLoc, STI);
}

/// Emit a store instruction with an symbol offset. Symbols are assumed to be
/// out of range for a simm16 will be expanded to appropriate instructions.
void RISCV64TargetStreamer::emitStoreWithSymOffset(
    unsigned Opcode, unsigned SrcReg, unsigned BaseReg, MCOperand &HiOperand,
    MCOperand &LoOperand, unsigned ATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  // sw $8, sym => lui $at, %hi(sym)
  //               sw $8, %lo(sym)($at)

  // Generate the base address in ATReg.
  emitRX(RISCV64::LUi, ATReg, HiOperand, IDLoc, STI);
  if (BaseReg != RISCV64::ZERO)
    emitRRR(RISCV64::ADDu, ATReg, ATReg, BaseReg, IDLoc, STI);
  // Emit the store with the adjusted base and offset.
  emitRRX(Opcode, SrcReg, ATReg, LoOperand, IDLoc, STI);
}

/// Emit a load instruction with an immediate offset. DstReg and TmpReg are
/// permitted to be the same register iff DstReg is distinct from BaseReg and
/// DstReg is a GPR. It is the callers responsibility to identify such cases
/// and pass the appropriate register in TmpReg.
void RISCV64TargetStreamer::emitLoadWithImmOffset(unsigned Opcode, unsigned DstReg,
                                               unsigned BaseReg, int64_t Offset,
                                               unsigned TmpReg, SMLoc IDLoc,
                                               const MCSubtargetInfo *STI) {
  if (isInt<16>(Offset)) {
    emitRRI(Opcode, DstReg, BaseReg, Offset, IDLoc, STI);
    return;
  }

  // 1) lw $8, offset($9) => lui $8, %hi(offset)
  //                         add $8, $8, $9
  //                         lw $8, %lo(offset)($9)
  // 2) lw $8, offset($8) => lui $at, %hi(offset)
  //                         add $at, $at, $8
  //                         lw $8, %lo(offset)($at)

  unsigned LoOffset = Offset & 0x0000ffff;
  unsigned HiOffset = (Offset & 0xffff0000) >> 16;

  // If msb of LoOffset is 1(negative number) we must increment HiOffset
  // to account for the sign-extension of the low part.
  if (LoOffset & 0x8000)
    HiOffset++;

  // Generate the base address in TmpReg.
  emitRI(RISCV64::LUi, TmpReg, HiOffset, IDLoc, STI);
  if (BaseReg != RISCV64::ZERO)
    emitRRR(RISCV64::ADDu, TmpReg, TmpReg, BaseReg, IDLoc, STI);
  // Emit the load with the adjusted base and offset.
  emitRRI(Opcode, DstReg, TmpReg, LoOffset, IDLoc, STI);
}

/// Emit a load instruction with an symbol offset. Symbols are assumed to be
/// out of range for a simm16 will be expanded to appropriate instructions.
/// DstReg and TmpReg are permitted to be the same register iff DstReg is a
/// GPR. It is the callers responsibility to identify such cases and pass the
/// appropriate register in TmpReg.
void RISCV64TargetStreamer::emitLoadWithSymOffset(unsigned Opcode, unsigned DstReg,
                                               unsigned BaseReg,
                                               MCOperand &HiOperand,
                                               MCOperand &LoOperand,
                                               unsigned TmpReg, SMLoc IDLoc,
                                               const MCSubtargetInfo *STI) {
  // 1) lw $8, sym        => lui $8, %hi(sym)
  //                         lw $8, %lo(sym)($8)
  // 2) ldc1 $f0, sym     => lui $at, %hi(sym)
  //                         ldc1 $f0, %lo(sym)($at)

  // Generate the base address in TmpReg.
  emitRX(RISCV64::LUi, TmpReg, HiOperand, IDLoc, STI);
  if (BaseReg != RISCV64::ZERO)
    emitRRR(RISCV64::ADDu, TmpReg, TmpReg, BaseReg, IDLoc, STI);
  // Emit the load with the adjusted base and offset.
  emitRRX(Opcode, DstReg, TmpReg, LoOperand, IDLoc, STI);
}

RISCV64TargetAsmStreamer::RISCV64TargetAsmStreamer(MCStreamer &S,
                                             formatted_raw_ostream &OS)
    : RISCV64TargetStreamer(S), OS(OS) {}

void RISCV64TargetAsmStreamer::emitDirectiveSetMicroRISCV64() {
  OS << "\t.set\tmicromips\n";
  forbidModuleDirective();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoMicroRISCV64() {
  OS << "\t.set\tnomicromips\n";
  forbidModuleDirective();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6416() {
  OS << "\t.set\tmips16\n";
  forbidModuleDirective();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoRISCV6416() {
  OS << "\t.set\tnomips16\n";
  RISCV64TargetStreamer::emitDirectiveSetNoRISCV6416();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetReorder() {
  OS << "\t.set\treorder\n";
  RISCV64TargetStreamer::emitDirectiveSetReorder();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoReorder() {
  OS << "\t.set\tnoreorder\n";
  forbidModuleDirective();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetMacro() {
  OS << "\t.set\tmacro\n";
  RISCV64TargetStreamer::emitDirectiveSetMacro();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoMacro() {
  OS << "\t.set\tnomacro\n";
  RISCV64TargetStreamer::emitDirectiveSetNoMacro();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetMsa() {
  OS << "\t.set\tmsa\n";
  RISCV64TargetStreamer::emitDirectiveSetMsa();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoMsa() {
  OS << "\t.set\tnomsa\n";
  RISCV64TargetStreamer::emitDirectiveSetNoMsa();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetMt() {
  OS << "\t.set\tmt\n";
  RISCV64TargetStreamer::emitDirectiveSetMt();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoMt() {
  OS << "\t.set\tnomt\n";
  RISCV64TargetStreamer::emitDirectiveSetNoMt();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetCRC() {
  OS << "\t.set\tcrc\n";
  RISCV64TargetStreamer::emitDirectiveSetCRC();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoCRC() {
  OS << "\t.set\tnocrc\n";
  RISCV64TargetStreamer::emitDirectiveSetNoCRC();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetVirt() {
  OS << "\t.set\tvirt\n";
  RISCV64TargetStreamer::emitDirectiveSetVirt();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoVirt() {
  OS << "\t.set\tnovirt\n";
  RISCV64TargetStreamer::emitDirectiveSetNoVirt();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetGINV() {
  OS << "\t.set\tginv\n";
  RISCV64TargetStreamer::emitDirectiveSetGINV();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoGINV() {
  OS << "\t.set\tnoginv\n";
  RISCV64TargetStreamer::emitDirectiveSetNoGINV();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetAt() {
  OS << "\t.set\tat\n";
  RISCV64TargetStreamer::emitDirectiveSetAt();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetAtWithArg(unsigned RegNo) {
  OS << "\t.set\tat=$" << Twine(RegNo) << "\n";
  RISCV64TargetStreamer::emitDirectiveSetAtWithArg(RegNo);
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoAt() {
  OS << "\t.set\tnoat\n";
  RISCV64TargetStreamer::emitDirectiveSetNoAt();
}

void RISCV64TargetAsmStreamer::emitDirectiveEnd(StringRef Name) {
  OS << "\t.end\t" << Name << '\n';
}

void RISCV64TargetAsmStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {
  OS << "\t.ent\t" << Symbol.getName() << '\n';
}

void RISCV64TargetAsmStreamer::emitDirectiveAbiCalls() { OS << "\t.abicalls\n"; }

void RISCV64TargetAsmStreamer::emitDirectiveNaN2008() { OS << "\t.nan\t2008\n"; }

void RISCV64TargetAsmStreamer::emitDirectiveNaNLegacy() {
  OS << "\t.nan\tlegacy\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveOptionPic0() {
  OS << "\t.option\tpic0\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveOptionPic2() {
  OS << "\t.option\tpic2\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveInsn() {
  RISCV64TargetStreamer::emitDirectiveInsn();
  OS << "\t.insn\n";
}

void RISCV64TargetAsmStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                      unsigned ReturnReg) {
  OS << "\t.frame\t$"
     << StringRef(RISCV64InstPrinter::getRegisterName(StackReg)).lower() << ","
     << StackSize << ",$"
     << StringRef(RISCV64InstPrinter::getRegisterName(ReturnReg)).lower() << '\n';
}

void RISCV64TargetAsmStreamer::emitDirectiveSetArch(StringRef Arch) {
  OS << "\t.set arch=" << Arch << "\n";
  RISCV64TargetStreamer::emitDirectiveSetArch(Arch);
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV640() {
  OS << "\t.set\tmips0\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV640();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV641() {
  OS << "\t.set\tmips1\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV641();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV642() {
  OS << "\t.set\tmips2\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV642();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV643() {
  OS << "\t.set\tmips3\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV643();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV644() {
  OS << "\t.set\tmips4\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV644();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV645() {
  OS << "\t.set\tmips5\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV645();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6432() {
  OS << "\t.set\tmips32\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6432();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6432R2() {
  OS << "\t.set\tmips32r2\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6432R2();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6432R3() {
  OS << "\t.set\tmips32r3\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6432R3();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6432R5() {
  OS << "\t.set\tmips32r5\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6432R5();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6432R6() {
  OS << "\t.set\tmips32r6\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6432R6();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6464() {
  OS << "\t.set\tmips64\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6464();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6464R2() {
  OS << "\t.set\tmips64r2\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6464R2();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6464R3() {
  OS << "\t.set\tmips64r3\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6464R3();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6464R5() {
  OS << "\t.set\tmips64r5\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6464R5();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetRISCV6464R6() {
  OS << "\t.set\tmips64r6\n";
  RISCV64TargetStreamer::emitDirectiveSetRISCV6464R6();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetDsp() {
  OS << "\t.set\tdsp\n";
  RISCV64TargetStreamer::emitDirectiveSetDsp();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetDspr2() {
  OS << "\t.set\tdspr2\n";
  RISCV64TargetStreamer::emitDirectiveSetDspr2();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoDsp() {
  OS << "\t.set\tnodsp\n";
  RISCV64TargetStreamer::emitDirectiveSetNoDsp();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetPop() {
  OS << "\t.set\tpop\n";
  RISCV64TargetStreamer::emitDirectiveSetPop();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetPush() {
 OS << "\t.set\tpush\n";
 RISCV64TargetStreamer::emitDirectiveSetPush();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetSoftFloat() {
  OS << "\t.set\tsoftfloat\n";
  RISCV64TargetStreamer::emitDirectiveSetSoftFloat();
}

void RISCV64TargetAsmStreamer::emitDirectiveSetHardFloat() {
  OS << "\t.set\thardfloat\n";
  RISCV64TargetStreamer::emitDirectiveSetHardFloat();
}

// Print a 32 bit hex number with all numbers.
static void printHex32(unsigned Value, raw_ostream &OS) {
  OS << "0x";
  for (int i = 7; i >= 0; i--)
    OS.write_hex((Value & (0xF << (i * 4))) >> (i * 4));
}

void RISCV64TargetAsmStreamer::emitMask(unsigned CPUBitmask,
                                     int CPUTopSavedRegOff) {
  OS << "\t.mask \t";
  printHex32(CPUBitmask, OS);
  OS << ',' << CPUTopSavedRegOff << '\n';
}

void RISCV64TargetAsmStreamer::emitFMask(unsigned FPUBitmask,
                                      int FPUTopSavedRegOff) {
  OS << "\t.fmask\t";
  printHex32(FPUBitmask, OS);
  OS << "," << FPUTopSavedRegOff << '\n';
}

void RISCV64TargetAsmStreamer::emitDirectiveCpLoad(unsigned RegNo) {
  OS << "\t.cpload\t$"
     << StringRef(RISCV64InstPrinter::getRegisterName(RegNo)).lower() << "\n";
  forbidModuleDirective();
}

bool RISCV64TargetAsmStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  RISCV64TargetStreamer::emitDirectiveCpRestore(Offset, GetATReg, IDLoc, STI);
  OS << "\t.cprestore\t" << Offset << "\n";
  return true;
}

void RISCV64TargetAsmStreamer::emitDirectiveCpsetup(unsigned RegNo,
                                                 int RegOrOffset,
                                                 const MCSymbol &Sym,
                                                 bool IsReg) {
  OS << "\t.cpsetup\t$"
     << StringRef(RISCV64InstPrinter::getRegisterName(RegNo)).lower() << ", ";

  if (IsReg)
    OS << "$"
       << StringRef(RISCV64InstPrinter::getRegisterName(RegOrOffset)).lower();
  else
    OS << RegOrOffset;

  OS << ", ";

  OS << Sym.getName();
  forbidModuleDirective();
}

void RISCV64TargetAsmStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                                  bool SaveLocationIsRegister) {
  OS << "\t.cpreturn";
  forbidModuleDirective();
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleFP() {
  OS << "\t.module\tfp=";
  OS << ABIFlagsSection.getFpABIString(ABIFlagsSection.getFpABI()) << "\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveSetFp(
    RISCV64ABIFlagsSection::FpABIKind Value) {
  RISCV64TargetStreamer::emitDirectiveSetFp(Value);

  OS << "\t.set\tfp=";
  OS << ABIFlagsSection.getFpABIString(Value) << "\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleOddSPReg() {
  RISCV64TargetStreamer::emitDirectiveModuleOddSPReg();

  OS << "\t.module\t" << (ABIFlagsSection.OddSPReg ? "" : "no") << "oddspreg\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveSetOddSPReg() {
  RISCV64TargetStreamer::emitDirectiveSetOddSPReg();
  OS << "\t.set\toddspreg\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveSetNoOddSPReg() {
  RISCV64TargetStreamer::emitDirectiveSetNoOddSPReg();
  OS << "\t.set\tnooddspreg\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleSoftFloat() {
  OS << "\t.module\tsoftfloat\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleHardFloat() {
  OS << "\t.module\thardfloat\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleMT() {
  OS << "\t.module\tmt\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleCRC() {
  OS << "\t.module\tcrc\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleNoCRC() {
  OS << "\t.module\tnocrc\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleVirt() {
  OS << "\t.module\tvirt\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleNoVirt() {
  OS << "\t.module\tnovirt\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleGINV() {
  OS << "\t.module\tginv\n";
}

void RISCV64TargetAsmStreamer::emitDirectiveModuleNoGINV() {
  OS << "\t.module\tnoginv\n";
}

// This part is for ELF object output.
RISCV64TargetELFStreamer::RISCV64TargetELFStreamer(MCStreamer &S,
                                             const MCSubtargetInfo &STI)
    : RISCV64TargetStreamer(S), MicroRISCV64Enabled(false), STI(STI) {
  MCAssembler &MCA = getStreamer().getAssembler();

  // It's possible that MCObjectFileInfo isn't fully initialized at this point
  // due to an initialization order problem where LLVMTargetMachine creates the
  // target streamer before TargetLoweringObjectFile calls
  // InitializeMCObjectFileInfo. There doesn't seem to be a single place that
  // covers all cases so this statement covers most cases and direct object
  // emission must call setPic() once MCObjectFileInfo has been initialized. The
  // cases we don't handle here are covered by RISCV64AsmPrinter.
  Pic = MCA.getContext().getObjectFileInfo()->isPositionIndependent();

  const FeatureBitset &Features = STI.getFeatureBits();

  // Set the header flags that we can in the constructor.
  // FIXME: This is a fairly terrible hack. We set the rest
  // of these in the destructor. The problem here is two-fold:
  //
  // a: Some of the eflags can be set/reset by directives.
  // b: There aren't any usage paths that initialize the ABI
  //    pointer until after we initialize either an assembler
  //    or the target machine.
  // We can fix this by making the target streamer construct
  // the ABI, but this is fraught with wide ranging dependency
  // issues as well.
  unsigned EFlags = MCA.getELFHeaderEFlags();

  // FIXME: Fix a dependency issue by instantiating the ABI object to some
  // default based off the triple. The triple doesn't describe the target
  // fully, but any external user of the API that uses the MCTargetStreamer
  // would otherwise crash on assertion failure.

  ABI = RISCV64ABIInfo(
      STI.getTargetTriple().getArch() == Triple::ArchType::mipsel ||
              STI.getTargetTriple().getArch() == Triple::ArchType::mips
          ? RISCV64ABIInfo::O32()
          : RISCV64ABIInfo::N64());

  // Architecture
  if (Features[RISCV64::FeatureRISCV6464r6])
    EFlags |= ELF::EF_MIPS_ARCH_64R6;
  else if (Features[RISCV64::FeatureRISCV6464r2] ||
           Features[RISCV64::FeatureRISCV6464r3] ||
           Features[RISCV64::FeatureRISCV6464r5])
    EFlags |= ELF::EF_MIPS_ARCH_64R2;
  else if (Features[RISCV64::FeatureRISCV6464])
    EFlags |= ELF::EF_MIPS_ARCH_64;
  else if (Features[RISCV64::FeatureRISCV645])
    EFlags |= ELF::EF_MIPS_ARCH_5;
  else if (Features[RISCV64::FeatureRISCV644])
    EFlags |= ELF::EF_MIPS_ARCH_4;
  else if (Features[RISCV64::FeatureRISCV643])
    EFlags |= ELF::EF_MIPS_ARCH_3;
  else if (Features[RISCV64::FeatureRISCV6432r6])
    EFlags |= ELF::EF_MIPS_ARCH_32R6;
  else if (Features[RISCV64::FeatureRISCV6432r2] ||
           Features[RISCV64::FeatureRISCV6432r3] ||
           Features[RISCV64::FeatureRISCV6432r5])
    EFlags |= ELF::EF_MIPS_ARCH_32R2;
  else if (Features[RISCV64::FeatureRISCV6432])
    EFlags |= ELF::EF_MIPS_ARCH_32;
  else if (Features[RISCV64::FeatureRISCV642])
    EFlags |= ELF::EF_MIPS_ARCH_2;
  else
    EFlags |= ELF::EF_MIPS_ARCH_1;

  // Machine
  if (Features[RISCV64::FeatureCnRISCV64])
    EFlags |= ELF::EF_MIPS_MACH_OCTEON;

  // Other options.
  if (Features[RISCV64::FeatureNaN2008])
    EFlags |= ELF::EF_MIPS_NAN2008;

  MCA.setELFHeaderEFlags(EFlags);
}

void RISCV64TargetELFStreamer::emitLabel(MCSymbol *S) {
  auto *Symbol = cast<MCSymbolELF>(S);
  getStreamer().getAssembler().registerSymbol(*Symbol);
  uint8_t Type = Symbol->getType();
  if (Type != ELF::STT_FUNC)
    return;

  if (isMicroRISCV64Enabled())
    Symbol->setOther(ELF::STO_MIPS_MICROMIPS);
}

void RISCV64TargetELFStreamer::finish() {
  MCAssembler &MCA = getStreamer().getAssembler();
  const MCObjectFileInfo &OFI = *MCA.getContext().getObjectFileInfo();

  // .bss, .text and .data are always at least 16-byte aligned.
  MCSection &TextSection = *OFI.getTextSection();
  MCA.registerSection(TextSection);
  MCSection &DataSection = *OFI.getDataSection();
  MCA.registerSection(DataSection);
  MCSection &BSSSection = *OFI.getBSSSection();
  MCA.registerSection(BSSSection);

  TextSection.setAlignment(std::max(16u, TextSection.getAlignment()));
  DataSection.setAlignment(std::max(16u, DataSection.getAlignment()));
  BSSSection.setAlignment(std::max(16u, BSSSection.getAlignment()));

  if (RoundSectionSizes) {
    // Make sections sizes a multiple of the alignment. This is useful for
    // verifying the output of IAS against the output of other assemblers but
    // it's not necessary to produce a correct object and increases section
    // size.
    MCStreamer &OS = getStreamer();
    for (MCSection &S : MCA) {
      MCSectionELF &Section = static_cast<MCSectionELF &>(S);

      unsigned Alignment = Section.getAlignment();
      if (Alignment) {
        OS.SwitchSection(&Section);
        if (Section.UseCodeAlign())
          OS.EmitCodeAlignment(Alignment, Alignment);
        else
          OS.EmitValueToAlignment(Alignment, 0, 1, Alignment);
      }
    }
  }

  const FeatureBitset &Features = STI.getFeatureBits();

  // Update e_header flags. See the FIXME and comment above in
  // the constructor for a full rundown on this.
  unsigned EFlags = MCA.getELFHeaderEFlags();

  // ABI
  // N64 does not require any ABI bits.
  if (getABI().IsO32())
    EFlags |= ELF::EF_MIPS_ABI_O32;
  else if (getABI().IsN32())
    EFlags |= ELF::EF_MIPS_ABI2;

  if (Features[RISCV64::FeatureGP64Bit]) {
    if (getABI().IsO32())
      EFlags |= ELF::EF_MIPS_32BITMODE; /* Compatibility Mode */
  } else if (Features[RISCV64::FeatureRISCV6464r2] || Features[RISCV64::FeatureRISCV6464])
    EFlags |= ELF::EF_MIPS_32BITMODE;

  // -mplt is not implemented but we should act as if it was
  // given.
  if (!Features[RISCV64::FeatureNoABICalls])
    EFlags |= ELF::EF_MIPS_CPIC;

  if (Pic)
    EFlags |= ELF::EF_MIPS_PIC | ELF::EF_MIPS_CPIC;

  MCA.setELFHeaderEFlags(EFlags);

  // Emit all the option records.
  // At the moment we are only emitting .RISCV64.options (ODK_REGINFO) and
  // .reginfo.
  RISCV64ELFStreamer &MEF = static_cast<RISCV64ELFStreamer &>(Streamer);
  MEF.EmitRISCV64OptionRecords();

  emitRISCV64AbiFlags();
}

void RISCV64TargetELFStreamer::emitAssignment(MCSymbol *S, const MCExpr *Value) {
  auto *Symbol = cast<MCSymbolELF>(S);
  // If on rhs is micromips symbol then mark Symbol as microRISCV64.
  if (Value->getKind() != MCExpr::SymbolRef)
    return;
  const auto &RhsSym = cast<MCSymbolELF>(
      static_cast<const MCSymbolRefExpr *>(Value)->getSymbol());

  if (!(RhsSym.getOther() & ELF::STO_MIPS_MICROMIPS))
    return;

  Symbol->setOther(ELF::STO_MIPS_MICROMIPS);
}

MCELFStreamer &RISCV64TargetELFStreamer::getStreamer() {
  return static_cast<MCELFStreamer &>(Streamer);
}

void RISCV64TargetELFStreamer::emitDirectiveSetMicroRISCV64() {
  MicroRISCV64Enabled = true;
  forbidModuleDirective();
}

void RISCV64TargetELFStreamer::emitDirectiveSetNoMicroRISCV64() {
  MicroRISCV64Enabled = false;
  forbidModuleDirective();
}

void RISCV64TargetELFStreamer::setUsesMicroRISCV64() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_MICROMIPS;
  MCA.setELFHeaderEFlags(Flags);
}

void RISCV64TargetELFStreamer::emitDirectiveSetRISCV6416() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_ARCH_ASE_M16;
  MCA.setELFHeaderEFlags(Flags);
  forbidModuleDirective();
}

void RISCV64TargetELFStreamer::emitDirectiveSetNoReorder() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_NOREORDER;
  MCA.setELFHeaderEFlags(Flags);
  forbidModuleDirective();
}

void RISCV64TargetELFStreamer::emitDirectiveEnd(StringRef Name) {
  MCAssembler &MCA = getStreamer().getAssembler();
  MCContext &Context = MCA.getContext();
  MCStreamer &OS = getStreamer();

  MCSectionELF *Sec = Context.getELFSection(".pdr", ELF::SHT_PROGBITS, 0);

  MCSymbol *Sym = Context.getOrCreateSymbol(Name);
  const MCSymbolRefExpr *ExprRef =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, Context);

  MCA.registerSection(*Sec);
  Sec->setAlignment(4);

  OS.PushSection();

  OS.SwitchSection(Sec);

  OS.EmitValueImpl(ExprRef, 4);

  OS.EmitIntValue(GPRInfoSet ? GPRBitMask : 0, 4); // reg_mask
  OS.EmitIntValue(GPRInfoSet ? GPROffset : 0, 4);  // reg_offset

  OS.EmitIntValue(FPRInfoSet ? FPRBitMask : 0, 4); // fpreg_mask
  OS.EmitIntValue(FPRInfoSet ? FPROffset : 0, 4);  // fpreg_offset

  OS.EmitIntValue(FrameInfoSet ? FrameOffset : 0, 4); // frame_offset
  OS.EmitIntValue(FrameInfoSet ? FrameReg : 0, 4);    // frame_reg
  OS.EmitIntValue(FrameInfoSet ? ReturnReg : 0, 4);   // return_reg

  // The .end directive marks the end of a procedure. Invalidate
  // the information gathered up until this point.
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;

  OS.PopSection();

  // .end also implicitly sets the size.
  MCSymbol *CurPCSym = Context.createTempSymbol();
  OS.EmitLabel(CurPCSym);
  const MCExpr *Size = MCBinaryExpr::createSub(
      MCSymbolRefExpr::create(CurPCSym, MCSymbolRefExpr::VK_None, Context),
      ExprRef, Context);

  // The ELFObjectWriter can determine the absolute size as it has access to
  // the layout information of the assembly file, so a size expression rather
  // than an absolute value is ok here.
  static_cast<MCSymbolELF *>(Sym)->setSize(Size);
}

void RISCV64TargetELFStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;

  // .ent also acts like an implicit '.type symbol, STT_FUNC'
  static_cast<const MCSymbolELF &>(Symbol).setType(ELF::STT_FUNC);
}

void RISCV64TargetELFStreamer::emitDirectiveAbiCalls() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_CPIC | ELF::EF_MIPS_PIC;
  MCA.setELFHeaderEFlags(Flags);
}

void RISCV64TargetELFStreamer::emitDirectiveNaN2008() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_NAN2008;
  MCA.setELFHeaderEFlags(Flags);
}

void RISCV64TargetELFStreamer::emitDirectiveNaNLegacy() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags &= ~ELF::EF_MIPS_NAN2008;
  MCA.setELFHeaderEFlags(Flags);
}

void RISCV64TargetELFStreamer::emitDirectiveOptionPic0() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  // This option overrides other PIC options like -KPIC.
  Pic = false;
  Flags &= ~ELF::EF_MIPS_PIC;
  MCA.setELFHeaderEFlags(Flags);
}

void RISCV64TargetELFStreamer::emitDirectiveOptionPic2() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Pic = true;
  // NOTE: We are following the GAS behaviour here which means the directive
  // 'pic2' also sets the CPIC bit in the ELF header. This is different from
  // what is stated in the SYSV ABI which consider the bits EF_MIPS_PIC and
  // EF_MIPS_CPIC to be mutually exclusive.
  Flags |= ELF::EF_MIPS_PIC | ELF::EF_MIPS_CPIC;
  MCA.setELFHeaderEFlags(Flags);
}

void RISCV64TargetELFStreamer::emitDirectiveInsn() {
  RISCV64TargetStreamer::emitDirectiveInsn();
  RISCV64ELFStreamer &MEF = static_cast<RISCV64ELFStreamer &>(Streamer);
  MEF.createPendingLabelRelocs();
}

void RISCV64TargetELFStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                      unsigned ReturnReg_) {
  MCContext &Context = getStreamer().getAssembler().getContext();
  const MCRegisterInfo *RegInfo = Context.getRegisterInfo();

  FrameInfoSet = true;
  FrameReg = RegInfo->getEncodingValue(StackReg);
  FrameOffset = StackSize;
  ReturnReg = RegInfo->getEncodingValue(ReturnReg_);
}

void RISCV64TargetELFStreamer::emitMask(unsigned CPUBitmask,
                                     int CPUTopSavedRegOff) {
  GPRInfoSet = true;
  GPRBitMask = CPUBitmask;
  GPROffset = CPUTopSavedRegOff;
}

void RISCV64TargetELFStreamer::emitFMask(unsigned FPUBitmask,
                                      int FPUTopSavedRegOff) {
  FPRInfoSet = true;
  FPRBitMask = FPUBitmask;
  FPROffset = FPUTopSavedRegOff;
}

void RISCV64TargetELFStreamer::emitDirectiveCpLoad(unsigned RegNo) {
  // .cpload $reg
  // This directive expands to:
  // lui   $gp, %hi(_gp_disp)
  // addui $gp, $gp, %lo(_gp_disp)
  // addu  $gp, $gp, $reg
  // when support for position independent code is enabled.
  if (!Pic || (getABI().IsN32() || getABI().IsN64()))
    return;

  // There's a GNU extension controlled by -mno-shared that allows
  // locally-binding symbols to be accessed using absolute addresses.
  // This is currently not supported. When supported -mno-shared makes
  // .cpload expand to:
  //   lui     $gp, %hi(__gnu_local_gp)
  //   addiu   $gp, $gp, %lo(__gnu_local_gp)

  StringRef SymName("_gp_disp");
  MCAssembler &MCA = getStreamer().getAssembler();
  MCSymbol *GP_Disp = MCA.getContext().getOrCreateSymbol(SymName);
  MCA.registerSymbol(*GP_Disp);

  MCInst TmpInst;
  TmpInst.setOpcode(RISCV64::LUi);
  TmpInst.addOperand(MCOperand::createReg(RISCV64::GP));
  const MCExpr *HiSym = RISCV64MCExpr::create(
      RISCV64MCExpr::MEK_HI,
      MCSymbolRefExpr::create("_gp_disp", MCSymbolRefExpr::VK_None,
                              MCA.getContext()),
      MCA.getContext());
  TmpInst.addOperand(MCOperand::createExpr(HiSym));
  getStreamer().EmitInstruction(TmpInst, STI);

  TmpInst.clear();

  TmpInst.setOpcode(RISCV64::ADDiu);
  TmpInst.addOperand(MCOperand::createReg(RISCV64::GP));
  TmpInst.addOperand(MCOperand::createReg(RISCV64::GP));
  const MCExpr *LoSym = RISCV64MCExpr::create(
      RISCV64MCExpr::MEK_LO,
      MCSymbolRefExpr::create("_gp_disp", MCSymbolRefExpr::VK_None,
                              MCA.getContext()),
      MCA.getContext());
  TmpInst.addOperand(MCOperand::createExpr(LoSym));
  getStreamer().EmitInstruction(TmpInst, STI);

  TmpInst.clear();

  TmpInst.setOpcode(RISCV64::ADDu);
  TmpInst.addOperand(MCOperand::createReg(RISCV64::GP));
  TmpInst.addOperand(MCOperand::createReg(RISCV64::GP));
  TmpInst.addOperand(MCOperand::createReg(RegNo));
  getStreamer().EmitInstruction(TmpInst, STI);

  forbidModuleDirective();
}

bool RISCV64TargetELFStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  RISCV64TargetStreamer::emitDirectiveCpRestore(Offset, GetATReg, IDLoc, STI);
  // .cprestore offset
  // When PIC mode is enabled and the O32 ABI is used, this directive expands
  // to:
  //    sw $gp, offset($sp)
  // and adds a corresponding LW after every JAL.

  // Note that .cprestore is ignored if used with the N32 and N64 ABIs or if it
  // is used in non-PIC mode.
  if (!Pic || (getABI().IsN32() || getABI().IsN64()))
    return true;

  // Store the $gp on the stack.
  emitStoreWithImmOffset(RISCV64::SW, RISCV64::GP, RISCV64::SP, Offset, GetATReg, IDLoc,
                         STI);
  return true;
}

void RISCV64TargetELFStreamer::emitDirectiveCpsetup(unsigned RegNo,
                                                 int RegOrOffset,
                                                 const MCSymbol &Sym,
                                                 bool IsReg) {
  // Only N32 and N64 emit anything for .cpsetup iff PIC is set.
  if (!Pic || !(getABI().IsN32() || getABI().IsN64()))
    return;

  forbidModuleDirective();

  MCAssembler &MCA = getStreamer().getAssembler();
  MCInst Inst;

  // Either store the old $gp in a register or on the stack
  if (IsReg) {
    // move $save, $gpreg
    emitRRR(RISCV64::OR64, RegOrOffset, RISCV64::GP, RISCV64::ZERO, SMLoc(), &STI);
  } else {
    // sd $gpreg, offset($sp)
    emitRRI(RISCV64::SD, RISCV64::GP, RISCV64::SP, RegOrOffset, SMLoc(), &STI);
  }

  if (getABI().IsN32()) {
    MCSymbol *GPSym = MCA.getContext().getOrCreateSymbol("__gnu_local_gp");
    const RISCV64MCExpr *HiExpr = RISCV64MCExpr::create(
        RISCV64MCExpr::MEK_HI, MCSymbolRefExpr::create(GPSym, MCA.getContext()),
        MCA.getContext());
    const RISCV64MCExpr *LoExpr = RISCV64MCExpr::create(
        RISCV64MCExpr::MEK_LO, MCSymbolRefExpr::create(GPSym, MCA.getContext()),
        MCA.getContext());

    // lui $gp, %hi(__gnu_local_gp)
    emitRX(RISCV64::LUi, RISCV64::GP, MCOperand::createExpr(HiExpr), SMLoc(), &STI);

    // addiu  $gp, $gp, %lo(__gnu_local_gp)
    emitRRX(RISCV64::ADDiu, RISCV64::GP, RISCV64::GP, MCOperand::createExpr(LoExpr),
            SMLoc(), &STI);

    return;
  }

  const RISCV64MCExpr *HiExpr = RISCV64MCExpr::createGpOff(
      RISCV64MCExpr::MEK_HI, MCSymbolRefExpr::create(&Sym, MCA.getContext()),
      MCA.getContext());
  const RISCV64MCExpr *LoExpr = RISCV64MCExpr::createGpOff(
      RISCV64MCExpr::MEK_LO, MCSymbolRefExpr::create(&Sym, MCA.getContext()),
      MCA.getContext());

  // lui $gp, %hi(%neg(%gp_rel(funcSym)))
  emitRX(RISCV64::LUi, RISCV64::GP, MCOperand::createExpr(HiExpr), SMLoc(), &STI);

  // addiu  $gp, $gp, %lo(%neg(%gp_rel(funcSym)))
  emitRRX(RISCV64::ADDiu, RISCV64::GP, RISCV64::GP, MCOperand::createExpr(LoExpr),
          SMLoc(), &STI);

  // daddu  $gp, $gp, $funcreg
  emitRRR(RISCV64::DADDu, RISCV64::GP, RISCV64::GP, RegNo, SMLoc(), &STI);
}

void RISCV64TargetELFStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                                  bool SaveLocationIsRegister) {
  // Only N32 and N64 emit anything for .cpreturn iff PIC is set.
  if (!Pic || !(getABI().IsN32() || getABI().IsN64()))
    return;

  MCInst Inst;
  // Either restore the old $gp from a register or on the stack
  if (SaveLocationIsRegister) {
    Inst.setOpcode(RISCV64::OR);
    Inst.addOperand(MCOperand::createReg(RISCV64::GP));
    Inst.addOperand(MCOperand::createReg(SaveLocation));
    Inst.addOperand(MCOperand::createReg(RISCV64::ZERO));
  } else {
    Inst.setOpcode(RISCV64::LD);
    Inst.addOperand(MCOperand::createReg(RISCV64::GP));
    Inst.addOperand(MCOperand::createReg(RISCV64::SP));
    Inst.addOperand(MCOperand::createImm(SaveLocation));
  }
  getStreamer().EmitInstruction(Inst, STI);

  forbidModuleDirective();
}

void RISCV64TargetELFStreamer::emitRISCV64AbiFlags() {
  MCAssembler &MCA = getStreamer().getAssembler();
  MCContext &Context = MCA.getContext();
  MCStreamer &OS = getStreamer();
  MCSectionELF *Sec = Context.getELFSection(
      ".MIPS.abiflags", ELF::SHT_MIPS_ABIFLAGS, ELF::SHF_ALLOC, 24, "");
  MCA.registerSection(*Sec);
  Sec->setAlignment(8);
  OS.SwitchSection(Sec);

  OS << ABIFlagsSection;
}
