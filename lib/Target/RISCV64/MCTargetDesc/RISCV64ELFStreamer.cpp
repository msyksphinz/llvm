//===-------- RISCV64ELFStreamer.cpp - ELF Object Output ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "RISCV64ELFStreamer.h"
#include "RISCV64OptionRecord.h"
#include "RISCV64TargetStreamer.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDwarf.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"

using namespace llvm;

RISCV64ELFStreamer::RISCV64ELFStreamer(MCContext &Context,
                                 std::unique_ptr<MCAsmBackend> MAB,
                                 std::unique_ptr<MCObjectWriter> OW,
                                 std::unique_ptr<MCCodeEmitter> Emitter)
    : MCELFStreamer(Context, std::move(MAB), std::move(OW),
                    std::move(Emitter)) {
  RegInfoRecord = new RISCV64RegInfoRecord(this, Context);
  RISCV64OptionRecords.push_back(
      std::unique_ptr<RISCV64RegInfoRecord>(RegInfoRecord));
}

void RISCV64ELFStreamer::EmitInstruction(const MCInst &Inst,
                                      const MCSubtargetInfo &STI, bool) {
  MCELFStreamer::EmitInstruction(Inst, STI);

  MCContext &Context = getContext();
  const MCRegisterInfo *MCRegInfo = Context.getRegisterInfo();

  for (unsigned OpIndex = 0; OpIndex < Inst.getNumOperands(); ++OpIndex) {
    const MCOperand &Op = Inst.getOperand(OpIndex);

    if (!Op.isReg())
      continue;

    unsigned Reg = Op.getReg();
    RegInfoRecord->SetPhysRegUsed(Reg, MCRegInfo);
  }

  createPendingLabelRelocs();
}

void RISCV64ELFStreamer::EmitCFIStartProcImpl(MCDwarfFrameInfo &Frame) {
  Frame.Begin = getContext().createTempSymbol();
  MCELFStreamer::EmitLabel(Frame.Begin);
}

MCSymbol *RISCV64ELFStreamer::EmitCFILabel() {
  MCSymbol *Label = getContext().createTempSymbol("cfi", true);
  MCELFStreamer::EmitLabel(Label);
  return Label;
}

void RISCV64ELFStreamer::EmitCFIEndProcImpl(MCDwarfFrameInfo &Frame) {
  Frame.End = getContext().createTempSymbol();
  MCELFStreamer::EmitLabel(Frame.End);
}

void RISCV64ELFStreamer::createPendingLabelRelocs() {
  RISCV64TargetELFStreamer *ELFTargetStreamer =
      static_cast<RISCV64TargetELFStreamer *>(getTargetStreamer());

  // FIXME: Also mark labels when in MIPS16 mode.
  if (ELFTargetStreamer->isMicroRISCV64Enabled()) {
    for (auto *L : Labels) {
      auto *Label = cast<MCSymbolELF>(L);
      getAssembler().registerSymbol(*Label);
      Label->setOther(ELF::STO_MIPS_MICROMIPS);
    }
  }

  Labels.clear();
}

void RISCV64ELFStreamer::EmitLabel(MCSymbol *Symbol, SMLoc Loc) {
  MCELFStreamer::EmitLabel(Symbol);
  Labels.push_back(Symbol);
}

void RISCV64ELFStreamer::SwitchSection(MCSection *Section,
                                    const MCExpr *Subsection) {
  MCELFStreamer::SwitchSection(Section, Subsection);
  Labels.clear();
}

void RISCV64ELFStreamer::EmitValueImpl(const MCExpr *Value, unsigned Size,
                                    SMLoc Loc) {
  MCELFStreamer::EmitValueImpl(Value, Size, Loc);
  Labels.clear();
}

void RISCV64ELFStreamer::EmitIntValue(uint64_t Value, unsigned Size) {
  MCELFStreamer::EmitIntValue(Value, Size);
  Labels.clear();
}

void RISCV64ELFStreamer::EmitRISCV64OptionRecords() {
  for (const auto &I : RISCV64OptionRecords)
    I->EmitRISCV64OptionRecord();
}

MCELFStreamer *llvm::createRISCV64ELFStreamer(
    MCContext &Context, std::unique_ptr<MCAsmBackend> MAB,
    std::unique_ptr<MCObjectWriter> OW, std::unique_ptr<MCCodeEmitter> Emitter,
    bool RelaxAll) {
  return new RISCV64ELFStreamer(Context, std::move(MAB), std::move(OW),
                             std::move(Emitter));
}
