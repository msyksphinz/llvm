//===-- RISCV64MCTargetDesc.cpp - RISCV64 Target Descriptions -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides RISCV64 specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "RISCV64MCTargetDesc.h"
#include "InstPrinter/RISCV64InstPrinter.h"
#include "RISCV64AsmBackend.h"
#include "RISCV64ELFStreamer.h"
#include "RISCV64MCAsmInfo.h"
#include "RISCV64MCNaCl.h"
#include "RISCV64TargetStreamer.h"
#include "llvm/ADT/Triple.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "RISCV64GenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "RISCV64GenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "RISCV64GenRegisterInfo.inc"

/// Select the RISCV64 CPU for the given triple and cpu name.
/// FIXME: Merge with the copy in RISCV64Subtarget.cpp
StringRef MIPS_MC::selectRISCV64CPU(const Triple &TT, StringRef CPU) {
  if (CPU.empty() || CPU == "generic") {
    if (TT.getSubArch() == llvm::Triple::RISCV64SubArch_r6) {
      if (TT.isMIPS32())
        CPU = "mips32r6";
      else
        CPU = "mips64r6";
    } else {
      if (TT.isMIPS32())
        CPU = "mips32";
      else
        CPU = "mips64";
    }
  }
  return CPU;
}

static MCInstrInfo *createRISCV64MCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitRISCV64MCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createRISCV64MCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitRISCV64MCRegisterInfo(X, RISCV64::RA);
  return X;
}

static MCSubtargetInfo *createRISCV64MCSubtargetInfo(const Triple &TT,
                                                  StringRef CPU, StringRef FS) {
  CPU = MIPS_MC::selectRISCV64CPU(TT, CPU);
  return createRISCV64MCSubtargetInfoImpl(TT, CPU, FS);
}

static MCAsmInfo *createRISCV64MCAsmInfo(const MCRegisterInfo &MRI,
                                      const Triple &TT) {
  MCAsmInfo *MAI = new RISCV64MCAsmInfo(TT);

  unsigned SP = MRI.getDwarfRegNum(RISCV64::SP, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, SP, 0);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCInstPrinter *createRISCV64MCInstPrinter(const Triple &T,
                                              unsigned SyntaxVariant,
                                              const MCAsmInfo &MAI,
                                              const MCInstrInfo &MII,
                                              const MCRegisterInfo &MRI) {
  return new RISCV64InstPrinter(MAI, MII, MRI);
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    std::unique_ptr<MCObjectWriter> &&OW,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  MCStreamer *S;
  if (!T.isOSNaCl())
    S = createRISCV64ELFStreamer(Context, std::move(MAB), std::move(OW),
                              std::move(Emitter), RelaxAll);
  else
    S = createRISCV64NaClELFStreamer(Context, std::move(MAB), std::move(OW),
                                  std::move(Emitter), RelaxAll);
  return S;
}

static MCTargetStreamer *createRISCV64AsmTargetStreamer(MCStreamer &S,
                                                     formatted_raw_ostream &OS,
                                                     MCInstPrinter *InstPrint,
                                                     bool isVerboseAsm) {
  return new RISCV64TargetAsmStreamer(S, OS);
}

static MCTargetStreamer *createRISCV64NullTargetStreamer(MCStreamer &S) {
  return new RISCV64TargetStreamer(S);
}

static MCTargetStreamer *
createRISCV64ObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new RISCV64TargetELFStreamer(S, STI);
}

namespace {

class RISCV64MCInstrAnalysis : public MCInstrAnalysis {
public:
  RISCV64MCInstrAnalysis(const MCInstrInfo *Info) : MCInstrAnalysis(Info) {}

  bool evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size,
                      uint64_t &Target) const override {
    unsigned NumOps = Inst.getNumOperands();
    if (NumOps == 0)
      return false;
    switch (Info->get(Inst.getOpcode()).OpInfo[NumOps - 1].OperandType) {
    case MCOI::OPERAND_UNKNOWN:
    case MCOI::OPERAND_IMMEDIATE:
      // jal, bal ...
      Target = Inst.getOperand(NumOps - 1).getImm();
      return true;
    case MCOI::OPERAND_PCREL:
      // b, j, beq ...
      Target = Addr + Inst.getOperand(NumOps - 1).getImm();
      return true;
    default:
      return false;
    }
  }
};
}

static MCInstrAnalysis *createRISCV64MCInstrAnalysis(const MCInstrInfo *Info) {
  return new RISCV64MCInstrAnalysis(Info);
}

extern "C" void LLVMInitializeRISCV64TargetMC() {
  for (Target *T : {&getTheRISCV64Target(), &getTheRISCV64elTarget(),
                    &getTheRISCV6464Target(), &getTheRISCV6464elTarget()}) {
    // Register the MC asm info.
    RegisterMCAsmInfoFn X(*T, createRISCV64MCAsmInfo);

    // Register the MC instruction info.
    TargetRegistry::RegisterMCInstrInfo(*T, createRISCV64MCInstrInfo);

    // Register the MC register info.
    TargetRegistry::RegisterMCRegInfo(*T, createRISCV64MCRegisterInfo);

    // Register the elf streamer.
    TargetRegistry::RegisterELFStreamer(*T, createMCStreamer);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createRISCV64AsmTargetStreamer);

    TargetRegistry::RegisterNullTargetStreamer(*T,
                                               createRISCV64NullTargetStreamer);

    // Register the MC subtarget info.
    TargetRegistry::RegisterMCSubtargetInfo(*T, createRISCV64MCSubtargetInfo);

    // Register the MC instruction analyzer.
    TargetRegistry::RegisterMCInstrAnalysis(*T, createRISCV64MCInstrAnalysis);

    // Register the MCInstPrinter.
    TargetRegistry::RegisterMCInstPrinter(*T, createRISCV64MCInstPrinter);

    TargetRegistry::RegisterObjectTargetStreamer(
        *T, createRISCV64ObjectTargetStreamer);

    // Register the asm backend.
    TargetRegistry::RegisterMCAsmBackend(*T, createRISCV64AsmBackend);
  }

  // Register the MC Code Emitter
  for (Target *T : {&getTheRISCV64Target(), &getTheRISCV6464Target()})
    TargetRegistry::RegisterMCCodeEmitter(*T, createRISCV64MCCodeEmitterEB);

  for (Target *T : {&getTheRISCV64elTarget(), &getTheRISCV6464elTarget()})
    TargetRegistry::RegisterMCCodeEmitter(*T, createRISCV64MCCodeEmitterEL);
}
