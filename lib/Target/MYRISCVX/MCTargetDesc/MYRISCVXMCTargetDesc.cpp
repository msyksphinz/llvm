//===-- MYRISCVXMCTargetDesc.cpp - MYRISCVX Target Descriptions ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides MYRISCVX specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXMCTargetDesc.h"
#include "MYRISCVXTargetStreamer.h"
#include "InstPrinter/MYRISCVXInstPrinter.h"
#include "MYRISCVXMCAsmInfo.h"

#include "llvm/MC/MachineLocation.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstPrinter.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "MYRISCVXGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "MYRISCVXGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "MYRISCVXGenRegisterInfo.inc"

/// Select the MYRISCVX Architecture Feature for the given triple and cpu name.
/// The function will be called at command 'llvm-objdump -d' for MYRISCVX elf input.
static StringRef selectMYRISCVXArchFeature(const Triple &TT, StringRef CPU) {
  if (TT.getArch() == Triple::ArchType::myriscvx64) {
    return "+64bit";
  } else if (TT.getArch() == Triple::ArchType::myriscvx32) {
    return "-64bit";
  }
  return "+simple32";
}
//@1 }

static MCInstrInfo *createMYRISCVXMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitMYRISCVXMCInstrInfo(X); // defined in MYRISCVXGenInstrInfo.inc
  return X;
}

static MCRegisterInfo *createMYRISCVXMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitMYRISCVXMCRegisterInfo(X, MYRISCVX::RA); // defined in MYRISCVXGenRegisterInfo.inc
  return X;
}

static MCSubtargetInfo *createMYRISCVXMCSubtargetInfo(const Triple &TT,
                                                      StringRef CPU, StringRef FS) {
  std::string ArchFS = selectMYRISCVXArchFeature(TT, CPU);

  if (!FS.empty()) {
    if (!ArchFS.empty())
      ArchFS = ArchFS + "," + FS.str();
    else
      ArchFS = FS;
  }
  return createMYRISCVXMCSubtargetInfoImpl(TT, CPU, ArchFS);
  // createMYRISCVXMCSubtargetInfoImpl defined in MYRISCVXGenSubtargetInfo.inc
}

static MCAsmInfo *createMYRISCVXMCAsmInfo(const MCRegisterInfo &MRI,
                                          const Triple &TT) {
  MCAsmInfo *MAI = new MYRISCVXMCAsmInfo(TT);

  unsigned SP = MRI.getDwarfRegNum(MYRISCVX::SP, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, SP, 0);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCInstPrinter *createMYRISCVXMCInstPrinter(const Triple &T,
                                                  unsigned SyntaxVariant,
                                                  const MCAsmInfo &MAI,
                                                  const MCInstrInfo &MII,
                                                  const MCRegisterInfo &MRI) {
  return new MYRISCVXInstPrinter(MAI, MII, MRI);
}

namespace {

class MYRISCVXMCInstrAnalysis : public MCInstrAnalysis {
 public:
  MYRISCVXMCInstrAnalysis(const MCInstrInfo *Info) : MCInstrAnalysis(Info) {}
};
}

static MCInstrAnalysis *createMYRISCVXMCInstrAnalysis(const MCInstrInfo *Info) {
  return new MYRISCVXMCInstrAnalysis(Info);
}


static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    std::unique_ptr<MCObjectWriter> &&OW,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  return createELFStreamer(Context, std::move(MAB), std::move(OW),
                           std::move(Emitter), RelaxAll);
}


static MCTargetStreamer *createMYRISCVXAsmTargetStreamer(MCStreamer &S,
                                                         formatted_raw_ostream &OS,
                                                         MCInstPrinter *InstPrint,
                                                         bool isVerboseAsm) {
  return new MYRISCVXTargetAsmStreamer(S, OS);
}


//@2 {
extern "C" void LLVMInitializeMYRISCVXTargetMC() {
  for (Target *T : {&getTheMYRISCVX32Target(), &getTheMYRISCVX64Target()}) {
    // Register the MC asm info.
    RegisterMCAsmInfoFn X(*T, createMYRISCVXMCAsmInfo);

    // Register the MC instruction info.
    TargetRegistry::RegisterMCInstrInfo(*T, createMYRISCVXMCInstrInfo);

    // Register the MC register info.
    TargetRegistry::RegisterMCRegInfo(*T, createMYRISCVXMCRegisterInfo);

    // Register the MC subtarget info.
    TargetRegistry::RegisterMCSubtargetInfo(*T,
	                                        createMYRISCVXMCSubtargetInfo);
    // Register the MC instruction analyzer.
    TargetRegistry::RegisterMCInstrAnalysis(*T, createMYRISCVXMCInstrAnalysis);
    // Register the MCInstPrinter.
    TargetRegistry::RegisterMCInstPrinter(*T,
	                                      createMYRISCVXMCInstPrinter);
    // Register the elf streamer.
    TargetRegistry::RegisterELFStreamer(*T, createMCStreamer);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createMYRISCVXAsmTargetStreamer);

    // Register the MC Code Emitter
    TargetRegistry::RegisterMCCodeEmitter(*T,
                                          createMYRISCVXMCCodeEmitterEL);

    // Register the asm backend.
    TargetRegistry::RegisterMCAsmBackend(*T,
                                         createMYRISCVXAsmBackendEL32);
  }

}
//@2 }
