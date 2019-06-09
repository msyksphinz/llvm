//===-- MYRISCVXTargetStreamer.cpp - MYRISCVX Target Streamer Methods -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides MYRISCVX specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "InstPrinter/MYRISCVXInstPrinter.h"
#include "MYRISCVXMCTargetDesc.h"
#include "MYRISCVXTargetObjectFile.h"
#include "MCTargetDesc/MYRISCVXTargetStreamer.h"

#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

MYRISCVXTargetStreamer::MYRISCVXTargetStreamer(MCStreamer &S)
    : MCTargetStreamer(S) {
}

MYRISCVXTargetAsmStreamer::MYRISCVXTargetAsmStreamer(MCStreamer &S,
                                                     formatted_raw_ostream &OS)
    : MYRISCVXTargetStreamer(S), OS(OS) {}
