//===-- Cpu0TargetStreamer.h - Cpu0 Target Streamer ------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXTARGETSTREAMER_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXTARGETSTREAMER_H

#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"

namespace llvm {

class MYRISCVXTargetStreamer : public MCTargetStreamer {
 public:
  MYRISCVXTargetStreamer(MCStreamer &S);
};

// This part is for ascii assembly output
class MYRISCVXTargetAsmStreamer : public MYRISCVXTargetStreamer {
  formatted_raw_ostream &OS;

 public:
  MYRISCVXTargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS);
};

}

#endif
