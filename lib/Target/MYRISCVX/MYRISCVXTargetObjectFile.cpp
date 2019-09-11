//===-- MYRISCVXTargetObjectFile.cpp - MYRISCVX Object Files ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXTargetObjectFile.h"

#include "MYRISCVXSubtarget.h"
#include "MYRISCVXTargetMachine.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Target/TargetMachine.h"

using namespace llvm;

void MYRISCVXTargetObjectFile::Initialize(MCContext &Ctx, const TargetMachine &TM)
{
  TargetLoweringObjectFileELF::Initialize(Ctx, TM);
  InitializeELF(TM.Options.UseInitArray);

  this->TM = &static_cast<const MYRISCVXTargetMachine &>(TM);

}
