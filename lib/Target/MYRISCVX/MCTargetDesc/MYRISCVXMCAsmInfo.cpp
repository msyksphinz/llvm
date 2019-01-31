//===-- MYRISCVXMCAsmInfo.cpp - MYRISCVX Asm Properties ---------------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MYRISCVXMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "llvm/ADT/Triple.h"
#include "MYRISCVXMCAsmInfo.h"

using namespace llvm;

void MYRISCVXMCAsmInfo::anchor() { }

MYRISCVXMCAsmInfo::MYRISCVXMCAsmInfo(const Triple &TheTriple) {
  IsLittleEndian = true; // the default of IsLittleEndian is true
  AlignmentIsInBytes = false;
  Data16bitsDirective = "\t.2byte\t";
  Data32bitsDirective = "\t.4byte\t";
  Data64bitsDirective = "\t.8byte\t";
  PrivateGlobalPrefix = "$";
  // PrivateLabelPrefix: display $BB for the labels of basic block
  PrivateLabelPrefix = "$";
  CommentString = "#";
  ZeroDirective = "\t.space\t";
  GPRel32Directive = "\t.gpword\t";
  GPRel64Directive = "\t.gpdword\t";
  WeakRefDirective = "\t.weak\t";
  UseAssignmentForEHBegin = true;
  SupportsDebugInformation = true;
  ExceptionsType = ExceptionHandling::DwarfCFI;
  DwarfRegNumForCFI = true;
}
