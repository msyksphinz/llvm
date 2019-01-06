//===-- MYRISCVXSERegisterInfo.cpp - CPU0 Register Information ------== -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the CPU0 implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXSERegisterInfo.h"

using namespace llvm;

#define DEBUG_TYPE "cpu0-reg-info"

MYRISCVXSERegisterInfo::MYRISCVXSERegisterInfo(const MYRISCVXSubtarget &ST)
  : MYRISCVXRegisterInfo(ST) {}

const TargetRegisterClass *
MYRISCVXSERegisterInfo::intRegClass(unsigned Size) const {
  return &MYRISCVX::CPURegsRegClass;
}
