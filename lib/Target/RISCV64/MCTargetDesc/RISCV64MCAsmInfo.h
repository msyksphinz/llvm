//===-- RISCV64MCAsmInfo.h - RISCV64 Asm Info ------------------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the RISCV64MCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MIPS_MCTARGETDESC_MIPSMCASMINFO_H
#define LLVM_LIB_TARGET_MIPS_MCTARGETDESC_MIPSMCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class Triple;

class RISCV64MCAsmInfo : public MCAsmInfoELF {
  void anchor() override;

public:
  explicit RISCV64MCAsmInfo(const Triple &TheTriple);
};

} // namespace llvm

#endif
