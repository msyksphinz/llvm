//===-- MYRISCVXMCInstLower.h - Lower MachineInstr to MCInst -------*- C++ -*--===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#pragma once

#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/Support/Compiler.h"
namespace llvm {
  class MCContext;
  class MCInst;
  class MCOperand;
  class MachineInstr;
  class MachineFunction;
  class MYRISCVXAsmPrinter;
  //@1 {
  /// This class is used to lower an MachineInstr into an MCInst.
  class LLVM_LIBRARY_VISIBILITY MYRISCVXMCInstLower {
    //@2
    typedef MachineOperand::MachineOperandType MachineOperandType;
    MCContext *Ctx;
    MYRISCVXAsmPrinter &AsmPrinter;
 private:
    MCOperand LowerSymbolOperand(const MachineOperand &MO,
                                 MachineOperandType MOTy, unsigned Offset) const;

 public:
    MYRISCVXMCInstLower(MYRISCVXAsmPrinter &asmprinter);
    void Initialize(MCContext* C);

    void Lower(const MachineInstr *MI, MCInst &OutMI) const;
    void LowerCPLOAD(SmallVector<MCInst, 4>& MCInsts);
    MCOperand LowerOperand(const MachineOperand& MO, unsigned offset = 0) const;
  };

}
