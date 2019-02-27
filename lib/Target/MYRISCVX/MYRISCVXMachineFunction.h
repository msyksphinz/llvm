//===-- MYRISCVXMachineFunctionInfo.h - Private data used for MYRISCVX ----*- C++ -*-=//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the MYRISCVX specific subclass of MachineFunctionInfo.
//
//===----------------------------------------------------------------------===//

#pragma once

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"
#include <map>
namespace llvm {
  //@1 {
  /// MYRISCVXFunctionInfo - This class is derived from MachineFunction private
  /// MYRISCVX target-specific information for each MachineFunction.
  class MYRISCVXFunctionInfo : public MachineFunctionInfo {
 public:
 MYRISCVXFunctionInfo(MachineFunction& MF)
     : MF(MF),
       VarArgsFrameIndex(0),
       MaxCallFrameSize(0),
       GlobalBaseReg(0),
       SRetReturnReg(0), CallsEhReturn(false), CallsEhDwarf(false)
    {
    }
    ~MYRISCVXFunctionInfo();
    int getVarArgsFrameIndex() const { return VarArgsFrameIndex; }
    void setVarArgsFrameIndex(int Index) { VarArgsFrameIndex = Index; }

    unsigned getSRetReturnReg() const { return SRetReturnReg; }
    void setSRetReturnReg(unsigned Reg) { SRetReturnReg = Reg; }

    bool hasByvalArg() const { return HasByvalArg; }
    void setFormalArgInfo(unsigned Size, bool HasByval) {
      IncomingArgSize = Size;
      HasByvalArg = HasByval;
    }

    unsigned getIncomingArgSize() const { return IncomingArgSize; }

    bool callsEhReturn() const { return CallsEhReturn; }
    void setCallsEhReturn() { CallsEhReturn = true; }

    bool callsEhDwarf() const { return CallsEhDwarf; }
    void setCallsEhDwarf() { CallsEhDwarf = true; }

    void createEhDataRegsFI();
    int getEhDataRegFI(unsigned Reg) const { return EhDataRegFI[Reg]; }

    unsigned getMaxCallFrameSize() const { return MaxCallFrameSize; }
    void setMaxCallFrameSize(unsigned S) { MaxCallFrameSize = S; }

    bool globalBaseRegFixed() const;
    bool globalBaseRegSet() const;
    unsigned getGlobalBaseReg();

   private:
    virtual void anchor();
    MachineFunction& MF;
    /// VarArgsFrameIndex - FrameIndex for start of varargs area.
    int VarArgsFrameIndex;
    unsigned MaxCallFrameSize;

    /// GlobalBaseReg - keeps track of the virtual register initialized for
    /// use as the global base register. This is used for PIC in some PIC
    /// relocation models.
    unsigned GlobalBaseReg;
    int GPFI; // Index of the frame object for restoring $gp

    /// SRetReturnReg - Some subtargets require that sret lowering includes
    /// returning the value of the returned struct in a register. This field
    /// holds the virtual register into which the sret argument is passed.
    unsigned SRetReturnReg;

    /// True if function has a byval argument.
    bool HasByvalArg;
    /// Size of incoming argument area.
    unsigned IncomingArgSize;
    /// CallsEhReturn - Whether the function calls llvm.eh.return.
    bool CallsEhReturn;
    /// CallsEhDwarf - Whether the function calls llvm.eh.dwarf.
    bool CallsEhDwarf;
    /// Frame objects for spilling eh data registers.
    int EhDataRegFI[2];

  };
  //@1 }
} // end of namespace llvm
