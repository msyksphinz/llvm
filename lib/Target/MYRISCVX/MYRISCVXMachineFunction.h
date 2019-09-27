//===-- MYRISCVXMachineFunctionInfo.h - Private data used for MYRISCVX ----*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===------------------------------------------------------------------------------===//
//
// This file declares the MYRISCVX specific subclass of MachineFunctionInfo.
//
//===------------------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXMACHINEFUNCTION_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXMACHINEFUNCTION_H

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
        GlobalBaseReg(0),
        VarArgsFrameIndex(0)
  {}

  ~MYRISCVXFunctionInfo();

  int getVarArgsFrameIndex() const { return VarArgsFrameIndex; }
  void setVarArgsFrameIndex(int Index) { VarArgsFrameIndex = Index; }
  bool globalBaseRegFixed() const;
  bool globalBaseRegSet() const;
  unsigned getGlobalBaseReg();


 private:
  virtual void anchor();

  MachineFunction& MF;

  /// GlobalBaseReg - keeps track of the virtual register initialized for
  /// use as the global base register. This is used for PIC in some PIC
  /// relocation models.
  unsigned GlobalBaseReg;

  /// VarArgsFrameIndex - FrameIndex for start of varargs area.
  int VarArgsFrameIndex;

};
//@1 }

} // end of namespace llvm

#endif // MYRISCVX_MACHINE_FUNCTION_INFO_H
