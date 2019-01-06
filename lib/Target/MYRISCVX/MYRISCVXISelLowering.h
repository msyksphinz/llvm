//===-- MYRISCVXISelLowering.h - MYRISCVX DAG Lowering Interface --------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that MYRISCVX uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MYRISCVX_MYRISCVXISELLOWERING_H
#define LLVM_LIB_TARGET_MYRISCVX_MYRISCVXISELLOWERING_H

#include "MCTargetDesc/MYRISCVXABIInfo.h"
#include "MYRISCVX.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/IR/Function.h"
#include "llvm/CodeGen/TargetLowering.h"
#include <deque>

namespace llvm {
  namespace MYRISCVXISD {
    enum NodeType {
      // Start the numbering from where ISD NodeType finishes.
      FIRST_NUMBER = ISD::BUILTIN_OP_END,

      // Jump and link (call)
      JmpLink,

      // Tail call
      TailCall,

      // Get the Higher 16 bits from a 32-bit immediate
      // No relation with MYRISCVX Hi register
      Hi,
      // Get the Lower 16 bits from a 32-bit immediate
      // No relation with MYRISCVX Lo register
      Lo,

      // Handle gp_rel (small data/bss sections) relocation.
      GPRel,

      // Thread Pointer
      ThreadPointer,

      // Return
      Ret,

      EH_RETURN,

      // DivRem(u)
      DivRem,
      DivRemU,

      Wrapper,
      DynAlloc,

      Sync
    };
  }

  //===--------------------------------------------------------------------===//
  // TargetLowering Implementation
  //===--------------------------------------------------------------------===//
  class MYRISCVXFunctionInfo;
  class MYRISCVXSubtarget;

  //@class MYRISCVXTargetLowering
  class MYRISCVXTargetLowering : public TargetLowering  {
  public:
    explicit MYRISCVXTargetLowering(const MYRISCVXTargetMachine &TM,
                                const MYRISCVXSubtarget &STI);

    static const MYRISCVXTargetLowering *create(const MYRISCVXTargetMachine &TM,
                                                const MYRISCVXSubtarget &STI);

    /// getTargetNodeName - This method returns the name of a target specific
    //  DAG node.
    const char *getTargetNodeName(unsigned Opcode) const override;

  protected:

    /// ByValArgInfo - Byval argument information.
    struct ByValArgInfo {
      unsigned FirstIdx; // Index of the first register used.
      unsigned NumRegs;  // Number of registers used for this argument.
      unsigned Address;  // Offset of the stack area used to pass this argument.

      ByValArgInfo() : FirstIdx(0), NumRegs(0), Address(0) {}
    };

  protected:
    // Subtarget Info
    const MYRISCVXSubtarget &Subtarget;
    // Cache the ABI from the TargetMachine, we use it everywhere.
    const MYRISCVXABIInfo &ABI;

  private:

    // Lower Operand specifics
    SDValue lowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const;

	//- must be exist even without function all
    SDValue
      LowerFormalArguments(SDValue Chain,
                           CallingConv::ID CallConv, bool IsVarArg,
                           const SmallVectorImpl<ISD::InputArg> &Ins,
                           const SDLoc &dl, SelectionDAG &DAG,
                           SmallVectorImpl<SDValue> &InVals) const override;

    SDValue LowerReturn(SDValue Chain,
                        CallingConv::ID CallConv, bool IsVarArg,
                        const SmallVectorImpl<ISD::OutputArg> &Outs,
                        const SmallVectorImpl<SDValue> &OutVals,
                        const SDLoc &dl, SelectionDAG &DAG) const override;

  };
  const MYRISCVXTargetLowering *
  createMYRISCVXSETargetLowering(const MYRISCVXTargetMachine &TM, const MYRISCVXSubtarget &STI);
}

#endif // MYRISCVXISELLOWERING_H
