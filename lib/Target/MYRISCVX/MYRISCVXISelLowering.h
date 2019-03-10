//===-- MYRISCVXISelLowering.h - MYRISCVX DAG Lowering Interface --------*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
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

#pragma once

#include "MCTargetDesc/MYRISCVXABIInfo.h"
#include "MCTargetDesc/MYRISCVXBaseInfo.h"
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
class MYRISCVXTargetLowering : public TargetLowering {

 public:
  explicit MYRISCVXTargetLowering(const MYRISCVXTargetMachine &TM,
                                  const MYRISCVXSubtarget &STI);
  static const MYRISCVXTargetLowering *create(const MYRISCVXTargetMachine &TM,
                                              const MYRISCVXSubtarget &STI);
  /// getTargetNodeName - This method returns the name of a target specific
  // DAG node.
  const char *getTargetNodeName(unsigned Opcode) const override;

  bool isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const override;

 protected:
  SDValue getGlobalReg(SelectionDAG &DAG, EVT Ty) const;

  // Lower Operand specifics
  SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

  // This method creates the following nodes, which are necessary for
  // computing a local symbol's address:
  //
  // (add (load (wrapper $gp, %got(sym)), %lo(sym))
  template<class NodeTy>
  SDValue getAddrLocal(NodeTy *N, EVT Ty, SelectionDAG &DAG) const {
    SDLoc DL(N);
    unsigned GOTFlag = MYRISCVXII::MO_GOT;
    SDValue GOT = DAG.getNode(MYRISCVXISD::Wrapper, DL, Ty, getGlobalReg(DAG, Ty),
                              getTargetNode(N, Ty, DAG, GOTFlag));
    SDValue Load =
        DAG.getLoad(Ty, DL, DAG.getEntryNode(), GOT,
                    MachinePointerInfo::getGOT(DAG.getMachineFunction()));
    unsigned LoFlag = MYRISCVXII::MO_ABS_LO;
    SDValue Lo = DAG.getNode(MYRISCVXISD::Lo, DL, Ty,
                             getTargetNode(N, Ty, DAG, LoFlag));
    return DAG.getNode(ISD::ADD, DL, Ty, Load, Lo);
  }


  //@getAddrGlobal {
  // This method creates the following nodes, which are necessary for
  // computing a global symbol's address:
  //
  // (load (wrapper $gp, %got(sym)))
  template<class NodeTy>
  SDValue getAddrGlobal(NodeTy *N, EVT Ty, SelectionDAG &DAG,
                        unsigned Flag, SDValue Chain,
                        const MachinePointerInfo &PtrInfo) const {
    SDLoc DL(N);
    SDValue Tgt = DAG.getNode(MYRISCVXISD::Wrapper, DL, Ty, getGlobalReg(DAG, Ty),
                              getTargetNode(N, Ty, DAG, Flag));
    return DAG.getLoad(Ty, DL, Chain, Tgt, PtrInfo);
  }


  //@getAddrGlobal }
  //@getAddrGlobalLargeGOT {
  // This method creates the following nodes, which are necessary for
  // computing a global symbol's address in large-GOT mode:
  //
  // (load (wrapper (add %hi(sym), $gp), %lo(sym)))
  template<class NodeTy>
  SDValue getAddrGlobalLargeGOT(NodeTy *N, EVT Ty, SelectionDAG &DAG,
                                unsigned HiFlag, unsigned LoFlag,
                                SDValue Chain,
                                const MachinePointerInfo &PtrInfo) const {
    SDLoc DL(N);
    SDValue Hi = DAG.getNode(MYRISCVXISD::Hi, DL, Ty,
                             getTargetNode(N, Ty, DAG, HiFlag));
    Hi = DAG.getNode(ISD::ADD, DL, Ty, Hi, getGlobalReg(DAG, Ty));
    SDValue Wrapper = DAG.getNode(MYRISCVXISD::Wrapper, DL, Ty, Hi,
                                  getTargetNode(N, Ty, DAG, LoFlag));
    return DAG.getLoad(Ty, DL, Chain, Wrapper, PtrInfo);
  }


  //@getAddrGlobalLargeGOT }
  //@getAddrNonPIC
  // This method creates the following nodes, which are necessary for
  // computing a symbol's address in non-PIC mode:
  //
  // (add %hi(sym), %lo(sym))
  template<class NodeTy>
  SDValue getAddrNonPIC(NodeTy *N, EVT Ty, SelectionDAG &DAG) const {
    SDLoc DL(N);
    SDValue Hi = getTargetNode(N, Ty, DAG, MYRISCVXII::MO_ABS_HI);
    SDValue Lo = getTargetNode(N, Ty, DAG, MYRISCVXII::MO_ABS_LO);

    return DAG.getNode(ISD::ADD, DL, Ty,
                       DAG.getNode(MYRISCVXISD::Hi, DL, Ty, Hi),
                       DAG.getNode(MYRISCVXISD::Lo, DL, Ty, Lo));
  }

  /// ByValArgInfo - Byval argument information.
  struct ByValArgInfo {
    unsigned FirstIdx; // Index of the first register used.
    unsigned NumRegs; // Number of registers used for this argument.
    unsigned Address; // Offset of the stack area used to pass this argument.
    ByValArgInfo() : FirstIdx(0), NumRegs(0), Address(0) {}
  };

  /// MYRISCVXCC - This class provides methods used to analyze formal and call
  /// arguments and inquire about calling convention information.
  class MYRISCVXCC {
   public:
    enum SpecialCallingConvType {
      NoSpecialCallingConv
    };
    MYRISCVXCC(CallingConv::ID CallConv, bool IsO32, CCState &Info,
               SpecialCallingConvType SpecialCallingConv = NoSpecialCallingConv);

    void analyzeCallResult(const SmallVectorImpl<ISD::InputArg> &Ins,
                           bool IsSoftFloat, const SDNode *CallNode,
                           const Type *RetTy) const;

    void analyzeReturn(const SmallVectorImpl<ISD::OutputArg> &Outs,
                       bool IsSoftFloat, const Type *RetTy) const;

    const CCState &getCCInfo() const { return CCInfo; }

    /// hasByValArg - Returns true if function has byval arguments.
    bool hasByValArg() const { return !ByValArgs.empty(); }

    /// reservedArgArea - The size of the area the caller reserves for
    /// register arguments. This is 16-byte if ABI is O32.
    unsigned reservedArgArea() const;

    typedef SmallVectorImpl<ByValArgInfo>::const_iterator byval_iterator;

    byval_iterator byval_begin() const { return ByValArgs.begin(); }

    byval_iterator byval_end() const { return ByValArgs.end(); }

   private:
    /// Return the type of the register which is used to pass an argument or
    /// return a value. This function returns f64 if the argument is an i64
    /// value which has been generated as a result of softening an f128 value.
    /// Otherwise, it just returns VT.
    MVT getRegVT(MVT VT, const Type *OrigTy, const SDNode *CallNode,
                 bool IsSoftFloat) const;

    template<typename Ty>
    void analyzeReturn(const SmallVectorImpl<Ty> &RetVals, bool IsSoftFloat,
                       const SDNode *CallNode, const Type *RetTy) const;

    CCState &CCInfo;
    CallingConv::ID CallConv;
    bool IsO32;
    SmallVector<ByValArgInfo, 2> ByValArgs;
  };

 protected:
  // Subtarget Info
  const MYRISCVXSubtarget &Subtarget;
  // Cache the ABI from the TargetMachine, we use it everywhere.
  const MYRISCVXABIInfo &ABI;

 private:
  // Create a TargetGlobalAddress node.
  SDValue getTargetNode(GlobalAddressSDNode *N, EVT Ty, SelectionDAG &DAG,
                        unsigned Flag) const;

  // Create a TargetExternalSymbol node.
  SDValue getTargetNode(ExternalSymbolSDNode *N, EVT Ty, SelectionDAG &DAG,
                        unsigned Flag) const;

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

  /// getSetCCResultType - get the ISD::SETCC result ValueType
  EVT getSetCCResultType(const DataLayout &DL, LLVMContext &Context,
                         EVT VT) const override;

};


const MYRISCVXTargetLowering *
createMYRISCVXSETargetLowering(const MYRISCVXTargetMachine &TM, const MYRISCVXSubtarget &STI);


}
