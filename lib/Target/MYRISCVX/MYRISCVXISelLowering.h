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
  Sync,
  SELECT_CC
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

  // This function fills Ops, which is the list of operands that will later
  /// be used when a function call node is created. It also generates
  /// copyToReg nodes to set up argument registers.
  virtual void
  getOpndList(SmallVectorImpl<SDValue> &Ops,
              std::deque< std::pair<unsigned, SDValue> > &RegsToPass,
              bool IsPICCall, bool GlobalOrExternal, bool InternalLinkage,
              CallLoweringInfo &CLI, SDValue Callee, SDValue Chain) const;

  // Inline asm support
  ConstraintType getConstraintType(StringRef Constraint) const override;

  /// Examine constraint string and operand type and determine a weight value.
  /// The operand object must already have been set up with the operand type.
  ConstraintWeight getSingleConstraintMatchWeight(
      AsmOperandInfo &info, const char *constraint) const override;

  /// This function parses registers that appear in inline-asm constraints.
  /// It returns pair (0, 0) on failure.
  std::pair<unsigned, const TargetRegisterClass *>
  parseRegForInlineAsmConstraint(const StringRef &C, MVT VT) const;

  std::pair<unsigned, const TargetRegisterClass *>
  getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                               StringRef Constraint, MVT VT) const override;

  /// LowerAsmOperandForConstraint - Lower the specified operand into the Ops
  /// vector.  If it is invalid, don't add anything to Ops. If hasMemory is
  /// true it means one of the asm constraint of the inline asm instruction
  /// being processed is 'm'.
  void LowerAsmOperandForConstraint(SDValue Op,
                                    std::string &Constraint,
                                    std::vector<SDValue> &Ops,
                                    SelectionDAG &DAG) const override;

  bool isLegalAddressingMode(const DataLayout &DL, const AddrMode &AM,
                             Type *Ty, unsigned AS,
                             Instruction *I = nullptr) const override;

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

    void analyzeFormalArguments(const SmallVectorImpl<ISD::InputArg> &Ins,
                                bool IsSoftFloat,
                                Function::const_arg_iterator FuncArg);

    void analyzeReturn(const SmallVectorImpl<ISD::OutputArg> &Outs,
                       bool IsSoftFloat, const Type *RetTy) const;

    void analyzeCallOperands(const SmallVectorImpl<ISD::OutputArg> &Outs,
                             bool IsVarArg, bool IsSoftFloat,
                             const SDNode *CallNode,
                             std::vector<ArgListEntry> &FuncArgs);

    const CCState &getCCInfo() const { return CCInfo; }

    /// hasByValArg - Returns true if function has byval arguments.
    bool hasByValArg() const { return !ByValArgs.empty(); }

    /// regSize - Size (in number of bits) of integer registers.
    unsigned regSize() const { return IsO32 ? 4 : 4; }
    /// numIntArgRegs - Number of integer registers available for calls.
    unsigned numIntArgRegs() const;

    /// reservedArgArea - The size of the area the caller reserves for
    /// register arguments. This is 16-byte if ABI is O32.
    unsigned reservedArgArea() const;

    /// Return pointer to array of integer argument registers.
    const ArrayRef<MCPhysReg> intArgRegs() const;

    typedef SmallVectorImpl<ByValArgInfo>::const_iterator byval_iterator;

    byval_iterator byval_begin() const { return ByValArgs.begin(); }

    byval_iterator byval_end() const { return ByValArgs.end(); }

   private:
    void handleByValArg(unsigned ValNo, MVT ValVT, MVT LocVT,
                        CCValAssign::LocInfo LocInfo,
                        ISD::ArgFlagsTy ArgFlags);

    /// useRegsForByval - Returns true if the calling convention allows the
    /// use of registers to pass byval arguments.
    bool useRegsForByval() const { return CallConv != CallingConv::Fast; }

    /// Return the function that analyzes fixed argument list functions.
    llvm::CCAssignFn *fixedArgFn() const;

    /// Return the function that analyzes variable argument list functions.
    llvm::CCAssignFn *varArgFn() const;

    void allocateRegs(ByValArgInfo &ByVal, unsigned ByValSize,
                      unsigned Align);

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

  // Create a TargetBlockAddress node.
  SDValue getTargetNode(BlockAddressSDNode *N, EVT Ty, SelectionDAG &DAG,
                        unsigned Flag) const;

  // Create a TargetJumpTable node.
  SDValue getTargetNode(JumpTableSDNode *N, EVT Ty, SelectionDAG &DAG,
                        unsigned Flag) const;

  // Create a TargetExternalSymbol node.
  SDValue getTargetNode(ExternalSymbolSDNode *N, EVT Ty, SelectionDAG &DAG,
                        unsigned Flag) const;

  SDValue lowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerBlockAddress(SDValue Op, SelectionDAG &DAG) const;

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

  SDValue lowerSELECT(SDValue Op, SelectionDAG &DAG) const;

  SDValue lowerVASTART(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerRETURNADDR(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerEH_RETURN(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerADD(SDValue Op, SelectionDAG &DAG) const;

  /// isEligibleForTailCallOptimization - Check whether the call is eligible
  /// for tail call optimization.
  virtual bool
  isEligibleForTailCallOptimization(const MYRISCVXCC &MYRISCVXCCInfo,
                                    unsigned NextStackOffset,
                                    const MYRISCVXFunctionInfo& FI) const = 0;

  /// copyByValArg - Copy argument registers which were used to pass a byval
  /// argument to the stack. Create a stack frame object for the byval
  /// argument.
  void copyByValRegs(SDValue Chain, const SDLoc &DL,
                     std::vector<SDValue> &OutChains, SelectionDAG &DAG,
                     const ISD::ArgFlagsTy &Flags,
                     SmallVectorImpl<SDValue> &InVals,
                     const Argument *FuncArg,
                     const MYRISCVXCC &CC, const ByValArgInfo &ByVal) const;

  /// writeVarArgRegs - Write variable function arguments passed in registers
  /// to the stack. Also create a stack frame object for the first variable
  /// argument.
  void writeVarArgRegs(std::vector<SDValue> &OutChains, const MYRISCVXCC &CC,
                       SDValue Chain, const SDLoc &DL, SelectionDAG &DAG) const;


  SDValue LowerCall(TargetLowering::CallLoweringInfo &CLI,
                    SmallVectorImpl<SDValue> &InVals) const override;

  // Lower Operand helpers
  SDValue LowerCallResult(SDValue Chain, SDValue InFlag,
                          CallingConv::ID CallConv, bool isVarArg,
                          const SmallVectorImpl<ISD::InputArg> &Ins,
                          const SDLoc &dl, SelectionDAG &DAG,
                          SmallVectorImpl<SDValue> &InVals,
                          const SDNode *CallNode, const Type *RetTy) const;

  /// passByValArg - Pass a byval argument in registers or on stack.
  void passByValArg(SDValue Chain, const SDLoc &DL,
                    std::deque< std::pair<unsigned, SDValue> > &RegsToPass,
                    SmallVectorImpl<SDValue> &MemOpChains, SDValue StackPtr,
                    MachineFrameInfo &MFI, SelectionDAG &DAG, SDValue Arg,
                    const MYRISCVXCC &CC, const ByValArgInfo &ByVal,
                    const ISD::ArgFlagsTy &Flags, bool isLittle) const;

  SDValue passArgOnStack(SDValue StackPtr, unsigned Offset, SDValue Chain,
                         SDValue Arg, const SDLoc &DL, bool IsTailCall,
                         SelectionDAG &DAG) const;

  bool CanLowerReturn(CallingConv::ID CallConv, MachineFunction &MF,
                      bool isVarArg,
                      const SmallVectorImpl<ISD::OutputArg> &Outs,
                      LLVMContext &Context) const override;

  /// getSetCCResultType - get the ISD::SETCC result ValueType
  EVT getSetCCResultType(const DataLayout &DL, LLVMContext &Context,
                         EVT VT) const override;

  MachineBasicBlock *
  EmitInstrWithCustomInserter(MachineInstr &MI,
                              MachineBasicBlock *BB) const override;

  MYRISCVXCC::SpecialCallingConvType getSpecialCallingConv(SDValue Callee) const;

  /// If a physical register, this returns the register that receives the
  /// exception address on entry to an EH pad.
  unsigned
  getExceptionPointerRegister(const Constant *PersonalityFn) const override {
    return MYRISCVX::A0;
  }

  /// If a physical register, this returns the register that receives the
  /// exception typeid on entry to a landing pad.
  unsigned
  getExceptionSelectorRegister(const Constant *PersonalityFn) const override {
    return MYRISCVX::A1;
  }
  SDValue lowerGlobalTLSAddress(SDValue Op, SelectionDAG &DAG) const;
};


const MYRISCVXTargetLowering *
createMYRISCVXSETargetLowering(const MYRISCVXTargetMachine &TM,
                               const MYRISCVXSubtarget &STI);


}
