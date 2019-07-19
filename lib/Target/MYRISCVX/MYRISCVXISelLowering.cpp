//===-- MYRISCVXISelLowering.cpp - MYRISCVX DAG Lowering Implementation ---===//
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
#include "MYRISCVXISelLowering.h"

#include "MYRISCVXMachineFunction.h"
#include "MYRISCVXTargetMachine.h"
#include "MYRISCVXTargetObjectFile.h"
#include "MYRISCVXSubtarget.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "MYRISCVX-lower"

// static cl::opt<bool>
// EnableMYRISCVXTailCalls("enable-MYRISCVX-tail-calls", cl::Hidden,
//                         cl::desc("MYRISCVX: Enable tail calls."), cl::init(true));

STATISTIC(NumTailCalls, "Number of tail calls");

//@3_1 1 {
const char *MYRISCVXTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
    case MYRISCVXISD::CALL:              return "MYRISCVXISD::CALL";
    case MYRISCVXISD::TailCall:          return "MYRISCVXISD::TailCall";
    case MYRISCVXISD::Hi:                return "MYRISCVXISD::Hi";
    case MYRISCVXISD::Lo:                return "MYRISCVXISD::Lo";
    case MYRISCVXISD::GPRel:             return "MYRISCVXISD::GPRel";
    case MYRISCVXISD::Ret:               return "MYRISCVXISD::Ret";
    case MYRISCVXISD::EH_RETURN:         return "MYRISCVXISD::EH_RETURN";
    case MYRISCVXISD::DivRem:            return "MYRISCVXISD::DivRem";
    case MYRISCVXISD::DivRemU:           return "MYRISCVXISD::DivRemU";
    case MYRISCVXISD::Wrapper:           return "MYRISCVXISD::Wrapper";
    case MYRISCVXISD::SELECT_CC:         return "MYRISCVXISD::SELECT_CC";
    default:                             return NULL;
  }
}
//@3_1 1 }

//@MYRISCVXTargetLowering {
MYRISCVXTargetLowering::MYRISCVXTargetLowering(const MYRISCVXTargetMachine &TM,
                                               const MYRISCVXSubtarget &STI)
    : TargetLowering(TM), Subtarget(STI), ABI(TM.getABI()) {
  // MYRISCVX does not have i1 type, so use i32 for
  // setcc operations results (slt, sgt, ...).
  setBooleanContents(ZeroOrOneBooleanContent);
  setBooleanVectorContents(ZeroOrNegativeOneBooleanContent);

  setOperationAction(ISD::GlobalAddress,      MVT::i32,   Custom);

  // Load extented operations for i1 types must be promoted
  for (MVT VT : MVT::integer_valuetypes()) {
    setLoadExtAction(ISD::EXTLOAD,  VT, MVT::i1,  Promote);
    setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1,  Promote);
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1,  Promote);
  }

  // Branch Instructions
  setOperationAction(ISD::BR_CC, MVT::i32, Expand);
  setOperationAction(ISD::BR_JT,     MVT::Other, Expand);

  // Effectively disable jump table generation.
  setMinimumJumpTableEntries(INT_MAX);

  setOperationAction(ISD::SELECT,    MVT::i32, Custom);
  setOperationAction(ISD::SELECT_CC, MVT::i32, Expand);

  // For Var Arguments
  setOperationAction(ISD::VASTART,   MVT::Other, Custom);

  // Support va_arg(): variable numbers (not fixed numbers) of arguments
  //  (parameters) for function all
  setOperationAction(ISD::VAARG,        MVT::Other, Expand);
  setOperationAction(ISD::VACOPY,       MVT::Other, Expand);
  setOperationAction(ISD::VAEND,        MVT::Other, Expand);

}

const MYRISCVXTargetLowering *MYRISCVXTargetLowering::create(const MYRISCVXTargetMachine &TM,
                                                             const MYRISCVXSubtarget &STI) {
  return llvm::createMYRISCVXSETargetLowering(TM, STI);
}

//===----------------------------------------------------------------------===//
//  Lower helper functions
//===----------------------------------------------------------------------===//

// addLiveIn - This helper function adds the specified physical register to the
// MachineFunction as a live in value.  It also creates a corresponding
// virtual register for it.
static unsigned
addLiveIn(MachineFunction &MF, unsigned PReg, const TargetRegisterClass *RC)
{
  unsigned VReg = MF.getRegInfo().createVirtualRegister(RC);
  MF.getRegInfo().addLiveIn(PReg, VReg);
  return VReg;
}


//===----------------------------------------------------------------------===//
//  Misc Lower Operation implementation
//===----------------------------------------------------------------------===//

#include "MYRISCVXGenCallingConv.inc"

//===----------------------------------------------------------------------===//
//@              Return Value Calling Convention Implementation
//===----------------------------------------------------------------------===//

SDValue
MYRISCVXTargetLowering::LowerReturn(SDValue Chain,
                                    CallingConv::ID CallConv, bool IsVarArg,
                                    const SmallVectorImpl<ISD::OutputArg> &Outs,
                                    const SmallVectorImpl<SDValue> &OutVals,
                                    const SDLoc &DL, SelectionDAG &DAG) const {
  // CCValAssign - represent the assignment of
  // the return value to a location
  SmallVector<CCValAssign, 16> RVLocs;
  MachineFunction &MF = DAG.getMachineFunction();

  // CCState - Info about the registers and stack slot.
  CCState CCInfo(CallConv, IsVarArg, MF, RVLocs,
                 *DAG.getContext());
  MYRISCVXCC MYRISCVXCCInfo(CallConv, ABI.IsLP32(), CCInfo);


  // Analyze return values.
  MYRISCVXCCInfo.analyzeReturn(Outs, Subtarget.abiUsesSoftFloat(),
                               MF.getFunction().getReturnType());

  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  // Copy the result values into the output registers.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    SDValue Val = OutVals[i];
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    if (RVLocs[i].getValVT() != RVLocs[i].getLocVT())
      Val = DAG.getNode(ISD::BITCAST, DL, RVLocs[i].getLocVT(), Val);

    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(), Val, Flag);

    // Guarantee that all emitted copies are stuck together with flags.
    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  //@Ordinary struct type: 2 {
  // The MYRISCVX ABIs for returning structs by value requires that we copy
  // the sret argument into $a0 for the return. We saved the argument into
  // a virtual register in the entry block, so now we copy the value out
  // and into $a0.
  if (MF.getFunction().hasStructRetAttr()) {
    MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();
    unsigned Reg = MYRISCVXFI->getSRetReturnReg();

    if (!Reg)
      llvm_unreachable("sret virtual register not created in the entry block");
    SDValue Val =
        DAG.getCopyFromReg(Chain, DL, Reg, getPointerTy(DAG.getDataLayout()));
    unsigned A0 = MYRISCVX::A0;

    Chain = DAG.getCopyToReg(Chain, DL, A0, Val, Flag);
    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(A0, getPointerTy(DAG.getDataLayout())));
  }
  //@Ordinary struct type: 2 }

  RetOps[0] = Chain;  // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode())
    RetOps.push_back(Flag);

  // Return on MYRISCVX is always a "ret $ra"
  return DAG.getNode(MYRISCVXISD::Ret, DL, MVT::Other, RetOps);
}


SDValue MYRISCVXTargetLowering::
LowerOperation(SDValue Op, SelectionDAG &DAG) const
{
  switch (Op.getOpcode())
  {
    case ISD::SELECT        : return lowerSELECT(Op, DAG);
    case ISD::GlobalAddress : return lowerGlobalAddress(Op, DAG);
    case ISD::VASTART       : return lowerVASTART(Op, DAG);
  }
  return SDValue();
}


SDValue MYRISCVXTargetLowering::lowerGlobalAddress(SDValue Op,
                                                   SelectionDAG &DAG) const {
  //@lowerGlobalAddress }
  SDLoc DL(Op);
  const MYRISCVXTargetObjectFile *TLOF =
      static_cast<const MYRISCVXTargetObjectFile *>(
          getTargetMachine().getObjFileLowering());
  //@lga 1 {
  EVT Ty = Op.getValueType();
  GlobalAddressSDNode *N = cast<GlobalAddressSDNode>(Op);
  const GlobalValue *GV = N->getGlobal();
  //@lga 1 }

  if (!isPositionIndependent()) {
    const GlobalObject *GO = GV->getBaseObject();
    //@ %gp_rel relocation
    if (TLOF->IsGlobalInSmallSection(GO, getTargetMachine())) {
      SDValue GA = DAG.getTargetGlobalAddress(GO, DL, MVT::i32, 0,
                                              MYRISCVXII::MO_GPREL);
      SDValue GPRelNode = DAG.getNode(MYRISCVXISD::GPRel, DL,
                                      DAG.getVTList(MVT::i32), GA);
      SDValue GPReg = DAG.getRegister(MYRISCVX::GP, MVT::i32);
      return DAG.getNode(ISD::ADD, DL, MVT::i32, GPReg, GPRelNode);
    }

    //@ %hi/%lo relocation
    return getAddrNonPIC(N, Ty, DAG);
  }

  if (GV->hasInternalLinkage() || (GV->hasLocalLinkage() && !isa<Function>(GV)))
    return getAddrLocal(N, Ty, DAG);

  //@large section
  const GlobalObject *GO = GV->getBaseObject();
  if (!TLOF->IsGlobalInSmallSection(GO, getTargetMachine()))
    return getAddrGlobalLargeGOT(
        N, Ty, DAG, MYRISCVXII::MO_GOT_HI20, MYRISCVXII::MO_GPREL,
        DAG.getEntryNode(),
        MachinePointerInfo::getGOT(DAG.getMachineFunction()));
  return getAddrGlobal(
      N, Ty, DAG, MYRISCVXII::MO_GOT, DAG.getEntryNode(),
      MachinePointerInfo::getGOT(DAG.getMachineFunction()));
}


SDValue MYRISCVXTargetLowering::
lowerSELECT(SDValue Op, SelectionDAG &DAG) const
{
  SDValue CondV = Op.getOperand(0);
  SDValue TrueV = Op.getOperand(1);
  SDValue FalseV = Op.getOperand(2);
  SDLoc DL(Op);

  // (select condv, truev, falsev)
  // -> (myriscvxisd::select_cc condv, zero, setne, truev, falsev)
  SDValue Zero = DAG.getConstant(0, DL, MVT::i32);
  SDValue SetNE = DAG.getConstant(ISD::SETNE, DL, MVT::i32);

  SDVTList VTs = DAG.getVTList(Op.getValueType(), MVT::Glue);
  SDValue Ops[] = {CondV, Zero, SetNE, TrueV, FalseV};

  return DAG.getNode(MYRISCVXISD::SELECT_CC, DL, VTs, Ops);
}


MYRISCVXTargetLowering::MYRISCVXCC::MYRISCVXCC(
    CallingConv::ID CC, bool IsLP32_, CCState &Info,
    MYRISCVXCC::SpecialCallingConvType SpecialCallingConv_)
    : CCInfo(Info), CallConv(CC), IsLP32(IsLP32_) {
  // Pre-allocate reserved argument area.
  CCInfo.AllocateStack(reservedArgArea(), 1);
}


//===----------------------------------------------------------------------===//
// Call Calling Convention Implementation
//===----------------------------------------------------------------------===//

static const MCPhysReg O32IntRegs[] = {
  MYRISCVX::A0, MYRISCVX::A1, MYRISCVX::A2, MYRISCVX::A3,
  MYRISCVX::A4, MYRISCVX::A5, MYRISCVX::A6, MYRISCVX::A7
};


//@LowerCall {
/// LowerCall - functions arguments are copied from virtual regs to
/// (physical regs)/(stack frame), CALLSEQ_START and CALLSEQ_END are emitted.
SDValue
MYRISCVXTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                                  SmallVectorImpl<SDValue> &InVals) const {

  SelectionDAG &DAG                     = CLI.DAG;
  SDLoc DL                              = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals     = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins   = CLI.Ins;
  SDValue Chain                         = CLI.Chain;
  SDValue Callee                        = CLI.Callee;
  bool &IsTailCall                      = CLI.IsTailCall;
  CallingConv::ID CallConv              = CLI.CallConv;
  bool IsVarArg                         = CLI.IsVarArg;

  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetFrameLowering *TFL = MF.getSubtarget().getFrameLowering();
  MYRISCVXFunctionInfo *FuncInfo = MF.getInfo<MYRISCVXFunctionInfo>();
  bool IsPIC = isPositionIndependent();
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();

  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 ArgLocs, *DAG.getContext());
  CCInfo.AnalyzeCallOperands (Outs, CC_MYRISCVX);

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NextStackOffset = CCInfo.getNextStackOffset();

  //@TailCall 1 {
  // Check if it's really possible to do a tail call.
  if (IsTailCall)
    IsTailCall =
        isEligibleForTailCallOptimization(CCInfo, NextStackOffset,
                                          *MF.getInfo<MYRISCVXFunctionInfo>());

  if (!IsTailCall && CLI.CS && CLI.CS.isMustTailCall())
    report_fatal_error("failed to perform tail call elimination on a call "
                       "site marked musttail");

  if (IsTailCall)
    ++NumTailCalls;
  //@TailCall 1 }

  // Chain is the output chain of the last Load/Store or CopyToReg node.
  // ByValChain is the output chain of the last Memcpy node created for copying
  // byval arguments to the stack.
  unsigned StackAlignment = TFL->getStackAlignment();
  NextStackOffset = alignTo(NextStackOffset, StackAlignment);
  SDValue NextStackOffsetVal = DAG.getIntPtrConstant(NextStackOffset, DL, true);

  //@TailCall 2 {
  if (!IsTailCall)
    Chain = DAG.getCALLSEQ_START(Chain, NextStackOffset, 0, DL);
  //@TailCall 2 }

  SDValue StackPtr =
      DAG.getCopyFromReg(Chain, DL, MYRISCVX::SP,
                         getPointerTy(DAG.getDataLayout()));

  // With EABI is it possible to have 16 args on registers.
  std::deque< std::pair<unsigned, SDValue> > RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;

  CCInfo.rewindByValRegsInfo();

  //@1 {
  // Walk the register/memloc assignments, inserting copies/loads.
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    //@1 }
    SDValue Arg = OutVals[i];
    CCValAssign &VA = ArgLocs[i];
    MVT LocVT = VA.getLocVT();
    ISD::ArgFlagsTy Flags = Outs[i].Flags;

    //@ByVal Arg {
    if (Flags.isByVal()) {
      unsigned FirstByValReg, LastByValReg;
      unsigned ByValIdx = CCInfo.getInRegsParamsProcessed();

      LLVM_DEBUG(dbgs() << "ByValIdx = " << ByValIdx << " , ByValRegs.size() = " << CCInfo.getInRegsParamsCount() << '\n');
      CCInfo.getInRegsParamInfo(ByValIdx, FirstByValReg, LastByValReg);

      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValIdx < CCInfo.getInRegsParamsCount());
      assert(!IsTailCall &&
             "Do not tail-call optimize if there is a byval argument.");
      passByValArg(Chain, DL, RegsToPass, MemOpChains, StackPtr, MFI, DAG, Arg,
                   CCInfo, FirstByValReg, LastByValReg, Flags, Subtarget.isLittle(), VA);
      CCInfo.nextInRegsParam();
      continue;
    }
    //@ByVal Arg }

    // Promote the value if needed.
    switch (VA.getLocInfo()) {
      default: llvm_unreachable("Unknown loc info!");
      case CCValAssign::Full:
        break;
      case CCValAssign::SExt:
        Arg = DAG.getNode(ISD::SIGN_EXTEND, DL, LocVT, Arg);
        break;
      case CCValAssign::ZExt:
        Arg = DAG.getNode(ISD::ZERO_EXTEND, DL, LocVT, Arg);
        break;
      case CCValAssign::AExt:
        Arg = DAG.getNode(ISD::ANY_EXTEND, DL, LocVT, Arg);
        break;
    }

    // Arguments that can be passed on register must be kept at
    // RegsToPass vector
    if (VA.isRegLoc()) {
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
      continue;
    }

    // Register can't get to this point...
    assert(VA.isMemLoc());

    // emit ISD::STORE whichs stores the
    // parameter value to a stack Location
    MemOpChains.push_back(passArgOnStack(StackPtr, VA.getLocMemOffset(),
                                         Chain, Arg, DL, IsTailCall, DAG));
  }

  // Transform all store nodes into one single node because all store
  // nodes are independent of each other.
  if (!MemOpChains.empty())
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, MemOpChains);

  // If the callee is a GlobalAddress/ExternalSymbol node (quite common, every
  // direct call is) turn it into a TargetGlobalAddress/TargetExternalSymbol
  // node so that legalize doesn't hack it.
  bool IsPICCall = IsPIC; // true if calls are translated to
  // jalr $t9
  bool GlobalOrExternal = false, InternalLinkage = false;
  SDValue CalleeLo;
  EVT Ty = Callee.getValueType();

  if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
    if (IsPICCall) {
      const GlobalValue *Val = G->getGlobal();
      InternalLinkage = Val->hasInternalLinkage();

      if (InternalLinkage)
        Callee = getAddrLocal(G, Ty, DAG);
      else
        Callee = getAddrGlobal(G, Ty, DAG, MYRISCVXII::MO_GOT_CALL, Chain,
                               FuncInfo->callPtrInfo(Val));
    } else
      Callee = DAG.getTargetGlobalAddress(G->getGlobal(), DL,
                                          getPointerTy(DAG.getDataLayout()), 0,
                                          MYRISCVXII::MO_NO_FLAG);
    GlobalOrExternal = true;
  } else if (ExternalSymbolSDNode *S = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    const char *Sym = S->getSymbol();

    if (!IsPIC) // static
      Callee = DAG.getTargetExternalSymbol(Sym,
                                           getPointerTy(DAG.getDataLayout()),
                                           MYRISCVXII::MO_NO_FLAG);
    else // PIC
      Callee = getAddrGlobal(S, Ty, DAG, MYRISCVXII::MO_GOT_CALL, Chain,
                             FuncInfo->callPtrInfo(Sym));

    GlobalOrExternal = true;
  }

  SmallVector<SDValue, 8> Ops(1, Chain);
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);

  getOpndList(Ops, RegsToPass, IsPICCall, GlobalOrExternal, InternalLinkage,
              CLI, Callee, Chain);

  //@TailCall 3 {
  if (IsTailCall)
    return DAG.getNode(MYRISCVXISD::TailCall, DL, MVT::Other, Ops);
  //@TailCall 3 }

  Chain = DAG.getNode(MYRISCVXISD::CALL, DL, NodeTys, Ops);
  SDValue InFlag = Chain.getValue(1);

  // Create the CALLSEQ_END node.
  Chain = DAG.getCALLSEQ_END(Chain, NextStackOffsetVal,
                             DAG.getIntPtrConstant(0, DL, true), InFlag, DL);
  InFlag = Chain.getValue(1);

  // Handle result values, copying them out of physregs into vregs that we
  // return.
  return LowerCallResult(Chain, InFlag, CallConv, IsVarArg,
                         Ins, DL, DAG, InVals, CLI.Callee.getNode(), CLI.RetTy);

}


void MYRISCVXTargetLowering::HandleByVal(CCState *State, unsigned &Size,
                                         unsigned Align) const
{
  const TargetFrameLowering *TFL = Subtarget.getFrameLowering();

  assert(Size && "Byval argument's size shouldn't be 0.");

  Align = std::min(Align, TFL->getStackAlignment());

  unsigned FirstReg = 0;
  unsigned NumRegs = 0;

  if (State->getCallingConv() != CallingConv::Fast) {
    unsigned RegSizeInBytes = Subtarget.getGPRSizeInBytes();
    ArrayRef<MCPhysReg> IntArgRegs = ABI.GetByValArgRegs();
    // FIXME: The O32 case actually describes no shadow registers.
    const MCPhysReg *ShadowRegs = IntArgRegs.data();

    // We used to check the size as well but we can't do that anymore since
    // CCState::HandleByVal() rounds up the size after calling this function.
    assert(!(Align % RegSizeInBytes) &&
           "Byval argument's alignment should be a multiple of"
           "RegSizeInBytes.");

    FirstReg = State->getFirstUnallocated(IntArgRegs);

    // If Align > RegSizeInBytes, the first arg register must be even.
    // FIXME: This condition happens to do the right thing but it's not the
    //        right way to test it. We want to check that the stack frame offset
    //        of the register is aligned.
    if ((Align > RegSizeInBytes) && (FirstReg % 2)) {
      State->AllocateReg(IntArgRegs[FirstReg], ShadowRegs[FirstReg]);
      ++FirstReg;
    }

    // Mark the registers allocated.
    Size = alignTo(Size, RegSizeInBytes);
    LLVM_DEBUG(dbgs() << "HandbleByVal = " << IntArgRegs.size() << '\n');
    for (unsigned I = FirstReg; Size > 0 && (I < IntArgRegs.size());
         Size -= RegSizeInBytes, ++I, ++NumRegs) {
      LLVM_DEBUG(dbgs() << "Size = " << Size << ", I = " << I << ", NumRegs = " << NumRegs << '\n');
      State->AllocateReg(IntArgRegs[I], ShadowRegs[I]);
    }
  }

  State->addInRegsParamInfo(FirstReg, FirstReg + NumRegs);
}


/// LowerCallResult - Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
SDValue
MYRISCVXTargetLowering::LowerCallResult(SDValue Chain, SDValue InFlag,
                                        CallingConv::ID CallConv, bool IsVarArg,
                                        const SmallVectorImpl<ISD::InputArg> &Ins,
                                        const SDLoc &DL, SelectionDAG &DAG,
                                        SmallVectorImpl<SDValue> &InVals,
                                        const SDNode *CallNode,
                                        const Type *RetTy) const {
  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 RVLocs, *DAG.getContext());
  const ExternalSymbolSDNode *ES =
      dyn_cast_or_null<const ExternalSymbolSDNode>(CallNode);
  CCInfo.AnalyzeCallResult(Ins, RetCC_MYRISCVX);

  // Copy all of the result registers out of their specified physreg.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    SDValue Val = DAG.getCopyFromReg(Chain, DL, RVLocs[i].getLocReg(),
                                     RVLocs[i].getLocVT(), InFlag);
    Chain = Val.getValue(1);
    InFlag = Val.getValue(2);

    if (RVLocs[i].getValVT() != RVLocs[i].getLocVT())
      Val = DAG.getNode(ISD::BITCAST, DL, RVLocs[i].getValVT(), Val);

    InVals.push_back(Val);
  }

  return Chain;
}


SDValue
MYRISCVXTargetLowering::passArgOnStack(SDValue StackPtr, unsigned Offset,
                                       SDValue Chain, SDValue Arg, const SDLoc &DL,
                                       bool IsTailCall, SelectionDAG &DAG) const {
  if (!IsTailCall) {
    SDValue PtrOff =
        DAG.getNode(ISD::ADD, DL, getPointerTy(DAG.getDataLayout()), StackPtr,
                    DAG.getIntPtrConstant(Offset, DL));
    return DAG.getStore(Chain, DL, Arg, PtrOff, MachinePointerInfo());
  }

  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
  int FI = MFI.CreateFixedObject(Arg.getValueSizeInBits() / 8, Offset, false);
  SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
  return DAG.getStore(Chain, DL, Arg, FIN, MachinePointerInfo(),
                      /* Alignment = */ 0, MachineMemOperand::MOVolatile);
}


// Copy byVal arg to registers and stack.
void MYRISCVXTargetLowering::
passByValArg(SDValue Chain, const SDLoc &DL,
             std::deque< std::pair<unsigned, SDValue> > &RegsToPass,
             SmallVectorImpl<SDValue> &MemOpChains, SDValue StackPtr,
             MachineFrameInfo &MFI, SelectionDAG &DAG, SDValue Arg,
             const CCState &CC, unsigned FirstReg, unsigned LastReg,
             const ISD::ArgFlagsTy &Flags, bool isLittle,
             const CCValAssign &VA) const {
  unsigned ByValSizeInBytes = Flags.getByValSize();
  unsigned OffsetInBytes = 0; // From beginning of struct
  unsigned RegSizeInBytes = Subtarget.getGPRSizeInBytes();
  unsigned Alignment = std::min(Flags.getByValAlign(), RegSizeInBytes);
  EVT PtrTy = getPointerTy(DAG.getDataLayout()),
      RegTy = MVT::getIntegerVT(RegSizeInBytes * 8);
  unsigned NumRegs = LastReg - FirstReg;

  if (NumRegs) {
    const ArrayRef<MCPhysReg> ArgRegs = ABI.GetByValArgRegs();
    bool LeftoverBytes = (NumRegs * RegSizeInBytes > ByValSizeInBytes);
    unsigned I = 0;

    // Copy words to registers.
    for (; I < NumRegs - LeftoverBytes;
         ++I, OffsetInBytes += RegSizeInBytes) {
      SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                    DAG.getConstant(OffsetInBytes, DL, PtrTy));
      SDValue LoadVal = DAG.getLoad(RegTy, DL, Chain, LoadPtr,
                                    MachinePointerInfo());
      MemOpChains.push_back(LoadVal.getValue(1));
      unsigned ArgReg = ArgRegs[FirstReg + I];
      RegsToPass.push_back(std::make_pair(ArgReg, LoadVal));
    }

    // Return if the struct has been fully copied.
    if (ByValSizeInBytes == OffsetInBytes)
      return;

    // Copy the remainder of the byval argument with sub-word loads and shifts.
    if (LeftoverBytes) {
      assert((ByValSizeInBytes > OffsetInBytes) &&
             (ByValSizeInBytes < OffsetInBytes + RegSizeInBytes) &&
             "Size of the remainder should be smaller than RegSizeInBytes.");
      SDValue Val;

      for (unsigned LoadSizeInBytes = RegSizeInBytes / 2, TotalBytesLoaded = 0;
           OffsetInBytes < ByValSizeInBytes; LoadSizeInBytes /= 2) {
        unsigned RemainingSizeInBytes = ByValSizeInBytes - OffsetInBytes;

        if (RemainingSizeInBytes < LoadSizeInBytes)
          continue;

        // Load subword.
        SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                      DAG.getConstant(OffsetInBytes, DL, PtrTy));
        SDValue LoadVal = DAG.getExtLoad(
            ISD::ZEXTLOAD, DL, RegTy, Chain, LoadPtr, MachinePointerInfo(),
            MVT::getIntegerVT(LoadSizeInBytes * 8), Alignment);
        MemOpChains.push_back(LoadVal.getValue(1));

        // Shift the loaded value.
        unsigned Shamt;

        if (isLittle)
          Shamt = TotalBytesLoaded * 8;
        else
          Shamt = (RegSizeInBytes - (TotalBytesLoaded + LoadSizeInBytes)) * 8;

        SDValue Shift = DAG.getNode(ISD::SHL, DL, RegTy, LoadVal,
                                    DAG.getConstant(Shamt, DL, MVT::i32));

        if (Val.getNode())
          Val = DAG.getNode(ISD::OR, DL, RegTy, Val, Shift);
        else
          Val = Shift;

        OffsetInBytes += LoadSizeInBytes;
        TotalBytesLoaded += LoadSizeInBytes;
        Alignment = std::min(Alignment, LoadSizeInBytes);
      }

      unsigned ArgReg = ArgRegs[FirstReg + I];
      RegsToPass.push_back(std::make_pair(ArgReg, Val));
      return;
    }
  }

  // Copy remainder of byval arg to it with memcpy.
  unsigned MemCpySize = ByValSizeInBytes - OffsetInBytes;
  SDValue Src = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                            DAG.getConstant(OffsetInBytes, DL, PtrTy));
  SDValue Dst = DAG.getNode(ISD::ADD, DL, PtrTy, StackPtr,
                            DAG.getIntPtrConstant(VA.getLocMemOffset(), DL));
  Chain = DAG.getMemcpy(Chain, DL, Dst, Src,
                        DAG.getConstant(MemCpySize, DL, PtrTy),
                        Alignment, /*isVolatile=*/false, /*AlwaysInline=*/false,
                        /*isTailCall=*/false,
                        MachinePointerInfo(), MachinePointerInfo());
  MemOpChains.push_back(Chain);
}


void MYRISCVXTargetLowering::
getOpndList(SmallVectorImpl<SDValue> &Ops,
            std::deque< std::pair<unsigned, SDValue> > &RegsToPass,
            bool IsPICCall, bool GlobalOrExternal, bool InternalLinkage,
            CallLoweringInfo &CLI, SDValue Callee, SDValue Chain) const {
  // T9 should contain the address of the callee function if
  // -reloction-model=pic or it is an indirect call.
  if (IsPICCall || !GlobalOrExternal) {
    unsigned T0Reg = MYRISCVX::T0;
    RegsToPass.push_front(std::make_pair(T0Reg, Callee));
  } else
    Ops.push_back(Callee);

  // Insert node "GP copy globalreg" before call to function.
  //
  // R_MYRISCVX_CALL* operators (emitted when non-internal functions are called
  // in PIC mode) allow symbols to be resolved via lazy binding.
  // The lazy binding stub requires GP to point to the GOT.
  if (IsPICCall && !InternalLinkage) {
    unsigned GPReg = MYRISCVX::GP;
    EVT Ty = MVT::i32;
    RegsToPass.push_back(std::make_pair(GPReg, getGlobalReg(CLI.DAG, Ty)));
  }

  // Build a sequence of copy-to-reg nodes chained together with token
  // chain and flag operands which copy the outgoing args into registers.
  // The InFlag in necessary since all emitted instructions must be
  // stuck together.
  SDValue InFlag;

  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Chain = CLI.DAG.getCopyToReg(Chain, CLI.DL, RegsToPass[i].first,
                                 RegsToPass[i].second, InFlag);
    InFlag = Chain.getValue(1);
  }

  // Add argument registers to the end of the list so that they are
  // known live into the call.
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i)
    Ops.push_back(CLI.DAG.getRegister(RegsToPass[i].first,
                                      RegsToPass[i].second.getValueType()));

  // Add a register mask operand representing the call-preserved registers.
  const TargetRegisterInfo *TRI = Subtarget.getRegisterInfo();
  const uint32_t *Mask =
      TRI->getCallPreservedMask(CLI.DAG.getMachineFunction(), CLI.CallConv);
  assert(Mask && "Missing call preserved mask for calling convention");
  Ops.push_back(CLI.DAG.getRegisterMask(Mask));

  if (InFlag.getNode())
    Ops.push_back(InFlag);
}


//===----------------------------------------------------------------------===//
//@            Formal Arguments Calling Convention Implementation
//===----------------------------------------------------------------------===//

//@LowerFormalArguments {
/// LowerFormalArguments - transform physical registers into virtual registers
/// and generate load operations for arguments places on the stack.
SDValue
MYRISCVXTargetLowering::LowerFormalArguments (SDValue Chain,
                                              CallingConv::ID CallConv,
                                              bool IsVarArg,
                                              const SmallVectorImpl<ISD::InputArg> &Ins,
                                              const SDLoc &DL, SelectionDAG &DAG,
                                              SmallVectorImpl<SDValue> &InVals) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();

  MYRISCVXFI->setVarArgsFrameIndex(0);

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 ArgLocs, *DAG.getContext());

  Function::const_arg_iterator FuncArg =
      DAG.getMachineFunction().getFunction().arg_begin();
  // bool UseSoftFloat = Subtarget.abiUsesSoftFloat();
  CCInfo.AnalyzeFormalArguments (Ins, CC_MYRISCVX);

  // Used with vargs to acumulate store chains.
  std::vector<SDValue> OutChains;

  unsigned CurArgIdx = 0;
  CCInfo.rewindByValRegsInfo();

  //@2 {
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    //@2 }
    CCValAssign &VA = ArgLocs[i];
    if (Ins[i].isOrigArg()) {
      std::advance(FuncArg, Ins[i].getOrigArgIndex() - CurArgIdx);
      CurArgIdx = Ins[i].getOrigArgIndex();
    }
    EVT ValVT = VA.getValVT();
    ISD::ArgFlagsTy Flags = Ins[i].Flags;
    bool IsRegLoc = VA.isRegLoc();

    //@byval pass {
    if (Flags.isByVal()) {
      assert(Ins[i].isOrigArg() && "Byval arguments cannot be implicit");
      unsigned FirstByValReg, LastByValReg;
      unsigned ByValIdx = CCInfo.getInRegsParamsProcessed();
      CCInfo.getInRegsParamInfo(ByValIdx, FirstByValReg, LastByValReg);

      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValIdx < CCInfo.getInRegsParamsCount());

      LLVM_DEBUG(dbgs() << "LowerFormalArguments ByValIdx = " << ByValIdx <<
                 ", FirstByValRegs = " << FirstByValReg <<
                 ", LastByValRegs = " << LastByValReg << '\n');

      copyByValRegs(Chain, DL, OutChains, DAG, Flags, InVals, &*FuncArg,
                    FirstByValReg, LastByValReg, VA, CCInfo);
      CCInfo.nextInRegsParam();
      continue;
    }
    //@byval pass }

    // Arguments stored on registers
    if (IsRegLoc) {
      MVT RegVT = VA.getLocVT();
      unsigned ArgReg = VA.getLocReg();
      const TargetRegisterClass *RC = getRegClassFor(RegVT);

      // Transform the arguments stored on
      // physical registers into virtual ones
      unsigned Reg = addLiveIn(DAG.getMachineFunction(), ArgReg, RC);
      SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, RegVT);

      // If this is an 8 or 16-bit value, it has been passed promoted
      // to 32 bits.  Insert an assert[sz]ext to capture this, then
      // truncate to the right size.
      if (VA.getLocInfo() != CCValAssign::Full) {
        unsigned Opcode = 0;
        if (VA.getLocInfo() == CCValAssign::SExt)
          Opcode = ISD::AssertSext;
        else if (VA.getLocInfo() == CCValAssign::ZExt)
          Opcode = ISD::AssertZext;
        if (Opcode)
          ArgValue = DAG.getNode(Opcode, DL, RegVT, ArgValue,
                                 DAG.getValueType(ValVT));
        ArgValue = DAG.getNode(ISD::TRUNCATE, DL, ValVT, ArgValue);
      }

      // Handle floating point arguments passed in integer registers.
      if ((RegVT == MVT::i32 && ValVT == MVT::f32) ||
          (RegVT == MVT::i64 && ValVT == MVT::f64))
        ArgValue = DAG.getNode(ISD::BITCAST, DL, ValVT, ArgValue);
      InVals.push_back(ArgValue);
    } else { // VA.isRegLoc()
      MVT LocVT = VA.getLocVT();

      // sanity check
      assert(VA.isMemLoc());

      // The stack pointer offset is relative to the caller stack frame.
      int FI = MFI.CreateFixedObject(ValVT.getSizeInBits()/8,
                                     VA.getLocMemOffset(), true);

      // Create load nodes to retrieve arguments from the stack
      SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
      SDValue Load = DAG.getLoad(
          LocVT, DL, Chain, FIN,
          MachinePointerInfo::getFixedStack(DAG.getMachineFunction(), FI));
      InVals.push_back(Load);
      OutChains.push_back(Load.getValue(1));
    }
  }

  //@Ordinary struct type: 1 {
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    // The MYRISCVX ABIs for returning structs by value requires that we copy
    // the sret argument into $v0 for the return. Save the argument into
    // a virtual register so that we can access it from the return points.
    if (Ins[i].Flags.isSRet()) {
      unsigned Reg = MYRISCVXFI->getSRetReturnReg();
      if (!Reg) {
        Reg = MF.getRegInfo().createVirtualRegister(
            getRegClassFor(MVT::i32));
        MYRISCVXFI->setSRetReturnReg(Reg);
      }
      SDValue Copy = DAG.getCopyToReg(DAG.getEntryNode(), DL, Reg, InVals[i]);
      Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, Copy, Chain);
      break;
    }
  }
  //@Ordinary struct type: 1 }

  if (IsVarArg)
    writeVarArgRegs(OutChains, Chain, DL, DAG, CCInfo);

  // All stores are grouped in one node to allow the matching between
  // the size of Ins and InVals. This only happens when on varg functions
  if (!OutChains.empty()) {
    OutChains.push_back(Chain);
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, OutChains);
  }

  return Chain;
}
// @LowerFormalArguments }


void MYRISCVXTargetLowering::MYRISCVXCC::
analyzeFormalArguments(const SmallVectorImpl<ISD::InputArg> &Args,
                       bool IsSoftFloat, Function::const_arg_iterator FuncArg) {
  unsigned NumArgs = Args.size();
  llvm::CCAssignFn *FixedFn = fixedArgFn();
  unsigned CurArgIdx = 0;
  for (unsigned I = 0; I != NumArgs; ++I) {
    MVT ArgVT = Args[I].VT;
    ISD::ArgFlagsTy ArgFlags = Args[I].Flags;
    if (Args[I].isOrigArg()) {
      std::advance(FuncArg, Args[I].getOrigArgIndex() - CurArgIdx);
      CurArgIdx = Args[I].getOrigArgIndex();
    } else {
      CurArgIdx = Args[I].getOrigArgIndex();
    }
    if (ArgFlags.isByVal()) {
      handleByValArg(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags);
      continue;
    }
    MVT RegVT = getRegVT(ArgVT, FuncArg->getType(), nullptr, IsSoftFloat);
    if (!FixedFn(I, ArgVT, RegVT, CCValAssign::Full, ArgFlags, CCInfo))
      continue;
    llvm_unreachable(nullptr);
  }
}


void MYRISCVXTargetLowering::copyByValRegs(
    SDValue Chain, const SDLoc &DL, std::vector<SDValue> &OutChains,
    SelectionDAG &DAG, const ISD::ArgFlagsTy &Flags,
    SmallVectorImpl<SDValue> &InVals, const Argument *FuncArg,
    unsigned FirstReg, unsigned LastReg, const CCValAssign &VA,
    CCState &State) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned GPRSizeInBytes = Subtarget.getGPRSizeInBytes();
  unsigned NumRegs = LastReg - FirstReg;
  unsigned RegAreaSize = NumRegs * GPRSizeInBytes;
  unsigned FrameObjSize = std::max(Flags.getByValSize(), RegAreaSize);
  int FrameObjOffset;
  ArrayRef<MCPhysReg> ByValArgRegs = ABI.GetByValArgRegs();

  if (RegAreaSize)
    FrameObjOffset =
        (int)ABI.GetCalleeAllocdArgSizeInBytes(State.getCallingConv()) -
        (int)((ByValArgRegs.size() - FirstReg) * GPRSizeInBytes);
  else
    FrameObjOffset = VA.getLocMemOffset();

  LLVM_DEBUG(dbgs() << "copyByValRegs : RegAreaSize = " << RegAreaSize << '\n');
  LLVM_DEBUG(dbgs() << "copyByValRegs : FrameObjOffset = " << FrameObjOffset << '\n');

  // Create frame object.
  EVT PtrTy = getPointerTy(DAG.getDataLayout());
  // Make the fixed object stored to mutable so that the load instructions
  // referencing it have their memory dependencies added.
  // Set the frame object as isAliased which clears the underlying objects
  // vector in ScheduleDAGInstrs::buildSchedGraph() resulting in addition of all
  // stores as dependencies for loads referencing this fixed object.
  int FI = MFI.CreateFixedObject(FrameObjSize, FrameObjOffset, false, true);
  SDValue FIN = DAG.getFrameIndex(FI, PtrTy);
  InVals.push_back(FIN);

  LLVM_DEBUG(dbgs() << "copyByValRegs : NumRegs = " << NumRegs << '\n');

  if (!NumRegs)
    return;

  // Copy arg registers.
  MVT RegTy = MVT::getIntegerVT(GPRSizeInBytes * 8);
  const TargetRegisterClass *RC = getRegClassFor(RegTy);

  for (unsigned I = 0; I < NumRegs; ++I) {
    unsigned ArgReg = ByValArgRegs[FirstReg + I];
    unsigned VReg = addLiveIn(MF, ArgReg, RC);
    unsigned Offset = I * GPRSizeInBytes;
    SDValue StorePtr = DAG.getNode(ISD::ADD, DL, PtrTy, FIN,
                                   DAG.getConstant(Offset, DL, PtrTy));
    SDValue Store = DAG.getStore(Chain, DL, DAG.getRegister(VReg, RegTy),
                                 StorePtr, MachinePointerInfo(FuncArg, Offset));
    OutChains.push_back(Store);

    LLVM_DEBUG(dbgs() << "copyByValRegs : Store(" << I << ") = " << "VReg = " << VReg << ", Offset = " << Offset << '\n');

  }
}


void MYRISCVXTargetLowering::MYRISCVXCC::handleByValArg(unsigned ValNo, MVT ValVT,
                                                        MVT LocVT,
                                                        CCValAssign::LocInfo LocInfo,
                                                        ISD::ArgFlagsTy ArgFlags) {
  assert(ArgFlags.getByValSize() && "Byval argument's size shouldn't be 0.");
  struct ByValArgInfo ByVal;
  unsigned RegSize = regSize();
  unsigned ByValSize = alignTo(ArgFlags.getByValSize(), RegSize);
  unsigned Align = std::min(std::max(ArgFlags.getByValAlign(), RegSize),
                            RegSize * 2);
  if (useRegsForByval())
    allocateRegs(ByVal, ByValSize, Align);
  // Allocate space on caller's stack.
  ByVal.Address = CCInfo.AllocateStack(ByValSize - RegSize * ByVal.NumRegs,
                                       Align);
  CCInfo.addLoc(CCValAssign::getMem(ValNo, ValVT, ByVal.Address, LocVT,
                                    LocInfo));
  ByValArgs.push_back(ByVal);
}


unsigned MYRISCVXTargetLowering::MYRISCVXCC::numIntArgRegs() const {
  return IsLP32 ? array_lengthof(O32IntRegs) : 0;
}


const ArrayRef<MCPhysReg> MYRISCVXTargetLowering::MYRISCVXCC::intArgRegs() const {
  return makeArrayRef(O32IntRegs);
}


void MYRISCVXTargetLowering::MYRISCVXCC::allocateRegs(ByValArgInfo &ByVal,
                                                      unsigned ByValSize,
                                                      unsigned Align) {
  unsigned RegSize = regSize(), NumIntArgRegs = numIntArgRegs();
  const ArrayRef<MCPhysReg> IntArgRegs = intArgRegs();
  assert(!(ByValSize % RegSize) && !(Align % RegSize) &&
         "Byval argument's size and alignment should be a multiple of"
         "RegSize.");
  ByVal.FirstIdx = CCInfo.getFirstUnallocated(IntArgRegs);
  // If Align > RegSize, the first arg register must be even.
  if ((Align > RegSize) && (ByVal.FirstIdx % 2)) {
    CCInfo.AllocateReg(IntArgRegs[ByVal.FirstIdx]);
    ++ByVal.FirstIdx;
  }
  // Mark the registers allocated.
  for (unsigned I = ByVal.FirstIdx; ByValSize && (I < NumIntArgRegs);
       ByValSize -= RegSize, ++I, ++ByVal.NumRegs)
    CCInfo.AllocateReg(IntArgRegs[I]);
}


template<typename Ty>
void MYRISCVXTargetLowering::MYRISCVXCC::
analyzeReturn(const SmallVectorImpl<Ty> &RetVals, bool IsSoftFloat,
              const SDNode *CallNode, const Type *RetTy) const {
  CCAssignFn *Fn;

  Fn = RetCC_MYRISCVX;

  for (unsigned I = 0, E = RetVals.size(); I < E; ++I) {
    MVT VT = RetVals[I].VT;
    ISD::ArgFlagsTy Flags = RetVals[I].Flags;
    MVT RegVT = this->getRegVT(VT, RetTy, CallNode, IsSoftFloat);

    if (Fn(I, VT, RegVT, CCValAssign::Full, Flags, this->CCInfo)) {
      LLVM_DEBUG(dbgs() << "Call result #" << I << " has unhandled type "
                 << EVT(VT).getEVTString() << '\n');
      llvm_unreachable(nullptr);
    }
  }
}


void MYRISCVXTargetLowering::MYRISCVXCC::
analyzeCallResult(const SmallVectorImpl<ISD::InputArg> &Ins, bool IsSoftFloat,
                  const SDNode *CallNode, const Type *RetTy) const {
  analyzeReturn(Ins, IsSoftFloat, CallNode, RetTy);
}

void MYRISCVXTargetLowering::MYRISCVXCC::
analyzeReturn(const SmallVectorImpl<ISD::OutputArg> &Outs, bool IsSoftFloat,
              const Type *RetTy) const {
  analyzeReturn(Outs, IsSoftFloat, nullptr, RetTy);
}

unsigned MYRISCVXTargetLowering::MYRISCVXCC::reservedArgArea() const {
  return (IsLP32 && (CallConv != CallingConv::Fast)) ? 8 : 0;
}

MVT MYRISCVXTargetLowering::MYRISCVXCC::getRegVT(MVT VT, const Type *OrigTy,
                                                 const SDNode *CallNode,
                                                 bool IsSoftFloat) const {
  if (IsSoftFloat || IsLP32)
    return VT;

  return VT;
}


SDValue MYRISCVXTargetLowering::getGlobalReg(SelectionDAG &DAG, EVT Ty) const {
  MYRISCVXFunctionInfo *FI = DAG.getMachineFunction().getInfo<MYRISCVXFunctionInfo>();
  return DAG.getRegister(FI->getGlobalBaseReg(), Ty);
}

//@getTargetNode(GlobalAddressSDNode
SDValue MYRISCVXTargetLowering::getTargetNode(GlobalAddressSDNode *N, EVT Ty,
                                              SelectionDAG &DAG,
                                              unsigned Flag) const {
  return DAG.getTargetGlobalAddress(N->getGlobal(), SDLoc(N), Ty, 0, Flag);
}

//@getTargetNode(ExternalSymbolSDNode
SDValue MYRISCVXTargetLowering::getTargetNode(ExternalSymbolSDNode *N, EVT Ty,
                                              SelectionDAG &DAG,
                                              unsigned Flag) const {
  return DAG.getTargetExternalSymbol(N->getSymbol(), Ty, Flag);
}


// Return the RISC-V branch opcode that matches the given DAG integer
// condition code. The CondCode must be one of those supported by the RISC-V
// ISA (see normaliseSetCC).
unsigned MYRISCVXTargetLowering::getBranchOpcodeForIntCondCode (ISD::CondCode CC) {
  switch (CC) {
  default:
    llvm_unreachable("Unsupported CondCode");
  case ISD::SETEQ:
    return MYRISCVX::BEQ;
  case ISD::SETNE:
    return MYRISCVX::BNE;
  case ISD::SETLT:
    return MYRISCVX::BLT;
  case ISD::SETGE:
    return MYRISCVX::BGE;
  case ISD::SETULT:
    return MYRISCVX::BLTU;
  case ISD::SETUGE:
    return MYRISCVX::BGEU;
  }
}


MachineBasicBlock *
MYRISCVXTargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI,
                                                    MachineBasicBlock *BB) const {

  // To "insert" a SELECT instruction, we actually have to insert the triangle
  // control-flow pattern.  The incoming instruction knows the destination vreg
  // to set, the condition code register to branch on, the true/false values to
  // select between, and the condcode to use to select the appropriate branch.
  //
  // We produce the following control flow:
  //     HeadMBB
  //     |  \
  //     |  IfFalseMBB
  //     | /
  //    TailMBB
  const TargetInstrInfo &TII = *BB->getParent()->getSubtarget().getInstrInfo();
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  DebugLoc DL = MI.getDebugLoc();
  MachineFunction::iterator I = ++BB->getIterator();

  MachineBasicBlock *HeadMBB = BB;
  MachineFunction *F = BB->getParent();
  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *IfFalseMBB = F->CreateMachineBasicBlock(LLVM_BB);

  F->insert(I, IfFalseMBB);
  F->insert(I, TailMBB);
  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());
  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);
  // Set the successors for HeadMBB.
  HeadMBB->addSuccessor(IfFalseMBB);
  HeadMBB->addSuccessor(TailMBB);

  // Insert appropriate branch.
  unsigned LHS = MI.getOperand(1).getReg();
  unsigned RHS = MI.getOperand(2).getReg();
  auto CC = static_cast<ISD::CondCode>(MI.getOperand(3).getImm());
  unsigned Opcode = getBranchOpcodeForIntCondCode(CC);

  BuildMI(HeadMBB, DL, TII.get(Opcode))
    .addReg(LHS)
    .addReg(RHS)
    .addMBB(TailMBB);

  // IfFalseMBB just falls through to TailMBB.
  IfFalseMBB->addSuccessor(TailMBB);

  // %Result = phi [ %TrueValue, HeadMBB ], [ %FalseValue, IfFalseMBB ]
  BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(MYRISCVX::PHI),
          MI.getOperand(0).getReg())
      .addReg(MI.getOperand(4).getReg())
      .addMBB(HeadMBB)
      .addReg(MI.getOperand(5).getReg())
      .addMBB(IfFalseMBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return TailMBB;
}


//===----------------------------------------------------------------------===//
// TODO: Implement a generic logic using tblgen that can support this.
// MYRISCVX 32 ABI rules:
// ---
//===----------------------------------------------------------------------===//
// Passed in stack only.
static bool CC_MYRISCVX_STACK32(unsigned ValNo, MVT ValVT, MVT LocVT,
                                CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                                CCState &State) {
  // Do not process byval args here.
  if (ArgFlags.isByVal())
    return true;
  // Promote i8 and i16
  if (LocVT == MVT::i8 || LocVT == MVT::i16) {
    LocVT = MVT::i32;
    if (ArgFlags.isSExt())
      LocInfo = CCValAssign::SExt;
    else if (ArgFlags.isZExt())
      LocInfo = CCValAssign::ZExt;
    else
      LocInfo = CCValAssign::AExt;
  }
  unsigned OrigAlign = ArgFlags.getOrigAlign();
  unsigned Offset = State.AllocateStack(ValVT.getSizeInBits() >> 3,
                                        OrigAlign);
  State.addLoc(CCValAssign::getMem(ValNo, ValVT, Offset, LocVT, LocInfo));
  return false;
}


// Passed first eight i32 arguments in registers and others in stack.
static bool CC_MYRISCVX_LP32(unsigned ValNo, MVT ValVT, MVT LocVT,
                             CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                             CCState &State) {
  static const MCPhysReg IntRegs[] = { MYRISCVX::A0, MYRISCVX::A1,
                                       MYRISCVX::A2, MYRISCVX::A3,
                                       MYRISCVX::A4, MYRISCVX::A5,
                                       MYRISCVX::A6, MYRISCVX::A7 };
  // Do not process byval args here.
  if (ArgFlags.isByVal())
    return true;
  // Promote i8 and i16
  if (LocVT == MVT::i8 || LocVT == MVT::i16) {
    LocVT = MVT::i32;
    if (ArgFlags.isSExt())
      LocInfo = CCValAssign::SExt;
    else if (ArgFlags.isZExt())
      LocInfo = CCValAssign::ZExt;
    else
      LocInfo = CCValAssign::AExt;
  }

  unsigned Reg;
  if (ValVT == MVT::i32) {
    Reg = State.AllocateReg(IntRegs);
    LocVT = MVT::i32;
  } else {
    llvm_unreachable("Cannot handle this ValVT.");
  }

  if (!Reg) {
    unsigned OrigAlign = ArgFlags.getOrigAlign();
    unsigned Offset    = State.AllocateStack(ValVT.getSizeInBits() >> 3,
                                             OrigAlign);
    State.addLoc(CCValAssign::getMem(ValNo, ValVT, Offset, LocVT, LocInfo));
  } else {
    State.addLoc(CCValAssign::getReg(ValNo, ValVT, Reg, LocVT, LocInfo));
  }

  return false;
}


llvm::CCAssignFn *MYRISCVXTargetLowering::MYRISCVXCC::fixedArgFn() const {
  if (IsLP32)
    return CC_MYRISCVX_LP32;
  else // otherwise
    return CC_MYRISCVX_STACK32;
}


SDValue MYRISCVXTargetLowering::lowerVASTART(SDValue Op, SelectionDAG &DAG) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MYRISCVXFunctionInfo *FuncInfo = MF.getInfo<MYRISCVXFunctionInfo>();

  SDLoc DL = SDLoc(Op);
  SDValue FI = DAG.getFrameIndex(FuncInfo->getVarArgsFrameIndex(),
                                 getPointerTy(MF.getDataLayout()));

  // vastart just stores the address of the VarArgsFrameIndex slot into the
  // memory location argument.
  const Value *SV = cast<SrcValueSDNode>(Op.getOperand(2))->getValue();
  return DAG.getStore(Op.getOperand(0), DL, FI, Op.getOperand(1),
                      MachinePointerInfo(SV));
}


void MYRISCVXTargetLowering::writeVarArgRegs(std::vector<SDValue> &OutChains,
                                             SDValue Chain, const SDLoc &DL,
                                             SelectionDAG &DAG,
                                             CCState &State) const {
  ArrayRef<MCPhysReg> ArgRegs = ABI.GetVarArgRegs();
  unsigned Idx = State.getFirstUnallocated(ArgRegs);
  unsigned RegSizeInBytes = Subtarget.getGPRSizeInBytes();
  MVT RegTy = MVT::getIntegerVT(RegSizeInBytes * 8);
  const TargetRegisterClass *RC = getRegClassFor(RegTy);
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();

  // Offset of the first variable argument from stack pointer.
  int VaArgOffset;

  if (ArgRegs.size() == Idx)
    VaArgOffset = alignTo(State.getNextStackOffset(), RegSizeInBytes);
  else {
    VaArgOffset =
        (int)ABI.GetCalleeAllocdArgSizeInBytes(State.getCallingConv()) -
        (int)(RegSizeInBytes * (ArgRegs.size() - Idx));
  }

  // Record the frame index of the first variable argument
  // which is a value necessary to VASTART.
  int FI = MFI.CreateFixedObject(RegSizeInBytes, VaArgOffset, true);
  MYRISCVXFI->setVarArgsFrameIndex(FI);

  // Copy the integer registers that have not been used for argument passing
  // to the argument register save area. For O32, the save area is allocated
  // in the caller's stack frame, while for N32/64, it is allocated in the
  // callee's stack frame.
  for (unsigned I = Idx; I < ArgRegs.size();
       ++I, VaArgOffset += RegSizeInBytes) {

    LLVM_DEBUG(dbgs() << "writeVarArgRegs I = " << I << '\n');

    unsigned Reg = addLiveIn(MF, ArgRegs[I], RC);
    SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, RegTy);
    FI = MFI.CreateFixedObject(RegSizeInBytes, VaArgOffset, true);
    SDValue PtrOff = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
    SDValue Store =
        DAG.getStore(Chain, DL, ArgValue, PtrOff, MachinePointerInfo());
    cast<StoreSDNode>(Store.getNode())->getMemOperand()->setValue(
        (Value *)nullptr);
    OutChains.push_back(Store);
  }
}
