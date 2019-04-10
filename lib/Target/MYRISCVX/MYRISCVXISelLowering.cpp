//===-- MYRISCVXISelLowering.cpp - MYRISCVX DAG Lowering Implementation -----------===//
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
#define DEBUG_TYPE "myriscvx-lower"

STATISTIC(NumTailCalls, "Number of tail calls");

//@3_1 1 {
const char *MYRISCVXTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
    case MYRISCVXISD::JmpLink:    return "MYRISCVXISD::JmpLink";
    case MYRISCVXISD::TailCall:   return "MYRISCVXISD::TailCall";
    case MYRISCVXISD::Hi:         return "MYRISCVXISD::Hi";
    case MYRISCVXISD::Lo:         return "MYRISCVXISD::Lo";
    case MYRISCVXISD::GPRel:      return "MYRISCVXISD::GPRel";
    case MYRISCVXISD::Ret:        return "MYRISCVXISD::Ret";
    case MYRISCVXISD::EH_RETURN:  return "MYRISCVXISD::EH_RETURN";
    case MYRISCVXISD::DivRem:     return "MYRISCVXISD::DivRem";
    case MYRISCVXISD::DivRemU:    return "MYRISCVXISD::DivRemU";
    case MYRISCVXISD::Wrapper:    return "MYRISCVXISD::Wrapper";
    case MYRISCVXISD::SELECT_CC:  return "MYRISCVXISD::SELECT_CC";
    default:                      return NULL;
  }
}


//@3_1 1 }
//@MYRISCVXTargetLowering {
MYRISCVXTargetLowering::MYRISCVXTargetLowering(const MYRISCVXTargetMachine &TM,
                                               const MYRISCVXSubtarget &STI)
    : TargetLowering(TM), Subtarget(STI), ABI(TM.getABI()) {

  //@MYRISCVXSETargetLowering body {
  // Set up the register classes
  addRegisterClass(MVT::i32, &MYRISCVX::GPRRegClass);

  // MYRISCVX Custom Operations
  setOperationAction(ISD::ROTL, MVT::i32, Expand);
  setOperationAction(ISD::ROTR, MVT::i32, Expand);
  setOperationAction(ISD::GlobalAddress, MVT::i32, Custom);
  setOperationAction(ISD::BlockAddress,  MVT::i32, Custom);

  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i32, Expand);

  // MYRISCVX does not have i1 type, so use i32 for
  // setcc operations results (slt, sgt, ...).
  setBooleanContents(ZeroOrOneBooleanContent);
  setBooleanVectorContents(ZeroOrNegativeOneBooleanContent);
  // Load extented operations for i1 types must be promoted
  for (MVT VT : MVT::integer_valuetypes()) {
    setLoadExtAction(ISD::EXTLOAD,  VT, MVT::i1, Promote);
    setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1, Promote);
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1, Promote);
  }

  // MYRISCVX doesn't have sext_inreg, replace them with shl/sra.
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1 , Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8 , Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i16 , Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i32 , Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::Other , Expand);

  setOperationAction(ISD::SMUL_LOHI, MVT::i32, Expand);
  setOperationAction(ISD::UMUL_LOHI, MVT::i32, Expand);

  // Handle i64 shl
  setOperationAction(ISD::SHL_PARTS, MVT::i32, Expand);
  setOperationAction(ISD::SRA_PARTS, MVT::i32, Expand);
  setOperationAction(ISD::SRL_PARTS, MVT::i32, Expand);

  // Branch Instructions
  setOperationAction(ISD::BR_CC, MVT::i32, Expand);
  setOperationAction(ISD::BR_CC, MVT::f32, Expand);
  setOperationAction(ISD::BR_CC, MVT::f64, Expand);

  setOperationAction(ISD::SELECT,    MVT::i32, Custom);
  setOperationAction(ISD::SELECT_CC, MVT::i32, Expand);

  setOperationAction(ISD::VASTART, MVT::Other, Custom);

  // Support va_arg(): variable numbers (not fixed numbers) of arguments
  // (parameters) for function all
  setOperationAction(ISD::VAARG, MVT::Other, Expand);
  setOperationAction(ISD::VACOPY, MVT::Other, Expand);
  setOperationAction(ISD::VAEND, MVT::Other, Expand);

  //@llvm.stacksave
  // Use the default for now
  setOperationAction(ISD::STACKSAVE, MVT::Other, Expand);
  setOperationAction(ISD::STACKRESTORE, MVT::Other, Expand);

  setOperationAction(ISD::EH_RETURN,     MVT::Other, Custom);

  setOperationAction(ISD::ADD,           MVT::i32, Custom);

  setOperationAction(ISD::BSWAP, MVT::i32, Expand);
  setOperationAction(ISD::BSWAP, MVT::i64, Expand);

  //- Set .align 2
  // It will emit .align 2 later
  setMinFunctionAlignment(2);
  // must, computeRegisterProperties - Once all of the register classes are
  // added, this allows us to compute derived properties we expose.
  computeRegisterProperties(STI.getRegisterInfo());

  //- Set .align 2
  // It will emit .align 2 later
  setMinFunctionAlignment(2);

  setStackPointerRegisterToSaveRestore(MYRISCVX::SP);
}

const MYRISCVXTargetLowering *MYRISCVXTargetLowering::create(const MYRISCVXTargetMachine &TM,
                                                             const MYRISCVXSubtarget &STI) {
  return llvm::createMYRISCVXSETargetLowering(TM, STI);
}


// addLiveIn - This helper function adds the specified physical register to the
// MachineFunction as a live in value. It also creates a corresponding
// virtual register for it.
static unsigned
addLiveIn(MachineFunction &MF, unsigned PReg, const TargetRegisterClass *RC)
{
  unsigned VReg = MF.getRegInfo().createVirtualRegister(RC);
  MF.getRegInfo().addLiveIn(PReg, VReg);
  return VReg;
}


//===----------------------------------------------------------------------===//
// Lower helper functions
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// Misc Lower Operation implementation
//===----------------------------------------------------------------------===//
#include "MYRISCVXGenCallingConv.inc"
//===----------------------------------------------------------------------===//
//@ Formal Arguments Calling Convention Implementation
//===----------------------------------------------------------------------===//


//@LowerFormalArguments {
/// LowerFormalArguments - transform physical registers into virtual registers
/// and generate load operations for arguments places on the stack.
SDValue
MYRISCVXTargetLowering::LowerFormalArguments(SDValue Chain,
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
  MYRISCVXCC MYRISCVXCCInfo(CallConv, ABI.IsO32(),
                            CCInfo);

  Function::const_arg_iterator FuncArg =
      DAG.getMachineFunction().getFunction().arg_begin();
  bool UseSoftFloat = Subtarget.abiUsesSoftFloat();

  MYRISCVXCCInfo.analyzeFormalArguments(Ins, UseSoftFloat, FuncArg);
  MYRISCVXFI->setFormalArgInfo(CCInfo.getNextStackOffset(),
                               MYRISCVXCCInfo.hasByValArg());

  // Used with vargs to acumulate store chains.
  std::vector<SDValue> OutChains;

  unsigned CurArgIdx = 0;
  MYRISCVXCC::byval_iterator ByValArg = MYRISCVXCCInfo.byval_begin();

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
      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValArg != MYRISCVXCCInfo.byval_end());
      copyByValRegs(Chain, DL, OutChains, DAG, Flags, InVals, &*FuncArg,
                    MYRISCVXCCInfo, *ByValArg);
      ++ByValArg;
      continue;
    }
    //@byval pass }
    // Arguments stored on registers
    if (ABI.IsO32() && IsRegLoc) {
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
    writeVarArgRegs(OutChains, MYRISCVXCCInfo, Chain, DL, DAG);

  // All stores are grouped in one node to allow the matching between
  // the size of Ins and InVals. This only happens when on varg functions
  if (!OutChains.empty()) {
    OutChains.push_back(Chain);
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, OutChains);
  }

  return Chain;
}
// @LowerFormalArguments }

//===----------------------------------------------------------------------===//
// Call Calling Convention Implementation
//===----------------------------------------------------------------------===//

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
    }
    CurArgIdx = Args[I].OrigArgIndex;
    if (ArgFlags.isByVal()) {
      handleByValArg(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags);
      continue;
    }
    MVT RegVT = getRegVT(ArgVT, FuncArg->getType(), nullptr, IsSoftFloat);
    if (!FixedFn(I, ArgVT, RegVT, CCValAssign::Full, ArgFlags, CCInfo))
      continue;
#ifndef NDEBUG
    dbgs() << "Formal Arg #" << I << " has unhandled type "
           << EVT(ArgVT).getEVTString();
#endif
    llvm_unreachable(nullptr);
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


//===----------------------------------------------------------------------===//
//@ Return Value Calling Convention Implementation
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
  MYRISCVXCC MYRISCVXCCInfo (CallConv, ABI.IsO32(), CCInfo);

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
  // the sret argument into $v0 for the return. We saved the argument into
  // a virtual register in the entry block, so now we copy the value out
  // and into $v0.
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
  RetOps[0] = Chain; // Update chain.
  // Add the flag if we have it.
  if (Flag.getNode())
    RetOps.push_back(Flag);
  // Return on MYRISCVX is always a "ret $lr"
  return DAG.getNode(MYRISCVXISD::Ret, DL, MVT::Other, RetOps);

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


SDValue MYRISCVXTargetLowering::
LowerOperation(SDValue Op, SelectionDAG &DAG) const
{
  switch (Op.getOpcode())
  {
    case ISD::GlobalAddress : return lowerGlobalAddress(Op, DAG);
    case ISD::BlockAddress  : return lowerBlockAddress(Op, DAG);
    case ISD::SELECT        : return lowerSELECT(Op, DAG);
    case ISD::VASTART       : return lowerVASTART(Op, DAG);
    case ISD::FRAMEADDR     : return lowerFRAMEADDR(Op, DAG);
    case ISD::RETURNADDR    : return lowerRETURNADDR(Op, DAG);
    case ISD::EH_RETURN     : return lowerEH_RETURN(Op, DAG);
    case ISD::ADD           : return lowerADD(Op, DAG);
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
    //@ %gp_rel relocation
    const GlobalObject *GO = GV->getBaseObject();
    if (TLOF->IsGlobalInSmallSection(GO, getTargetMachine())) {
      SDValue GA = DAG.getTargetGlobalAddress(GV, DL, MVT::i32, 0,
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
        N, Ty, DAG, MYRISCVXII::MO_GOT_HI16, MYRISCVXII::MO_GOT_LO16,
        DAG.getEntryNode(),
        MachinePointerInfo::getGOT(DAG.getMachineFunction()));
  return getAddrGlobal(
      N, Ty, DAG, MYRISCVXII::MO_GOT, DAG.getEntryNode(),
      MachinePointerInfo::getGOT(DAG.getMachineFunction()));
}


SDValue MYRISCVXTargetLowering::lowerBlockAddress(SDValue Op,
                                                  SelectionDAG &DAG) const {
  BlockAddressSDNode *N = cast<BlockAddressSDNode>(Op);
  EVT Ty = Op.getValueType();

  if (!isPositionIndependent())
    return getAddrNonPIC(N, Ty, DAG);

  return getAddrLocal(N, Ty, DAG);
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


MYRISCVXTargetLowering::MYRISCVXCC::MYRISCVXCC(CallingConv::ID CC, bool IsO32_, CCState &Info,
                                               MYRISCVXCC::SpecialCallingConvType SpecialCallingConv_)
    : CCInfo(Info), CallConv(CC), IsO32(IsO32_) {
  // Pre-allocate reserved argument area.
  CCInfo.AllocateStack(reservedArgArea(), 1);
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
#ifndef NDEBUG
      dbgs() << "Call result #" << I << " has unhandled type "
             << EVT(VT).getEVTString() << '\n';
#endif
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
  return (IsO32 && (CallConv != CallingConv::Fast)) ? 8 : 0;
}


MVT MYRISCVXTargetLowering::MYRISCVXCC::getRegVT(MVT VT, const Type *OrigTy,
                                                 const SDNode *CallNode,
                                                 bool IsSoftFloat) const {
  if (IsSoftFloat || IsO32)
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


SDValue MYRISCVXTargetLowering::getTargetNode(BlockAddressSDNode *N, EVT Ty,
                                              SelectionDAG &DAG,
                                              unsigned Flag) const {
  return DAG.getTargetBlockAddress(N->getBlockAddress(), Ty, 0, Flag);
}

SDValue MYRISCVXTargetLowering::getTargetNode(JumpTableSDNode *N, EVT Ty,
                                              SelectionDAG &DAG,
                                              unsigned Flag) const {
  return DAG.getTargetJumpTable(N->getIndex(), Ty, Flag);
}


//@getTargetNode(ExternalSymbolSDNode
SDValue MYRISCVXTargetLowering::getTargetNode(ExternalSymbolSDNode *N, EVT Ty,
                                              SelectionDAG &DAG,
                                              unsigned Flag) const {
  return DAG.getTargetExternalSymbol(N->getSymbol(), Ty, Flag);
}

bool
MYRISCVXTargetLowering::isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const {
  // The MYRISCVX target isn't yet aware of offsets.
  return false;
}

EVT MYRISCVXTargetLowering::getSetCCResultType(const DataLayout &, LLVMContext &,
                                               EVT VT) const {
  if (!VT.isVector())
    return MVT::i32;
  return VT.changeVectorElementTypeToInteger();
}


// Return the RISC-V branch opcode that matches the given DAG integer
// condition code. The CondCode must be one of those supported by the RISC-V
// ISA (see normaliseSetCC).
static unsigned getBranchOpcodeForIntCondCode(ISD::CondCode CC) {
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
  dbgs() << "MYRISCVXTargetLowering::EmitInstrWithCustomInserter\n";

  switch (MI.getOpcode()) {
    default:
      llvm_unreachable("Unexpected instr type to insert");
    case MYRISCVX::Select_GPR_Using_CC_GPR:
      break;
  }

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
static bool CC_MYRISCVXS32(unsigned ValNo, MVT ValVT, MVT LocVT,
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


// Passed first two i32 arguments in registers and others in stack.
static bool CC_MYRISCVXO32(unsigned ValNo, MVT ValVT, MVT LocVT,
                       CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                       CCState &State) {
  static const MCPhysReg IntRegs[] = { MYRISCVX::A0, MYRISCVX::A1 };
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
  // f32 and f64 are allocated in A0, A1 when either of the following
  // is true: function is vararg, argument is 3rd or higher, there is previous
  // argument which is not f32 or f64.
  bool AllocateFloatsInIntReg = true;
  unsigned OrigAlign = ArgFlags.getOrigAlign();
  bool isI64 = (ValVT == MVT::i32 && OrigAlign == 8);
  if (ValVT == MVT::i32 || (ValVT == MVT::f32 && AllocateFloatsInIntReg)) {
    Reg = State.AllocateReg(IntRegs);
    // If this is the first part of an i64 arg,
    // the allocated register must be A0.
    if (isI64 && (Reg == MYRISCVX::A1))
      Reg = State.AllocateReg(IntRegs);
    LocVT = MVT::i32;
  } else if (ValVT == MVT::f64 && AllocateFloatsInIntReg) {
    // Allocate int register. If first
    // available register is MYRISCVX::A1, shadow it too.
    Reg = State.AllocateReg(IntRegs);
    if (Reg == MYRISCVX::A1)
      Reg = State.AllocateReg(IntRegs);
    State.AllocateReg(IntRegs);
    LocVT = MVT::i32;
  } else
    llvm_unreachable("Cannot handle this ValVT.");
  if (!Reg) {
    unsigned Offset = State.AllocateStack(ValVT.getSizeInBits() >> 3,
                                          OrigAlign);
    State.addLoc(CCValAssign::getMem(ValNo, ValVT, Offset, LocVT, LocInfo));
  } else
    State.addLoc(CCValAssign::getReg(ValNo, ValVT, Reg, LocVT, LocInfo));
  return false;
}


void MYRISCVXTargetLowering::
copyByValRegs(SDValue Chain, const SDLoc &DL, std::vector<SDValue> &OutChains,
              SelectionDAG &DAG, const ISD::ArgFlagsTy &Flags,
              SmallVectorImpl<SDValue> &InVals, const Argument *FuncArg,
              const MYRISCVXCC &CC, const ByValArgInfo &ByVal) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned RegAreaSize = ByVal.NumRegs * CC.regSize();
  unsigned FrameObjSize = std::max(Flags.getByValSize(), RegAreaSize);
  int FrameObjOffset;
  const ArrayRef<MCPhysReg> ByValArgRegs = CC.intArgRegs();
  if (RegAreaSize)
    FrameObjOffset = (int)CC.reservedArgArea() -
        (int)((CC.numIntArgRegs() - ByVal.FirstIdx) * CC.regSize());
  else
    FrameObjOffset = ByVal.Address;
  // Create frame object.
  EVT PtrTy = getPointerTy(DAG.getDataLayout());
  int FI = MFI.CreateFixedObject(FrameObjSize, FrameObjOffset, true);
  SDValue FIN = DAG.getFrameIndex(FI, PtrTy);
  InVals.push_back(FIN);
  if (!ByVal.NumRegs)
    return;
  // Copy arg registers.
  MVT RegTy = MVT::getIntegerVT(CC.regSize() * 8);
  const TargetRegisterClass *RC = getRegClassFor(RegTy);
  for (unsigned I = 0; I < ByVal.NumRegs; ++I) {
    unsigned ArgReg = ByValArgRegs[ByVal.FirstIdx + I];
    unsigned VReg = addLiveIn(MF, ArgReg, RC);
    unsigned Offset = I * CC.regSize();
    SDValue StorePtr = DAG.getNode(ISD::ADD, DL, PtrTy, FIN,
                                   DAG.getConstant(Offset, DL, PtrTy));
    SDValue Store = DAG.getStore(Chain, DL, DAG.getRegister(VReg, RegTy),
                                 StorePtr, MachinePointerInfo(FuncArg, Offset));
    OutChains.push_back(Store);
  }
}


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
  MYRISCVXCC::SpecialCallingConvType SpecialCallingConv =
      getSpecialCallingConv(Callee);
  MYRISCVXCC MYRISCVXCCInfo(CallConv, ABI.IsO32(),
                            CCInfo, SpecialCallingConv);
  MYRISCVXCCInfo.analyzeCallOperands(Outs, IsVarArg,
                                     Subtarget.abiUsesSoftFloat(),
                                     Callee.getNode(), CLI.getArgs());
  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NextStackOffset = CCInfo.getNextStackOffset();


#ifdef ENABLE_GPRESTORE
  if (!MYRISCVXReserveGP) {
    // If this is the first call, create a stack frame object that points to
    // a location to which .cprestore saves $gp.
    if (IsPIC && MYRISCVXFI->globalBaseRegFixed() && !MYRISCVXFI->getGPFI())
      MYRISCVXFI->setGPFI(MFI.CreateFixedObject(4, 0, true));
    if (MYRISCVXFI->needGPSaveRestore())
      MFI.setObjectOffset(MYRISCVXFI->getGPFI(), NextStackOffset);
  }
#endif

  //@TailCall 1 {
  // Check if it's really possible to do a tail call.
  if (IsTailCall)
    IsTailCall =
        isEligibleForTailCallOptimization(MYRISCVXCCInfo, NextStackOffset,
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
  MYRISCVXCC::byval_iterator ByValArg = MYRISCVXCCInfo.byval_begin();
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
      // unsigned FirstByValReg, LastByValReg;
      // unsigned ByValIdx = CCInfo.getInRegsParamsProcessed();
      // CCInfo.getInRegsParamInfo(ByValIdx, FirstByValReg, LastByValReg);

      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValArg != MYRISCVXCCInfo.byval_end());
      assert(!IsTailCall &&
             "Do not tail-call optimize if there is a byval argument.");
      passByValArg(Chain, DL, RegsToPass, MemOpChains, StackPtr, MFI, DAG, Arg,
                   MYRISCVXCCInfo, *ByValArg, Flags, true);
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
  // SDValue CalleeLo;
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
  Chain = DAG.getNode(MYRISCVXISD::JmpLink, DL, NodeTys, Ops);
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
  MYRISCVXCC MYRISCVXCCInfo(CallConv, ABI.IsO32(), CCInfo);
  MYRISCVXCCInfo.analyzeCallResult(Ins, Subtarget.abiUsesSoftFloat(),
                                   CallNode, RetTy);
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


bool
MYRISCVXTargetLowering::CanLowerReturn(CallingConv::ID CallConv,
                                       MachineFunction &MF, bool IsVarArg,
                                       const SmallVectorImpl<ISD::OutputArg> &Outs,
                                       LLVMContext &Context) const {
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, IsVarArg, MF,
                 RVLocs, Context);
  return CCInfo.CheckReturn(Outs, RetCC_MYRISCVX);
}


MYRISCVXTargetLowering::MYRISCVXCC::SpecialCallingConvType
MYRISCVXTargetLowering::getSpecialCallingConv(SDValue Callee) const {
  MYRISCVXCC::SpecialCallingConvType SpecialCallingConv =

      MYRISCVXCC::NoSpecialCallingConv;
  return SpecialCallingConv;
}


void MYRISCVXTargetLowering::MYRISCVXCC::
analyzeCallOperands(const SmallVectorImpl<ISD::OutputArg> &Args,
                    bool IsVarArg, bool IsSoftFloat, const SDNode *CallNode,
                    std::vector<ArgListEntry> &FuncArgs) {
  //@analyzeCallOperands body {
  assert((CallConv != CallingConv::Fast || !IsVarArg) &&
         "CallingConv::Fast shouldn't be used for vararg functions.");
  unsigned NumOpnds = Args.size();
  llvm::CCAssignFn *FixedFn = fixedArgFn();
  llvm::CCAssignFn *VarFn = varArgFn();

  //@3 {
  for (unsigned I = 0; I != NumOpnds; ++I) {
    //@3 }
    MVT ArgVT = Args[I].VT;
    ISD::ArgFlagsTy ArgFlags = Args[I].Flags;
    bool R;
    if (ArgFlags.isByVal()) {
      handleByValArg(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags);
      continue;
    }

    if (IsVarArg && !Args[I].IsFixed)
      R = VarFn(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags, CCInfo);
    else
    {
      MVT RegVT = getRegVT(ArgVT, FuncArgs[Args[I].OrigArgIndex].Ty, CallNode,
                           IsSoftFloat);
      R = FixedFn(I, ArgVT, RegVT, CCValAssign::Full, ArgFlags, CCInfo);
    }
    if (R) {
#ifndef NDEBUG
      dbgs() << "Call operand #" << I << " has unhandled type "
             << EVT(ArgVT).getEVTString();
#endif
      llvm_unreachable(nullptr);
    }
  }
}


static const MCPhysReg O32IntRegs[] = {
  MYRISCVX::A0, MYRISCVX::A1, MYRISCVX::A2, MYRISCVX::A3,
  MYRISCVX::A4, MYRISCVX::A5, MYRISCVX::A6, MYRISCVX::A7
};

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


void MYRISCVXTargetLowering::
getOpndList(SmallVectorImpl<SDValue> &Ops,
            std::deque< std::pair<unsigned, SDValue> > &RegsToPass,
            bool IsPICCall, bool GlobalOrExternal, bool InternalLinkage,
            CallLoweringInfo &CLI, SDValue Callee, SDValue Chain) const {
  // T9 should contain the address of the callee function if
  // -reloction-model=pic or it is an indirect call.
  if (IsPICCall || !GlobalOrExternal) {
    unsigned GPReg = MYRISCVX::GP;
    RegsToPass.push_front(std::make_pair(GPReg, Callee));
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


// Copy byVal arg to registers and stack.
void MYRISCVXTargetLowering::
passByValArg(SDValue Chain, const SDLoc &DL,
             std::deque< std::pair<unsigned, SDValue> > &RegsToPass,
             SmallVectorImpl<SDValue> &MemOpChains, SDValue StackPtr,
             MachineFrameInfo &MFI, SelectionDAG &DAG, SDValue Arg,
             const MYRISCVXCC &CC, const ByValArgInfo &ByVal,
             const ISD::ArgFlagsTy &Flags, bool isLittle) const {
  unsigned ByValSizeInBytes = Flags.getByValSize();
  unsigned OffsetInBytes = 0; // From beginning of struct
  unsigned RegSizeInBytes = CC.regSize();
  unsigned Alignment = std::min(Flags.getByValAlign(), RegSizeInBytes);
  EVT PtrTy = getPointerTy(DAG.getDataLayout()),
      RegTy = MVT::getIntegerVT(RegSizeInBytes * 8);

  if (ByVal.NumRegs) {
    const ArrayRef<MCPhysReg> ArgRegs = CC.intArgRegs();
    bool LeftoverBytes = (ByVal.NumRegs * RegSizeInBytes > ByValSizeInBytes);
    unsigned I = 0;

    // Copy words to registers.
    for (; I < ByVal.NumRegs - LeftoverBytes;
         ++I, OffsetInBytes += RegSizeInBytes) {
      SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                    DAG.getConstant(OffsetInBytes, DL, PtrTy));
      SDValue LoadVal = DAG.getLoad(RegTy, DL, Chain, LoadPtr,
                                    MachinePointerInfo());
      MemOpChains.push_back(LoadVal.getValue(1));
      unsigned ArgReg = ArgRegs[ByVal.FirstIdx + I];
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

      unsigned ArgReg = ArgRegs[ByVal.FirstIdx + I];
      RegsToPass.push_back(std::make_pair(ArgReg, Val));
      return;
    }
  }

  // Copy remainder of byval arg to it with memcpy.
  unsigned MemCpySize = ByValSizeInBytes - OffsetInBytes;
  SDValue Src = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                            DAG.getConstant(OffsetInBytes, DL, PtrTy));
  SDValue Dst = DAG.getNode(ISD::ADD, DL, PtrTy, StackPtr,
                            DAG.getIntPtrConstant(ByVal.Address, DL));
  Chain = DAG.getMemcpy(Chain, DL, Dst, Src,
                        DAG.getConstant(MemCpySize, DL, PtrTy),
                        Alignment, /*isVolatile=*/false, /*AlwaysInline=*/false,
                        /*isTailCall=*/false,
                        MachinePointerInfo(), MachinePointerInfo());
  MemOpChains.push_back(Chain);
}



unsigned MYRISCVXTargetLowering::MYRISCVXCC::numIntArgRegs() const {
  return IsO32 ? array_lengthof(O32IntRegs) : 0;
}

const ArrayRef<MCPhysReg> MYRISCVXTargetLowering::MYRISCVXCC::intArgRegs() const {
  return makeArrayRef(O32IntRegs);
}


llvm::CCAssignFn *MYRISCVXTargetLowering::MYRISCVXCC::fixedArgFn() const {
  if (IsO32)
    return CC_MYRISCVXO32;

  else // IsS32
    return CC_MYRISCVXS32;
}



llvm::CCAssignFn *MYRISCVXTargetLowering::MYRISCVXCC::varArgFn() const {
  if (IsO32)
    return CC_MYRISCVXO32;
  else // IsS32
    return CC_MYRISCVXS32;
}


void MYRISCVXTargetLowering::writeVarArgRegs(std::vector<SDValue> &OutChains,
                                         const MYRISCVXCC &CC, SDValue Chain,
                                         const SDLoc &DL, SelectionDAG &DAG) const {
  unsigned NumRegs = CC.numIntArgRegs();
  const ArrayRef<MCPhysReg> ArgRegs = CC.intArgRegs();
  const CCState &CCInfo = CC.getCCInfo();
  unsigned Idx = CCInfo.getFirstUnallocated(ArgRegs);
  unsigned RegSize = CC.regSize();
  MVT RegTy = MVT::getIntegerVT(RegSize * 8);
  const TargetRegisterClass *RC = getRegClassFor(RegTy);
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();

  // Offset of the first variable argument from stack pointer.
  int VaArgOffset;
  if (NumRegs == Idx)
    VaArgOffset = alignTo(CCInfo.getNextStackOffset(), RegSize);
  else
    VaArgOffset = (int)CC.reservedArgArea() - (int)(RegSize * (NumRegs - Idx));

  // Record the frame index of the first variable argument
  // which is a value necessary to VASTART.

  int FI = MFI.CreateFixedObject(RegSize, VaArgOffset, true);
  MYRISCVXFI->setVarArgsFrameIndex(FI);

  // Copy the integer registers that have not been used for argument passing
  // to the argument register save area. For O32, the save area is allocated
  // in the caller's stack frame, while for N32/64, it is allocated in the
  // callee's stack frame.
  for (unsigned I = Idx; I < NumRegs; ++I, VaArgOffset += RegSize) {
    unsigned Reg = addLiveIn(MF, ArgRegs[I], RC);
    SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, RegTy);
    FI = MFI.CreateFixedObject(RegSize, VaArgOffset, true);
    SDValue PtrOff = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
    SDValue Store = DAG.getStore(Chain, DL, ArgValue, PtrOff,
                                 MachinePointerInfo());
    cast<StoreSDNode>(Store.getNode())->getMemOperand()->setValue(
        (Value *)nullptr);
    OutChains.push_back(Store);
  }
}


SDValue MYRISCVXTargetLowering::
lowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const {
  // check the depth
  assert((cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue() == 0) &&
         "Frame address can only be determined for current frame.");

  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
  MFI.setFrameAddressIsTaken(true);
  EVT VT = Op.getValueType();
  SDLoc DL(Op);
  SDValue FrameAddr = DAG.getCopyFromReg(
      DAG.getEntryNode(), DL, MYRISCVX::S0, VT);
  return FrameAddr;
}


SDValue MYRISCVXTargetLowering::lowerRETURNADDR(SDValue Op,
                                                SelectionDAG &DAG) const {
  if (verifyReturnAddressArgumentIsConstant(Op, DAG))
    return SDValue();

  // check the depth
  assert((cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue() == 0) &&
         "Return address can be determined only for current frame.");

  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MVT VT = Op.getSimpleValueType();
  unsigned RA = MYRISCVX::RA;
  MFI.setReturnAddressIsTaken(true);

  // Return RA, which contains the return address. Mark it an implicit live-in.
  unsigned Reg = MF.addLiveIn(RA, getRegClassFor(VT));
  return DAG.getCopyFromReg(DAG.getEntryNode(), SDLoc(Op), Reg, VT);
}


// An EH_RETURN is the result of lowering llvm.eh.return which in turn is
// generated from __builtin_eh_return (offset, handler)
// The effect of this is to adjust the stack pointer by "offset"
// and then branch to "handler".
SDValue MYRISCVXTargetLowering::lowerEH_RETURN(SDValue Op, SelectionDAG &DAG)
    const {
  MachineFunction &MF = DAG.getMachineFunction();
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();

  MYRISCVXFI->setCallsEhReturn();
  SDValue Chain     = Op.getOperand(0);
  SDValue Offset    = Op.getOperand(1);
  SDValue Handler   = Op.getOperand(2);
  SDLoc DL(Op);
  EVT Ty = MVT::i32;

  // Store stack offset in A1, store jump target in V0. Glue CopyToReg and
  // EH_RETURN nodes, so that instructions are emitted back-to-back.
  unsigned OffsetReg = MYRISCVX::A1;
  unsigned AddrReg = MYRISCVX::A0;
  Chain = DAG.getCopyToReg(Chain, DL, OffsetReg, Offset, SDValue());
  Chain = DAG.getCopyToReg(Chain, DL, AddrReg, Handler, Chain.getValue(1));
  return DAG.getNode(MYRISCVXISD::EH_RETURN, DL, MVT::Other, Chain,
                     DAG.getRegister(OffsetReg, Ty),
                     DAG.getRegister(AddrReg, getPointerTy(MF.getDataLayout())),
                     Chain.getValue(1));
}


SDValue MYRISCVXTargetLowering::lowerADD(SDValue Op, SelectionDAG &DAG) const {
  if ((Op->getOperand(0).getOpcode() != ISD::FRAMEADDR) ||
      (cast<ConstantSDNode>(Op->getOperand(0).getOperand(0))->getZExtValue() != 0) ||
      Op->getOperand(1).getOpcode() != ISD::FRAME_TO_ARGS_OFFSET)
    return SDValue();

  MachineFunction &MF = DAG.getMachineFunction();
  MYRISCVXFunctionInfo *MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();

  MYRISCVXFI->setCallsEhDwarf();
  return Op;
}


//===----------------------------------------------------------------------===//
//                           MYRISCVX Inline Assembly Support
//===----------------------------------------------------------------------===//

/// getConstraintType - Given a constraint letter, return the type of
/// constraint it is for this target.
MYRISCVXTargetLowering::ConstraintType
MYRISCVXTargetLowering::getConstraintType(StringRef Constraint) const
{
  // MYRISCVX specific constraints
  // GCC config/mips/constraints.md
  // 'c' : A register suitable for use in an indirect
  //       jump. This will always be $t9 for -mabicalls.
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
      default : break;
      case 'c':
        return C_RegisterClass;
      case 'R':
        return C_Memory;
    }
  }
  return TargetLowering::getConstraintType(Constraint);
}

/// Examine constraint type and operand type and determine a weight value.
/// This object must already have been set up with the operand type
/// and the current alternative constraint selected.
TargetLowering::ConstraintWeight
MYRISCVXTargetLowering::getSingleConstraintMatchWeight(
    AsmOperandInfo &info, const char *constraint) const {
  ConstraintWeight weight = CW_Invalid;
  Value *CallOperandVal = info.CallOperandVal;
  // If we don't have a value, we can't do a match,
  // but allow it at the lowest weight.
  if (!CallOperandVal)
    return CW_Default;
  Type *type = CallOperandVal->getType();
  // Look at the constraint type.
  switch (*constraint) {
    default:
      weight = TargetLowering::getSingleConstraintMatchWeight(info, constraint);
      break;
    case 'c': // $t9 for indirect jumps
      if (type->isIntegerTy())
        weight = CW_SpecificReg;
      break;
    case 'I': // signed 16 bit immediate
    case 'J': // integer zero
    case 'K': // unsigned 16 bit immediate
    case 'L': // signed 32 bit immediate where lower 16 bits are 0
    case 'N': // immediate in the range of -65535 to -1 (inclusive)
    case 'O': // signed 15 bit immediate (+- 16383)
    case 'P': // immediate in the range of 65535 to 1 (inclusive)
      if (isa<ConstantInt>(CallOperandVal))
        weight = CW_Constant;
      break;
    case 'R':
      weight = CW_Memory;
      break;
  }
  return weight;
}

/// This is a helper function to parse a physical register string and split it
/// into non-numeric and numeric parts (Prefix and Reg). The first boolean flag
/// that is returned indicates whether parsing was successful. The second flag
/// is true if the numeric part exists.
static std::pair<bool, bool>
parsePhysicalReg(const StringRef &C, std::string &Prefix,
                 unsigned long long &Reg) {
  if (C.front() != '{' || C.back() != '}')
    return std::make_pair(false, false);

  // Search for the first numeric character.
  StringRef::const_iterator I, B = C.begin() + 1, E = C.end() - 1;
  I = std::find_if(B, E, std::ptr_fun(isdigit));

  Prefix.assign(B, I - B);

  // The second flag is set to false if no numeric characters were found.
  if (I == E)
    return std::make_pair(true, false);

  // Parse the numeric characters.
  return std::make_pair(!getAsUnsignedInteger(StringRef(I, E - I), 10, Reg),
                        true);
}

std::pair<unsigned, const TargetRegisterClass *> MYRISCVXTargetLowering::
parseRegForInlineAsmConstraint(const StringRef &C, MVT VT) const {
  const TargetRegisterClass *RC;
  std::string Prefix;
  unsigned long long Reg;

  std::pair<bool, bool> R = parsePhysicalReg(C, Prefix, Reg);

  if (!R.first)
    return std::make_pair(0U, nullptr);
  if (!R.second)
    return std::make_pair(0U, nullptr);

  // Parse $0-$15.
  assert(Prefix == "$");
  RC = getRegClassFor((VT == MVT::Other) ? MVT::i32 : VT);

  assert(Reg < RC->getNumRegs());
  return std::make_pair(*(RC->begin() + Reg), RC);
}

/// Given a register class constraint, like 'r', if this corresponds directly
/// to an LLVM register class, return a register of 0 and the register class
/// pointer.
std::pair<unsigned, const TargetRegisterClass *>
MYRISCVXTargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                                                     StringRef Constraint,
                                                     MVT VT) const
{
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
      case 'r':
        if (VT == MVT::i32 || VT == MVT::i16 || VT == MVT::i8) {
          return std::make_pair(0U, &MYRISCVX::GPRRegClass);
        }
        if (VT == MVT::i64)
          return std::make_pair(0U, &MYRISCVX::GPRRegClass);
        // This will generate an error message
        return std::make_pair(0u, static_cast<const TargetRegisterClass*>(0));
      case 'c': // register suitable for indirect jump
        if (VT == MVT::i32)
          return std::make_pair((unsigned)MYRISCVX::T0, &MYRISCVX::GPRRegClass);
        assert("Unexpected type.");
    }
  }

  std::pair<unsigned, const TargetRegisterClass *> R;
  R = parseRegForInlineAsmConstraint(Constraint, VT);

  if (R.second)
    return R;

  return TargetLowering::getRegForInlineAsmConstraint(TRI, Constraint, VT);
}

/// LowerAsmOperandForConstraint - Lower the specified operand into the Ops
/// vector.  If it is invalid, don't add anything to Ops.
void MYRISCVXTargetLowering::LowerAsmOperandForConstraint(SDValue Op,
                                                          std::string &Constraint,
                                                          std::vector<SDValue>&Ops,
                                                          SelectionDAG &DAG) const {
  SDLoc DL(Op);
  SDValue Result;

  // Only support length 1 constraints for now.
  if (Constraint.length() > 1) return;

  char ConstraintLetter = Constraint[0];
  switch (ConstraintLetter) {
    default: break; // This will fall through to the generic implementation
    case 'I': // Signed 16 bit constant
      // If this fails, the parent routine will give an error
      if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
        EVT Type = Op.getValueType();
        int64_t Val = C->getSExtValue();
        if (isInt<16>(Val)) {
          Result = DAG.getTargetConstant(Val, DL, Type);
          break;
        }
      }
      return;
    case 'J': // integer zero
      if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
        EVT Type = Op.getValueType();
        int64_t Val = C->getZExtValue();
        if (Val == 0) {
          Result = DAG.getTargetConstant(0, DL, Type);
          break;
        }
      }
      return;
    case 'K': // unsigned 16 bit immediate
      if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
        EVT Type = Op.getValueType();
        uint64_t Val = (uint64_t)C->getZExtValue();
        if (isUInt<16>(Val)) {
          Result = DAG.getTargetConstant(Val, DL, Type);
          break;
        }
      }
      return;
    case 'L': // signed 32 bit immediate where lower 16 bits are 0
      if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
        EVT Type = Op.getValueType();
        int64_t Val = C->getSExtValue();
        if ((isInt<32>(Val)) && ((Val & 0xffff) == 0)){
          Result = DAG.getTargetConstant(Val, DL, Type);
          break;
        }
      }
      return;
    case 'N': // immediate in the range of -65535 to -1 (inclusive)
      if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
        EVT Type = Op.getValueType();
        int64_t Val = C->getSExtValue();
        if ((Val >= -65535) && (Val <= -1)) {
          Result = DAG.getTargetConstant(Val, DL, Type);
          break;
        }
      }
      return;
    case 'O': // signed 15 bit immediate
      if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
        EVT Type = Op.getValueType();
        int64_t Val = C->getSExtValue();
        if ((isInt<15>(Val))) {
          Result = DAG.getTargetConstant(Val, DL, Type);
          break;
        }
      }
      return;
    case 'P': // immediate in the range of 1 to 65535 (inclusive)
      if (ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op)) {
        EVT Type = Op.getValueType();
        int64_t Val = C->getSExtValue();
        if ((Val <= 65535) && (Val >= 1)) {
          Result = DAG.getTargetConstant(Val, DL, Type);
          break;
        }
      }
      return;
  }

  if (Result.getNode()) {
    Ops.push_back(Result);
    return;
  }

  TargetLowering::LowerAsmOperandForConstraint(Op, Constraint, Ops, DAG);
}

bool MYRISCVXTargetLowering::isLegalAddressingMode(const DataLayout &DL,
                                                   const AddrMode &AM, Type *Ty,
                                                   unsigned AS,
                                                   Instruction *I) const {
  // No global is ever allowed as a base.
  if (AM.BaseGV)
    return false;

  switch (AM.Scale) {
    case 0: // "r+i" or just "i", depending on HasBaseReg.
      break;
    case 1:
      if (!AM.HasBaseReg) // allow "r+i".
        break;
      return false; // disallow "r+r" or "r+r+i".
    default:
      return false;
  }

  return true;
}
