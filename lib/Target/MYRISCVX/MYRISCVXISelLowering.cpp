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
#define DEBUG_TYPE "MYRISCVX-lower"


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

  //- Set .align 2
  // It will emit .align 2 later
  setMinFunctionAlignment(2);
  // must, computeRegisterProperties - Once all of the register classes are
  // added, this allows us to compute derived properties we expose.
  computeRegisterProperties(STI.getRegisterInfo());

}

const MYRISCVXTargetLowering *MYRISCVXTargetLowering::create(const MYRISCVXTargetMachine &TM,
                                                             const MYRISCVXSubtarget &STI) {
  return llvm::createMYRISCVXSETargetLowering(TM, STI);
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
  return Chain;
}
// @LowerFormalArguments }

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
LowerOperation(SDValue Op, SelectionDAG &DAG) const
{
  switch (Op.getOpcode())
  {
  case ISD::GlobalAddress:      return lowerGlobalAddress(Op, DAG);
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


//@getTargetNode(ExternalSymbolSDNode
SDValue MYRISCVXTargetLowering::getTargetNode(ExternalSymbolSDNode *N, EVT Ty,
                                              SelectionDAG &DAG,
                                              unsigned Flag) const {
  return DAG.getTargetExternalSymbol(N->getSymbol(), Ty, Flag);
}
