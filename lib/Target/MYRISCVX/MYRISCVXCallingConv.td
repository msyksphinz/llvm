//===-- MYRISCVXCallingConv.td - Calling Conventions for MYRISCVX --*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===------------------------------------------------------------------------------===//
// This describes the calling conventions for MYRISCVX architecture.
//===------------------------------------------------------------------------------===//

def CSR_LP32 : CalleeSavedRegs<(add SP, FP, S1, (sequence "S%u", 2, 11))>;

/// CCIfSubtarget - Match if the current subtarget has a feature F.
class CCIfSubtarget<string F, CCAction A, string Invert = "">
    : CCIf<!strconcat(Invert,
                      "static_cast<const MYRISCVXSubtarget&>"
			"(State.getMachineFunction().getSubtarget()).",
                      F),
           A>;

//===----------------------------------------------------------------------===//
// MYRISCVX LP32 Calling Convention
//===----------------------------------------------------------------------===//

def CC_LP32 : CallingConv<[
  // Promote i8/i16 arguments to i32.
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,

  // Integer arguments are passed in integer registers.
  CCIfType<[i32], CCAssignToReg<[A0, A1, A2, A3, A4, A5, A6, A7]>>,

  // Integer values get stored in stack slots that are 4 bytes in
  // size and 4-byte aligned.
  CCIfType<[i32], CCAssignToStack<4, 4>>
]>;

//===----------------------------------------------------------------------===//
// MYRISCVX STACK32 Calling Convention
//===----------------------------------------------------------------------===//

def CC_STACK32 : CallingConv<[
  // Promote i8/i16 arguments to i32.
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,

  // Integer values get stored in stack slots that are 4 bytes in
  // size and 4-byte aligned.
  CCIfType<[i32], CCAssignToStack<4, 4>>
]>;


def CC_MYRISCVX : CallingConv<[
  CCIfSubtarget<"isABI_STACK32()", CCDelegateTo<CC_STACK32>>,
  CCDelegateTo<CC_LP32>
]>;


def RetCC_MYRISCVXEABI : CallingConv<[
  // i32 are returned in registers A0, A1
  CCIfType<[i32], CCAssignToReg<[A0, A1]>>
]>;

def RetCC_MYRISCVX : CallingConv<[
  CCDelegateTo<RetCC_MYRISCVXEABI>
]>;
