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

/// CCIfSubtarget - Match if the current subtarget has a feature F.

/// CCIfSubtarget - Match if the current subtarget has a feature F.
class CCIfSubtarget<string F, CCAction A, string Invert = "">
    : CCIf<!strconcat(Invert,
                      "static_cast<const MYRISCVXSubtarget&>"
                      "(State.getMachineFunction().getSubtarget()).",
                      F),
           A>;

def CSR_LP32 : CalleeSavedRegs<(add SP, FP, S1, (sequence "S%u", 2, 11))>;


//===----------------------------------------------------------------------===//
// MYRISCVX LP32/STACK32 Calling Convention
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

def CC_STACK32 : CallingConv<[
  // Promote i8/i16 arguments to i32.
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,

  // Integer values get stored in stack slots that are 4 bytes in
  // size and 4-byte aligned.
  CCIfType<[i32], CCAssignToStack<4, 4>>
]>;

//===----------------------------------------------------------------------===//
// MYRISCVX LP64/STACK64 Calling Convention
//===----------------------------------------------------------------------===//

def CC_LP64 : CallingConv<[
  // Promote i1/i8/i16 arguments to i32.
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,

  // Integer arguments are passed in integer registers.
  CCIfType<[i32, i64], CCAssignToReg<[A0, A1, A2, A3, A4, A5, A6, A7]>>,

  // Integer values get stored in stack slots that are 4 bytes in
  // size and 4-byte aligned.
  CCIfType<[i32], CCAssignToStack<4, 4>>,
  CCIfType<[i64], CCAssignToStack<8, 8>>
]>;

def CC_STACK64 : CallingConv<[
  // Promote i8/i16/i32 arguments to i64.
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,

  // Integer values get stored in stack slots that are 8 bytes in
  // size and 8-byte aligned.
  CCIfType<[i32], CCAssignToStack<4, 4>>,
  CCIfType<[i64], CCAssignToStack<8, 8>>
]>;

def CC_MYRISCVX : CallingConv<[
  CCIfSubtarget<"isABI_STACK64()", CCDelegateTo<CC_STACK64>>,
  CCIfSubtarget<"isABI_LP64   ()", CCDelegateTo<CC_LP64>>,
  CCIfSubtarget<"isABI_STACK32()", CCDelegateTo<CC_STACK32>>,
  CCDelegateTo<CC_LP32>
]>;
