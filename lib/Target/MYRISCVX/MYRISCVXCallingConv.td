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

def CC_MYRISCVX_ByVal : CallingConv<[
  CCIfByVal<CCPassByVal<4, 4>>
]>;

//===----------------------------------------------------------------------===//
// MYRISCVX LP32 Calling Convention
//===----------------------------------------------------------------------===//
def CC_LP32 : CallingConv<[
  CCIfByVal<CCDelegateTo<CC_MYRISCVX_ByVal>>,

  // Promote i8/i16 arguments to i32.
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,

  // Integer arguments are passed in integer registers.
  CCIfType<[i32, i64], CCAssignToReg<[A0, A1, A2, A3, A4, A5, A6, A7]>>,
  // Single Floating-Point arguments are passed in FP registers
  CCIfType<[f32], CCAssignToReg<[F10_S, F11_S, F12_S, F13_S, F14_S, F15_S, F16_S, F17_S]>>,
  // Double Floating-Point arguments are passed in FP registers
  CCIfType<[f64], CCAssignToReg<[F10_D, F11_D, F12_D, F13_D, F14_D, F15_D, F16_D, F17_D]>>,

  // Integer values get stored in stack slots that are 4 bytes in size and 4-byte aligned.
  CCIfType<[i32, i64], CCAssignToStack<4, 4>>,
  // Floating-Point values get stored in stack slots that are 4 bytes in size and 4-byte aligned.
  CCIfType<[f32], CCAssignToStack<4, 4>>,
  // Floating-Point values get stored in stack slots that are 8 bytes in size and 8-byte aligned.
  CCIfType<[f64], CCAssignToStack<8, 8>>
]>;

//===----------------------------------------------------------------------===//
// MYRISCVX STACK32 Calling Convention
//===----------------------------------------------------------------------===//
def CC_STACK32 : CallingConv<[
  CCIfByVal<CCDelegateTo<CC_MYRISCVX_ByVal>>,

  // Promote i8/i16 arguments to i32.
  CCIfType<[i1, i8, i16], CCPromoteToType<i32>>,

  // Integer values get stored in stack slots that are 4 bytes in size and 4-byte aligned.
  CCIfType<[i32, i64], CCAssignToStack<4, 4>>,
  // Floating-Point values get stored in stack slots that are 4 bytes in size and 4-byte aligned.
  CCIfType<[f32], CCAssignToStack<4, 4>>,
  // Floating-Point values get stored in stack slots that are 8 bytes in size and 8-byte aligned.
  CCIfType<[f64], CCAssignToStack<8, 8>>
]>;


def CC_MYRISCVX : CallingConv<[
  CCIfSubtarget<"isABI_STACK32()", CCDelegateTo<CC_STACK32>>,
  CCDelegateTo<CC_LP32>
]>;


def RetCC_MYRISCVXEABI : CallingConv<[
  // i32 are returned in registers A0, A1
  CCIfType<[i32, i64], CCAssignToReg<[A0, A1]>>,

  // Floating-Point are return in registers FA0, FA1
  CCIfType<[f32], CCAssignToReg<[F10_S]>>,
  CCIfType<[f64], CCAssignToReg<[F10_D]>>
]>;

def RetCC_MYRISCVX : CallingConv<[
  CCDelegateTo<RetCC_MYRISCVXEABI>
]>;
