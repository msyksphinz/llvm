//===-- MYRISCVXCallingConv.td - Calling Conventions for MYRISCVX --*- tablegen -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for MYRISCVX architecture.
//===----------------------------------------------------------------------===//

/// CCIfSubtarget - Match if the current subtarget has a feature F.
class CCIfSubtarget<string F, CCAction A>:
  CCIf<!strconcat("State.getTarget().getSubtarget<MYRISCVXSubtarget>().", F), A>;

def CSR : CalleeSavedRegs<(add SP, (sequence "S%u", 0, 11))>;

def RetCC_MYRISCVXEABI : CallingConv<[
  // i32 are returned in registers A0, A1, A2, A3, A4, A5, A6, A7
  CCIfType<[i32], CCAssignToReg<[A0, A1, A2, A3, A4, A5, A6, A7]>>
]>;

def RetCC_MYRISCVX : CallingConv<[
  CCDelegateTo<RetCC_MYRISCVXEABI>
]>;
