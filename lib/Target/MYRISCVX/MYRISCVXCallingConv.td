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
class CCIfSubtarget<string F, CCAction A>:
  CCIf<!strconcat("State.getTarget().getSubtarget<MYRISCVXSubtarget>().", F), A>;

def CSR_LP32 : CalleeSavedRegs<(add SP, FP, S1, (sequence "S%u", 2, 11))>;
