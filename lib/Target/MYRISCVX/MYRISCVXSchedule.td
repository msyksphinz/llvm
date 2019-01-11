//===-- MYRISCVXSchedule.td - MYRISCVX Scheduling Definitions ------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Functional units across MYRISCVX chips sets. Based on GCC/MYRISCVX backend files.
//===----------------------------------------------------------------------===//
def ALU     : FuncUnit;
def IMULDIV : FuncUnit;

//===----------------------------------------------------------------------===//
// Instruction Itinerary classes used for MYRISCVX
//===----------------------------------------------------------------------===//
def IIAlu            : InstrItinClass;
def IILoad           : InstrItinClass;
def IIStore          : InstrItinClass;
def IIBranch         : InstrItinClass;

def IIPseudo         : InstrItinClass;
