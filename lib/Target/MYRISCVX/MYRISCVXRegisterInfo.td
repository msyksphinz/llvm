//===-- MYRISCVXRegisterInfo.td - MYRISCVX Register defs -----------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the MYRISCVX register file
//===----------------------------------------------------------------------===//

// We have banks of 16 registers each.
class MYRISCVXReg<bits<16> Enc, string n> : Register<n> {
  // For tablegen(... -gen-emitter)  in CMakeLists.txt
  let HWEncoding = Enc;

  let Namespace = "MYRISCVX";
}

// MYRISCVX CPU Registers
class MYRISCVXGPRReg<bits<16> Enc, string n> : MYRISCVXReg<Enc, n>;

// Co-processor 0 Registers
class MYRISCVXC0Reg<bits<16> Enc, string n> : MYRISCVXReg<Enc, n>;

//===----------------------------------------------------------------------===//
//@Registers
//===----------------------------------------------------------------------===//
// The register string, such as "9" or "gp" will show on "llvm-objdump -d"
//@ All registers definition
let Namespace = "MYRISCVX" in {
  //@ General Purpose Registers
  def ZERO : MYRISCVXGPRReg<0,  "zero">, DwarfRegNum<[0]>;
}

//===----------------------------------------------------------------------===//
//@Register Classes
//===----------------------------------------------------------------------===//

def CPURegs : RegisterClass<"MYRISCVX", [i32], 32, (add ZERO)>;
