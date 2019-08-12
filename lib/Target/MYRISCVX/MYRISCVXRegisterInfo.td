//===-- MYRISCVXRegisterInfo.td - RISC-V Register defs -----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the MYRISCVX register files
//===----------------------------------------------------------------------===//

let Namespace = "MYRISCVX" in {
// We have banks of 32 registers each.
  class MYRISCVXReg<bits<5> Enc, string n, list<string> alt = []> : Register<n> {
    let HWEncoding{4-0} = Enc;
    let AltNames = alt;
  }

  // RISCV CPU Registers
  class MYRISCVXGPRReg<bits<5> Enc, string n, list<string> alt> : MYRISCVXReg<Enc, n>;

  def sub_32 : SubRegIndex<32>;
  // MYRISCVX Double Floating-Point Registers
  class MYRISCVXFPR_D<bits<5> Enc, list<string> alt,
                      MYRISCVXGPRReg subreg, list<SubRegIndex> subreg_indices> : Register<""> {
    let HWEncoding{4-0} = Enc;
    let SubRegs  = [subreg];
    let SubRegIndices = [sub_32];
    let AsmName  = subreg.AsmName;
    let AltNames = subreg.AltNames;
  }
}

//===----------------------------------------------------------------------===//
//  Registers
//===----------------------------------------------------------------------===//

let Namespace = "MYRISCVX" in {
  def ZERO : MYRISCVXGPRReg<0,  "zero", ["zero"]>,  DwarfRegNum<[0]>;

  def RA   : MYRISCVXGPRReg<1,  "x1", ["ra"]>,    DwarfRegNum<[1]>;
  def SP   : MYRISCVXGPRReg<2,  "x2", ["sp"]>,    DwarfRegNum<[2]>;
  def GP   : MYRISCVXGPRReg<3,  "x3", ["gp"]>,    DwarfRegNum<[3]>;
  def TP   : MYRISCVXGPRReg<4,  "x4", ["tp"]>,    DwarfRegNum<[4]>;
  def T0   : MYRISCVXGPRReg<5,  "x5", ["t0"]>,    DwarfRegNum<[5]>;
  def T1   : MYRISCVXGPRReg<6,  "x6", ["t1"]>,    DwarfRegNum<[6]>;
  def T2   : MYRISCVXGPRReg<7,  "x7", ["t2"]>,    DwarfRegNum<[7]>;
  def FP   : MYRISCVXGPRReg<8,  "x8", ["s0", "fp"]>, DwarfRegNum<[8]>;  // used as FP
  def S1   : MYRISCVXGPRReg<9,  "x9", ["s1"]>,    DwarfRegNum<[9]>;

  def A0   : MYRISCVXGPRReg<10, "x10", ["a0"]>,   DwarfRegNum<[10]>;
  def A1   : MYRISCVXGPRReg<11, "x11", ["a1"]>,   DwarfRegNum<[11]>;
  def A2   : MYRISCVXGPRReg<12, "x12", ["a2"]>,   DwarfRegNum<[12]>;
  def A3   : MYRISCVXGPRReg<13, "x13", ["a3"]>,   DwarfRegNum<[13]>;
  def A4   : MYRISCVXGPRReg<14, "x14", ["a4"]>,   DwarfRegNum<[10]>;
  def A5   : MYRISCVXGPRReg<15, "x15", ["a5"]>,   DwarfRegNum<[11]>;
  def A6   : MYRISCVXGPRReg<16, "x16", ["a6"]>,   DwarfRegNum<[12]>;
  def A7   : MYRISCVXGPRReg<17, "x17", ["a7"]>,   DwarfRegNum<[13]>;

  def S2   : MYRISCVXGPRReg<18, "x18", ["s2"]>,   DwarfRegNum<[18]>;
  def S3   : MYRISCVXGPRReg<19, "x19", ["s3"]>,   DwarfRegNum<[19]>;
  def S4   : MYRISCVXGPRReg<20, "x20", ["s4"]>,   DwarfRegNum<[20]>;
  def S5   : MYRISCVXGPRReg<21, "x21", ["s5"]>,   DwarfRegNum<[21]>;
  def S6   : MYRISCVXGPRReg<22, "x22", ["s6"]>,   DwarfRegNum<[22]>;
  def S7   : MYRISCVXGPRReg<23, "x23", ["s7"]>,   DwarfRegNum<[23]>;
  def S8   : MYRISCVXGPRReg<24, "x24", ["s8"]>,   DwarfRegNum<[24]>;
  def S9   : MYRISCVXGPRReg<25, "x25", ["s9"]>,   DwarfRegNum<[25]>;
  def S10  : MYRISCVXGPRReg<26, "x26", ["s10"]>,   DwarfRegNum<[26]>;
  def S11  : MYRISCVXGPRReg<27, "x27", ["s11"]>,   DwarfRegNum<[27]>;

  def T3   : MYRISCVXGPRReg<28, "x28", ["t3"]>,   DwarfRegNum<[28]>;
  def T4   : MYRISCVXGPRReg<29, "x29", ["t4"]>,   DwarfRegNum<[29]>;
  def T5   : MYRISCVXGPRReg<30, "x30", ["t5"]>,   DwarfRegNum<[30]>;
  def T6   : MYRISCVXGPRReg<31, "x31", ["t6"]>,   DwarfRegNum<[31]>;
}

// Floating point registers
let Namespace = "MYRISCVX" in {
  def F0_S  : MYRISCVXGPRReg< 0,  "f0", ["ft0" ]>, DwarfRegNum<[32]>;
  def F1_S  : MYRISCVXGPRReg< 1,  "f1", ["ft1" ]>, DwarfRegNum<[33]>;
  def F2_S  : MYRISCVXGPRReg< 2,  "f2", ["ft2" ]>, DwarfRegNum<[34]>;
  def F3_S  : MYRISCVXGPRReg< 3,  "f3", ["ft3" ]>, DwarfRegNum<[35]>;
  def F4_S  : MYRISCVXGPRReg< 4,  "f4", ["ft4" ]>, DwarfRegNum<[36]>;
  def F5_S  : MYRISCVXGPRReg< 5,  "f5", ["ft5" ]>, DwarfRegNum<[37]>;
  def F6_S  : MYRISCVXGPRReg< 6,  "f6", ["ft6" ]>, DwarfRegNum<[38]>;
  def F7_S  : MYRISCVXGPRReg< 7,  "f7", ["ft7" ]>, DwarfRegNum<[39]>;
  def F8_S  : MYRISCVXGPRReg< 8,  "f8", ["fs0" ]>, DwarfRegNum<[40]>;
  def F9_S  : MYRISCVXGPRReg< 9,  "f9", ["fs1" ]>, DwarfRegNum<[41]>;
  def F10_S : MYRISCVXGPRReg<10, "f10", ["fa0" ]>, DwarfRegNum<[42]>;
  def F11_S : MYRISCVXGPRReg<11, "f11", ["fa1" ]>, DwarfRegNum<[43]>;
  def F12_S : MYRISCVXGPRReg<12, "f12", ["fa2" ]>, DwarfRegNum<[44]>;
  def F13_S : MYRISCVXGPRReg<13, "f13", ["fa3" ]>, DwarfRegNum<[45]>;
  def F14_S : MYRISCVXGPRReg<14, "f14", ["fa4" ]>, DwarfRegNum<[46]>;
  def F15_S : MYRISCVXGPRReg<15, "f15", ["fa5" ]>, DwarfRegNum<[47]>;
  def F16_S : MYRISCVXGPRReg<16, "f16", ["fa6" ]>, DwarfRegNum<[48]>;
  def F17_S : MYRISCVXGPRReg<17, "f17", ["fa7" ]>, DwarfRegNum<[49]>;
  def F18_S : MYRISCVXGPRReg<18, "f18", ["fs2" ]>, DwarfRegNum<[50]>;
  def F19_S : MYRISCVXGPRReg<19, "f19", ["fs3" ]>, DwarfRegNum<[51]>;
  def F20_S : MYRISCVXGPRReg<20, "f20", ["fs4" ]>, DwarfRegNum<[52]>;
  def F21_S : MYRISCVXGPRReg<21, "f21", ["fs5" ]>, DwarfRegNum<[53]>;
  def F22_S : MYRISCVXGPRReg<22, "f22", ["fs6" ]>, DwarfRegNum<[54]>;
  def F23_S : MYRISCVXGPRReg<23, "f23", ["fs7" ]>, DwarfRegNum<[55]>;
  def F24_S : MYRISCVXGPRReg<24, "f24", ["fs8" ]>, DwarfRegNum<[56]>;
  def F25_S : MYRISCVXGPRReg<25, "f25", ["fs9" ]>, DwarfRegNum<[57]>;
  def F26_S : MYRISCVXGPRReg<26, "f26", ["fs10"]>, DwarfRegNum<[58]>;
  def F27_S : MYRISCVXGPRReg<27, "f27", ["fs11"]>, DwarfRegNum<[59]>;
  def F28_S : MYRISCVXGPRReg<28, "f28", ["ft8" ]>, DwarfRegNum<[60]>;
  def F29_S : MYRISCVXGPRReg<29, "f29", ["ft9" ]>, DwarfRegNum<[61]>;
  def F30_S : MYRISCVXGPRReg<30, "f30", ["ft10"]>, DwarfRegNum<[62]>;
  def F31_S : MYRISCVXGPRReg<31, "f31", ["ft11"]>, DwarfRegNum<[63]>;

  def F0_D  : MYRISCVXFPR_D< 0, ["ft0" ], F0_S , [sub_32]>, DwarfRegNum<[64]>;
  def F1_D  : MYRISCVXFPR_D< 1, ["ft1" ], F1_S , [sub_32]>, DwarfRegNum<[65]>;
  def F2_D  : MYRISCVXFPR_D< 2, ["ft2" ], F2_S , [sub_32]>, DwarfRegNum<[66]>;
  def F3_D  : MYRISCVXFPR_D< 3, ["ft3" ], F3_S , [sub_32]>, DwarfRegNum<[67]>;
  def F4_D  : MYRISCVXFPR_D< 4, ["ft4" ], F4_S , [sub_32]>, DwarfRegNum<[68]>;
  def F5_D  : MYRISCVXFPR_D< 5, ["ft5" ], F5_S , [sub_32]>, DwarfRegNum<[69]>;
  def F6_D  : MYRISCVXFPR_D< 6, ["ft6" ], F6_S , [sub_32]>, DwarfRegNum<[70]>;
  def F7_D  : MYRISCVXFPR_D< 7, ["ft7" ], F7_S , [sub_32]>, DwarfRegNum<[71]>;
  def F8_D  : MYRISCVXFPR_D< 8, ["fs0" ], F8_S , [sub_32]>, DwarfRegNum<[72]>;
  def F9_D  : MYRISCVXFPR_D< 9, ["fs1" ], F9_S , [sub_32]>, DwarfRegNum<[73]>;
  def F10_D : MYRISCVXFPR_D<10, ["fa0" ], F10_S, [sub_32]>, DwarfRegNum<[74]>;
  def F11_D : MYRISCVXFPR_D<11, ["fa1" ], F11_S, [sub_32]>, DwarfRegNum<[75]>;
  def F12_D : MYRISCVXFPR_D<12, ["fa2" ], F12_S, [sub_32]>, DwarfRegNum<[76]>;
  def F13_D : MYRISCVXFPR_D<13, ["fa3" ], F13_S, [sub_32]>, DwarfRegNum<[77]>;
  def F14_D : MYRISCVXFPR_D<14, ["fa4" ], F14_S, [sub_32]>, DwarfRegNum<[78]>;
  def F15_D : MYRISCVXFPR_D<15, ["fa5" ], F15_S, [sub_32]>, DwarfRegNum<[79]>;
  def F16_D : MYRISCVXFPR_D<16, ["fa6" ], F16_S, [sub_32]>, DwarfRegNum<[80]>;
  def F17_D : MYRISCVXFPR_D<17, ["fa7" ], F17_S, [sub_32]>, DwarfRegNum<[81]>;
  def F18_D : MYRISCVXFPR_D<18, ["fs2" ], F18_S, [sub_32]>, DwarfRegNum<[82]>;
  def F19_D : MYRISCVXFPR_D<19, ["fs3" ], F19_S, [sub_32]>, DwarfRegNum<[83]>;
  def F20_D : MYRISCVXFPR_D<20, ["fs4" ], F20_S, [sub_32]>, DwarfRegNum<[84]>;
  def F21_D : MYRISCVXFPR_D<21, ["fs5" ], F21_S, [sub_32]>, DwarfRegNum<[85]>;
  def F22_D : MYRISCVXFPR_D<22, ["fs6" ], F22_S, [sub_32]>, DwarfRegNum<[86]>;
  def F23_D : MYRISCVXFPR_D<23, ["fs7" ], F23_S, [sub_32]>, DwarfRegNum<[87]>;
  def F24_D : MYRISCVXFPR_D<24, ["fs8" ], F24_S, [sub_32]>, DwarfRegNum<[88]>;
  def F25_D : MYRISCVXFPR_D<25, ["fs9" ], F25_S, [sub_32]>, DwarfRegNum<[89]>;
  def F26_D : MYRISCVXFPR_D<26, ["fs10"], F26_S, [sub_32]>, DwarfRegNum<[90]>;
  def F27_D : MYRISCVXFPR_D<27, ["fs11"], F27_S, [sub_32]>, DwarfRegNum<[91]>;
  def F28_D : MYRISCVXFPR_D<28, ["ft8" ], F28_S, [sub_32]>, DwarfRegNum<[92]>;
  def F29_D : MYRISCVXFPR_D<29, ["ft9" ], F29_S, [sub_32]>, DwarfRegNum<[93]>;
  def F30_D : MYRISCVXFPR_D<30, ["ft10"], F30_S, [sub_32]>, DwarfRegNum<[94]>;
  def F31_D : MYRISCVXFPR_D<31, ["ft11"], F31_S, [sub_32]>, DwarfRegNum<[95]>;
}


//===----------------------------------------------------------------------===//
//@Register Classes
//===----------------------------------------------------------------------===//

def XLenVT : ValueTypeByHwMode<[RV32, RV64, DefaultMode],
                               [i32,  i64,  i32]>;

def GPR : RegisterClass<"MYRISCVX", [XLenVT], 32, (add
  // Reserved
  ZERO,
  // Return Values and Arguments
  A0, A1, A2, A3, A4, A5, A6, A7,
  // Not preserved across procedure calls
  T0, T1, T2, T3, T4, T5, T6,
  // Callee save
  FP, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11,
  // Reserved
  RA, SP, GP, TP
  )> {
    let RegInfos = RegInfoByHwMode<
      [RV32,              RV64,              DefaultMode],
      [RegInfo<32,32,32>, RegInfo<64,64,64>, RegInfo<32,32,32>]>;
}


// Single Floating Point Register Class
def FPR_S : RegisterClass<"MYRISCVX", [f32], 32, (add
  F0_S , F1_S,  F2_S,  F3_S,  F4_S,  F5_S,  F6_S,  F7_S,
  F8_S , F9_S,  F10_S, F11_S, F12_S, F13_S, F14_S, F15_S,
  F16_S, F17_S, F18_S, F19_S, F20_S, F21_S, F22_S, F23_S,
  F24_S, F25_S, F26_S, F27_S, F28_S, F29_S, F30_S, F31_S
  )>;

// Double Floating Point Register Class
def FPR_D : RegisterClass<"MYRISCVX", [f64], 32, (add
  F0_D , F1_D,  F2_D,  F3_D,  F4_D,  F5_D,  F6_D,  F7_D,
  F8_D , F9_D,  F10_D, F11_D, F12_D, F13_D, F14_D, F15_D,
  F16_D, F17_D, F18_D, F19_D, F20_D, F21_D, F22_D, F23_D,
  F24_D, F25_D, F26_D, F27_D, F28_D, F29_D, F30_D, F31_D
  )>;
