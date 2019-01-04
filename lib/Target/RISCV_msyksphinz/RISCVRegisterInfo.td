let Namespace = "MYRISCVX" in {

// We have banks of 32 registers each.
class MYRISCVXReg<bits<16> Enc, string n> : Register<n> {
  let HWEncoding = Enc;
}

// RISCV CPU Registers
class RISCVGPRReg<bits<16> Enc, string n> : MYRISCVXReg<Enc, n>;

// Co-processor 0 Registers
class RISCVFPReg<bits<16> Enc, string n> : MYRISCVXReg<Enc, n>;

def ZERO : RISCVGPRReg<0,  "x0">,    DwarfRegNum<[0]>;
def RA   : RISCVGPRReg<1,  "x1">,    DwarfRegNum<[1]>;
def SP   : RISCVGPRReg<2,  "x2">,    DwarfRegNum<[2]>;
def GP   : RISCVGPRReg<3,  "x3">,    DwarfRegNum<[3]>;
def TP   : RISCVGPRReg<4,  "x4">,    DwarfRegNum<[4]>;
def T0   : RISCVGPRReg<5,  "x5">,    DwarfRegNum<[5]>;
def T1   : RISCVGPRReg<6,  "x6">,    DwarfRegNum<[6]>;
def T2   : RISCVGPRReg<7,  "x7">,    DwarfRegNum<[7]>;
def S0   : RISCVGPRReg<8,  "x8">,    DwarfRegNum<[8]>;
def S1   : RISCVGPRReg<9,  "x9">,    DwarfRegNum<[9]>;

def A0   : RISCVGPRReg<10, "x10">,   DwarfRegNum<[10]>;
def A1   : RISCVGPRReg<11, "x11">,   DwarfRegNum<[11]>;
def A2   : RISCVGPRReg<12, "x12">,   DwarfRegNum<[12]>;
def A3   : RISCVGPRReg<13, "x13">,   DwarfRegNum<[13]>;
def A4   : RISCVGPRReg<14, "x14">,   DwarfRegNum<[10]>;
def A5   : RISCVGPRReg<15, "x15">,   DwarfRegNum<[11]>;
def A6   : RISCVGPRReg<16, "x16">,   DwarfRegNum<[12]>;
def A7   : RISCVGPRReg<17, "x17">,   DwarfRegNum<[13]>;

def S2   : RISCVGPRReg<18, "x18">,   DwarfRegNum<[18]>;
def S3   : RISCVGPRReg<19, "x19">,   DwarfRegNum<[19]>;
def S4   : RISCVGPRReg<20, "x20">,   DwarfRegNum<[20]>;
def S5   : RISCVGPRReg<21, "x21">,   DwarfRegNum<[21]>;
def S6   : RISCVGPRReg<22, "x22">,   DwarfRegNum<[22]>;
def S7   : RISCVGPRReg<23, "x23">,   DwarfRegNum<[23]>;
def S8   : RISCVGPRReg<24, "x24">,   DwarfRegNum<[24]>;
def S9   : RISCVGPRReg<25, "x25">,   DwarfRegNum<[25]>;
def S10  : RISCVGPRReg<26, "x26">,   DwarfRegNum<[26]>;
def S11  : RISCVGPRReg<27, "x27">,   DwarfRegNum<[27]>;

def T3   : RISCVGPRReg<28, "x28">,   DwarfRegNum<[28]>;
def T4   : RISCVGPRReg<29, "x29">,   DwarfRegNum<[29]>;
def T5   : RISCVGPRReg<30, "x30">,   DwarfRegNum<[30]>;
def T6   : RISCVGPRReg<31, "x31">,   DwarfRegNum<[31]>;

}


//===----------------------------------------------------------------------===//
//@Register Classes
//===----------------------------------------------------------------------===//

def RISCVRegs : RegisterClass<"MYRISCVX", [i32], 32, (add
  // Reserved
  ZERO,
  // Return Values and Arguments
  A0, A1, A2, A3, A4, A5, A6, A7,
  // Not preserved across procedure calls
  T0, T1, T2, T3, T4, T5, T6,
  // Callee save
  S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11,
  // Reserved
  RA, SP, GP, TP
  )>;
