//===- MYRISCVXInstrInfo.td - Target Description for MYRISCVX Target -*- tablegen -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MYRISCVX implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Instruction definition
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// MYRISCVX Instructions
//===----------------------------------------------------------------------===//

// Floating Point Load Store
defm FLW    : LoadM32 <0b0000111, 0b010, "flw", FPR_S, load_a >;
defm FSW    : StoreM32<0b0100111, 0b010, "fsw", FPR_S, truncstoref32_a>;

defm FLD    : LoadM32 <0b0000111, 0b011, "fld", FPR_D, load_a >;
defm FSD    : StoreM32<0b0100111, 0b011, "fsd", FPR_D, truncstoref64_a>;

def FADD_S  : ArithLogicR<0b1010011, 0b000, 0b0000000, "fadd.s", fadd, FPR_S>;
def FSUB_S  : ArithLogicR<0b1010011, 0b000, 0b0000100, "fsub.s", fsub, FPR_S>;
def FMUL_S  : ArithLogicR<0b1010011, 0b000, 0b0001000, "fmul.s", fmul, FPR_S>;
def FDIV_S  : ArithLogicR<0b1010011, 0b000, 0b0001100, "fdiv.s", fdiv, FPR_S>;

def FADD_D  : ArithLogicR<0b1010011, 0b000, 0b0000001, "fadd.d", fadd, FPR_D>;
def FSUB_D  : ArithLogicR<0b1010011, 0b000, 0b0000101, "fsub.d", fsub, FPR_D>;
def FMUL_D  : ArithLogicR<0b1010011, 0b000, 0b0001001, "fmul.d", fmul, FPR_D>;
def FDIV_D  : ArithLogicR<0b1010011, 0b000, 0b0001101, "fdiv.d", fdiv, FPR_D>;

//===----------------------------------------------------------------------===//
//  Arbitrary patterns that map to one or more instructions
//===----------------------------------------------------------------------===//

def : Pat<(f32 (load GPR:$rs1))                          , (FLW GPR:$rs1, 0)           >;
def : Pat<(f32 (load addr_fi:$rs1))                      , (FLW addr_fi:$rs1, 0)       >;
def : Pat<(f32 (load (add GPR:$rs1, simm12:$simm12)))    , (FLW GPR:$rs1, $simm12)     >;
def : Pat<(f32 (load (add addr_fi:$rs1, simm12:$simm12))), (FLW addr_fi:$rs1, $simm12) >;

def : Pat<(store FPR_S:$rs2, GPR:$rs1)                          , (FSW FPR_S:$rs2, GPR:$rs1, 0)                         >;
def : Pat<(store FPR_S:$rs2, addr_fi:$rs1)                      , (FSW FPR_S:$rs2, addr_fi:$rs1, 0)                     >;
def : Pat<(store FPR_S:$rs2, (add GPR:$rs1, simm12:$simm12))    , (FSW FPR_S:$rs2, GPR:$rs1, simm12:$simm12)            >;
def : Pat<(store FPR_S:$rs2, (add addr_fi:$rs1, simm12:$simm12)), (FSW FPR_S:$rs2, addr_fi:$rs1, simm12:$simm12)        >;

def : Pat<(f64 (load GPR:$rs1))                          , (FLD GPR:$rs1, 0)           >;
def : Pat<(f64 (load addr_fi:$rs1))                      , (FLD addr_fi:$rs1, 0)       >;
def : Pat<(f64 (load (add GPR:$rs1, simm12:$simm12)))    , (FLD GPR:$rs1, $simm12)     >;
def : Pat<(f64 (load (add addr_fi:$rs1, simm12:$simm12))), (FLD addr_fi:$rs1, $simm12) >;

def : Pat<(store FPR_D:$rs2, GPR:$rs1)                          , (FSD FPR_D:$rs2, GPR:$rs1, 0)                         >;
def : Pat<(store FPR_D:$rs2, addr_fi:$rs1)                      , (FSD FPR_D:$rs2, addr_fi:$rs1, 0)                     >;
def : Pat<(store FPR_D:$rs2, (add GPR:$rs1, simm12:$simm12))    , (FSD FPR_D:$rs2, GPR:$rs1, simm12:$simm12)            >;
def : Pat<(store FPR_D:$rs2, (add addr_fi:$rs1, simm12:$simm12)), (FSD FPR_D:$rs2, addr_fi:$rs1, simm12:$simm12)        >;
