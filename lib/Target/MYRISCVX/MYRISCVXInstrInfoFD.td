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
// Instructions specific format
//===----------------------------------------------------------------------===//

// Floating-Point FMA instructions with 3 register operands.
class FPMultAdd<bits<7> opcode, bits<2> fmt, bits<3> rm,
                string instr_asm,
                RegisterClass RC> :
  MYRISCVX_FMA<opcode, fmt, rm, (outs RC:$rd), (ins RC:$rs1, RC:$rs2, RC:$rs3),
               !strconcat(instr_asm, "\t$rd, $rs1, $rs2, $rs3"),
               [], IIAlu> {
    let isReMaterializable = 1;
}

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

//
// Single Floating Point Operations
//
def FMADD_S  : FPMultAdd<0b1010011, 0b00, 0b000, "fmadd.s",  FPR_S>;
def FMSUB_S  : FPMultAdd<0b1000111, 0b00, 0b000, "fmsub.s",  FPR_S>;
def FNMSUB_S : FPMultAdd<0b1001011, 0b00, 0b000, "fnmsub.s", FPR_S>;
def FNMADD_S : FPMultAdd<0b1001111, 0b00, 0b000, "fnmadd.s", FPR_S>;

// Floating-Point FMA pattern registration
def : Pat<(fma FPR_S:$rs1, FPR_S:$rs2, FPR_S:$rs3),               (FMADD_S  $rs1, $rs2, $rs3)                  >;
def : Pat<(fma FPR_S:$rs1, FPR_S:$rs2, (fneg FPR_S:$rs3)),        (FMSUB_S  FPR_S:$rs1, FPR_S:$rs2, FPR_S:$rs3)>;
def : Pat<(fma (fneg FPR_S:$rs1), FPR_S:$rs2, FPR_S:$rs3),        (FNMSUB_S FPR_S:$rs1, FPR_S:$rs2, FPR_S:$rs3)>;
def : Pat<(fma (fneg FPR_S:$rs1), FPR_S:$rs2, (fneg FPR_S:$rs3)), (FNMADD_S FPR_S:$rs1, FPR_S:$rs2, FPR_S:$rs3)>;

def FADD_S   : ArithLogicR<0b1010011, 0b000, 0b0000000, "fadd.s", fadd, FPR_S>;
def FSUB_S   : ArithLogicR<0b1010011, 0b000, 0b0000100, "fsub.s", fsub, FPR_S>;
def FMUL_S   : ArithLogicR<0b1010011, 0b000, 0b0001000, "fmul.s", fmul, FPR_S>;
def FDIV_S   : ArithLogicR<0b1010011, 0b000, 0b0001100, "fdiv.s", fdiv, FPR_S>;

//
// Double Floating Point Operations
//
def FMADD_D  : FPMultAdd<0b1010011, 0b01, 0b000, "fmadd.d",  FPR_D>;
def FMSUB_D  : FPMultAdd<0b1000111, 0b01, 0b000, "fmsub.d",  FPR_D>;
def FNMSUB_D : FPMultAdd<0b1001011, 0b01, 0b000, "fnmsub.d", FPR_D>;
def FNMADD_D : FPMultAdd<0b1001111, 0b01, 0b000, "fnmadd.d", FPR_D>;

// Floating-Point FMA pattern registration
def : Pat<(fma FPR_D:$rs1, FPR_D:$rs2, FPR_D:$rs3),               (FMADD_D  $rs1, $rs2, $rs3)                  >;
def : Pat<(fma FPR_D:$rs1, FPR_D:$rs2, (fneg FPR_D:$rs3)),        (FMSUB_D  FPR_D:$rs1, FPR_D:$rs2, FPR_D:$rs3)>;
def : Pat<(fma (fneg FPR_D:$rs1), FPR_D:$rs2, FPR_D:$rs3),        (FNMSUB_D FPR_D:$rs1, FPR_D:$rs2, FPR_D:$rs3)>;
def : Pat<(fma (fneg FPR_D:$rs1), FPR_D:$rs2, (fneg FPR_D:$rs3)), (FNMADD_D FPR_D:$rs1, FPR_D:$rs2, FPR_D:$rs3)>;

def FADD_D   : ArithLogicR<0b1010011, 0b000, 0b0000001, "fadd.d", fadd, FPR_D>;
def FSUB_D   : ArithLogicR<0b1010011, 0b000, 0b0000101, "fsub.d", fsub, FPR_D>;
def FMUL_D   : ArithLogicR<0b1010011, 0b000, 0b0001001, "fmul.d", fmul, FPR_D>;
def FDIV_D   : ArithLogicR<0b1010011, 0b000, 0b0001101, "fdiv.d", fdiv, FPR_D>;


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
