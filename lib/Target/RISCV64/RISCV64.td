//===-- RISCV64.td - Describe the RISCV64 Target Machine ---------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This is the top level entry point for the RISCV64 target.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

// The overall idea of the PredicateControl class is to chop the Predicates list
// into subsets that are usually overridden independently. This allows
// subclasses to partially override the predicates of their superclasses without
// having to re-add all the existing predicates.
class PredicateControl {
  // Predicates for the encoding scheme in use such as HasStdEnc
  list<Predicate> EncodingPredicates = [];
  // Predicates for the GPR size such as IsGP64bit
  list<Predicate> GPRPredicates = [];
  // Predicates for the PTR size such as IsPTR64bit
  list<Predicate> PTRPredicates = [];
  // Predicates for the FGR size and layout such as IsFP64bit
  list<Predicate> FGRPredicates = [];
  // Predicates for the instruction group membership such as ISA's.
  list<Predicate> InsnPredicates = [];
  // Predicate for the ASE that an instruction belongs to.
  list<Predicate> ASEPredicate = [];
  // Predicate for marking the instruction as usable in hard-float mode only.
  list<Predicate> HardFloatPredicate = [];
  // Predicates for anything else
  list<Predicate> AdditionalPredicates = [];
  list<Predicate> Predicates = !listconcat(EncodingPredicates,
                                           GPRPredicates,
                                           PTRPredicates,
                                           FGRPredicates,
                                           InsnPredicates,
                                           HardFloatPredicate,
                                           ASEPredicate,
                                           AdditionalPredicates);
}

// Like Requires<> but for the AdditionalPredicates list
class AdditionalRequires<list<Predicate> preds> {
  list<Predicate> AdditionalPredicates = preds;
}

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "RISCV64RegisterInfo.td"
include "RISCV64Schedule.td"
include "RISCV64InstrInfo.td"
include "RISCV64CallingConv.td"
include "RISCV64RegisterBanks.td"

// Avoid forward declaration issues.
include "RISCV64ScheduleP5600.td"
include "RISCV64ScheduleGeneric.td"

def RISCV64InstrInfo : InstrInfo;

//===----------------------------------------------------------------------===//
// RISCV64 Subtarget features                                                    //
//===----------------------------------------------------------------------===//

def FeatureNoABICalls  : SubtargetFeature<"noabicalls", "NoABICalls", "true",
                                "Disable SVR4-style position-independent code">;
def FeaturePTR64Bit    : SubtargetFeature<"ptr64", "IsPTR64bit", "true",
                                "Pointers are 64-bit wide">;
def FeatureGP64Bit     : SubtargetFeature<"gp64", "IsGP64bit", "true",
                                "General Purpose Registers are 64-bit wide">;
def FeatureFP64Bit     : SubtargetFeature<"fp64", "IsFP64bit", "true",
                                "Support 64-bit FP registers">;
def FeatureFPXX        : SubtargetFeature<"fpxx", "IsFPXX", "true",
                                "Support for FPXX">;
def FeatureNaN2008     : SubtargetFeature<"nan2008", "IsNaN2008bit", "true",
                                "IEEE 754-2008 NaN encoding">;
def FeatureSingleFloat : SubtargetFeature<"single-float", "IsSingleFloat",
                                "true", "Only supports single precision float">;
def FeatureSoftFloat   : SubtargetFeature<"soft-float", "IsSoftFloat", "true",
                                "Does not support floating point instructions">;
def FeatureNoOddSPReg  : SubtargetFeature<"nooddspreg", "UseOddSPReg", "false",
                              "Disable odd numbered single-precision "
                              "registers">;
def FeatureVFPU        : SubtargetFeature<"vfpu", "HasVFPU",
                                "true", "Enable vector FPU instructions">;
def FeatureRISCV641       : SubtargetFeature<"mips1", "RISCV64ArchVersion", "RISCV641",
                                "RISCV64 I ISA Support [highly experimental]">;
def FeatureRISCV642       : SubtargetFeature<"mips2", "RISCV64ArchVersion", "RISCV642",
                                "RISCV64 II ISA Support [highly experimental]",
                                [FeatureRISCV641]>;
def FeatureRISCV643_32    : SubtargetFeature<"mips3_32", "HasRISCV643_32", "true",
                                "Subset of MIPS-III that is also in MIPS32 "
                                "[highly experimental]">;
def FeatureRISCV643_32r2  : SubtargetFeature<"mips3_32r2", "HasRISCV643_32r2", "true",
                                "Subset of MIPS-III that is also in MIPS32r2 "
                                "[highly experimental]">;
def FeatureRISCV643       : SubtargetFeature<"mips3", "RISCV64ArchVersion", "RISCV643",
                                "MIPS III ISA Support [highly experimental]",
                                [FeatureRISCV642, FeatureRISCV643_32,
                                 FeatureRISCV643_32r2, FeatureGP64Bit,
                                 FeatureFP64Bit]>;
def FeatureRISCV644_32    : SubtargetFeature<"mips4_32", "HasRISCV644_32", "true",
                                "Subset of MIPS-IV that is also in MIPS32 "
                                "[highly experimental]">;
def FeatureRISCV644_32r2  : SubtargetFeature<"mips4_32r2", "HasRISCV644_32r2", "true",
                                "Subset of MIPS-IV that is also in MIPS32r2 "
                                "[highly experimental]">;
def FeatureRISCV644       : SubtargetFeature<"mips4", "RISCV64ArchVersion",
                                "RISCV644", "MIPS IV ISA Support",
                                [FeatureRISCV643, FeatureRISCV644_32,
                                 FeatureRISCV644_32r2]>;
def FeatureRISCV645_32r2  : SubtargetFeature<"mips5_32r2", "HasRISCV645_32r2", "true",
                                "Subset of MIPS-V that is also in MIPS32r2 "
                                "[highly experimental]">;
def FeatureRISCV645       : SubtargetFeature<"mips5", "RISCV64ArchVersion", "RISCV645",
                                "MIPS V ISA Support [highly experimental]",
                                [FeatureRISCV644, FeatureRISCV645_32r2]>;
def FeatureRISCV6432      : SubtargetFeature<"mips32", "RISCV64ArchVersion", "RISCV6432",
                                "RISCV6432 ISA Support",
                                [FeatureRISCV642, FeatureRISCV643_32,
                                 FeatureRISCV644_32]>;
def FeatureRISCV6432r2    : SubtargetFeature<"mips32r2", "RISCV64ArchVersion",
                                "RISCV6432r2", "RISCV6432r2 ISA Support",
                                [FeatureRISCV643_32r2, FeatureRISCV644_32r2,
                                 FeatureRISCV645_32r2, FeatureRISCV6432]>;
def FeatureRISCV6432r3    : SubtargetFeature<"mips32r3", "RISCV64ArchVersion",
                                "RISCV6432r3", "RISCV6432r3 ISA Support",
                                [FeatureRISCV6432r2]>;
def FeatureRISCV6432r5    : SubtargetFeature<"mips32r5", "RISCV64ArchVersion",
                                "RISCV6432r5", "RISCV6432r5 ISA Support",
                                [FeatureRISCV6432r3]>;
def FeatureRISCV6432r6    : SubtargetFeature<"mips32r6", "RISCV64ArchVersion",
                                "RISCV6432r6",
                                "RISCV6432r6 ISA Support [experimental]",
                                [FeatureRISCV6432r5, FeatureFP64Bit,
                                 FeatureNaN2008]>;
def FeatureRISCV6464      : SubtargetFeature<"mips64", "RISCV64ArchVersion",
                                "RISCV6464", "RISCV6464 ISA Support",
                                [FeatureRISCV645, FeatureRISCV6432]>;
def FeatureRISCV6464r2    : SubtargetFeature<"mips64r2", "RISCV64ArchVersion",
                                "RISCV6464r2", "RISCV6464r2 ISA Support",
                                [FeatureRISCV6464, FeatureRISCV6432r2]>;
def FeatureRISCV6464r3    : SubtargetFeature<"mips64r3", "RISCV64ArchVersion",
                                "RISCV6464r3", "RISCV6464r3 ISA Support",
                                [FeatureRISCV6464r2, FeatureRISCV6432r3]>;
def FeatureRISCV6464r5    : SubtargetFeature<"mips64r5", "RISCV64ArchVersion",
                                "RISCV6464r5", "RISCV6464r5 ISA Support",
                                [FeatureRISCV6464r3, FeatureRISCV6432r5]>;
def FeatureRISCV6464r6    : SubtargetFeature<"mips64r6", "RISCV64ArchVersion",
                                "RISCV6464r6",
                                "RISCV6464r6 ISA Support [experimental]",
                                [FeatureRISCV6432r6, FeatureRISCV6464r5,
                                 FeatureNaN2008]>;
def FeatureSym32       : SubtargetFeature<"sym32", "HasSym32", "true",
                                          "Symbols are 32 bit on RISCV6464">;

def FeatureRISCV6416  : SubtargetFeature<"mips16", "InRISCV6416Mode", "true",
                                      "RISCV6416 mode">;

def FeatureDSP : SubtargetFeature<"dsp", "HasDSP", "true", "RISCV64 DSP ASE">;
def FeatureDSPR2 : SubtargetFeature<"dspr2", "HasDSPR2", "true",
                                    "RISCV64 DSP-R2 ASE", [FeatureDSP]>;
def FeatureDSPR3
    : SubtargetFeature<"dspr3", "HasDSPR3", "true", "RISCV64 DSP-R3 ASE",
                       [ FeatureDSP, FeatureDSPR2 ]>;

def FeatureMSA : SubtargetFeature<"msa", "HasMSA", "true", "RISCV64 MSA ASE">;

def FeatureEVA : SubtargetFeature<"eva", "HasEVA", "true", "RISCV64 EVA ASE">;

def FeatureCRC : SubtargetFeature<"crc", "HasCRC", "true", "RISCV64 R6 CRC ASE">;

def FeatureVirt : SubtargetFeature<"virt", "HasVirt", "true",
                                   "RISCV64 Virtualization ASE">;

def FeatureGINV : SubtargetFeature<"ginv", "HasGINV", "true",
                                   "RISCV64 Global Invalidate ASE">;

def FeatureMicroRISCV64  : SubtargetFeature<"micromips", "InMicroRISCV64Mode", "true",
                                         "microRISCV64 mode">;

def FeatureCnRISCV64     : SubtargetFeature<"cnmips", "HasCnRISCV64",
                                "true", "Octeon cnMIPS Support",
                                [FeatureRISCV6464r2]>;

def FeatureUseTCCInDIV : SubtargetFeature<
                               "use-tcc-in-div",
                               "UseTCCInDIV", "false",
                               "Force the assembler to use trapping">;

def FeatureMadd4 : SubtargetFeature<"nomadd4", "DisableMadd4", "true",
                                    "Disable 4-operand madd.fmt and related instructions">;

def FeatureMT : SubtargetFeature<"mt", "HasMT", "true", "RISCV64 MT ASE">;

def FeatureLongCalls : SubtargetFeature<"long-calls", "UseLongCalls", "true",
                                        "Disable use of the jal instruction">;

def FeatureUseIndirectJumpsHazard : SubtargetFeature<"use-indirect-jump-hazard",
                                                    "UseIndirectJumpsHazard",
                                                    "true", "Use indirect jump"
                        " guards to prevent certain speculation based attacks">;
//===----------------------------------------------------------------------===//
// RISCV64 processors supported.
//===----------------------------------------------------------------------===//

def ImplP5600 : SubtargetFeature<"p5600", "ProcImpl",
                                 "RISCV64Subtarget::CPU::P5600",
                                 "The P5600 Processor", [FeatureRISCV6432r5]>;

class Proc<string Name, list<SubtargetFeature> Features>
 : ProcessorModel<Name, RISCV64GenericModel, Features>;

def : Proc<"mips1", [FeatureRISCV641]>;
def : Proc<"mips2", [FeatureRISCV642]>;
def : Proc<"mips32", [FeatureRISCV6432]>;
def : Proc<"mips32r2", [FeatureRISCV6432r2]>;
def : Proc<"mips32r3", [FeatureRISCV6432r3]>;
def : Proc<"mips32r5", [FeatureRISCV6432r5]>;
def : Proc<"mips32r6", [FeatureRISCV6432r6]>;

def : Proc<"mips3", [FeatureRISCV643]>;
def : Proc<"mips4", [FeatureRISCV644]>;
def : Proc<"mips5", [FeatureRISCV645]>;
def : Proc<"mips64", [FeatureRISCV6464]>;
def : Proc<"mips64r2", [FeatureRISCV6464r2]>;
def : Proc<"mips64r3", [FeatureRISCV6464r3]>;
def : Proc<"mips64r5", [FeatureRISCV6464r5]>;
def : Proc<"mips64r6", [FeatureRISCV6464r6]>;
def : Proc<"octeon", [FeatureRISCV6464r2, FeatureCnRISCV64]>;
def : ProcessorModel<"p5600", RISCV64P5600Model, [ImplP5600]>;

def RISCV64AsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 0;
}

def RISCV64AsmParserVariant : AsmParserVariant {
  int Variant = 0;

  // Recognize hard coded registers.
  string RegisterPrefix = "$";
}

def RISCV64 : Target {
  let InstructionSet = RISCV64InstrInfo;
  let AssemblyParsers = [RISCV64AsmParser];
  let AssemblyParserVariants = [RISCV64AsmParserVariant];
  let AllowRegisterRenaming = 1;
}
