//===- MYRISCVXDisassembler.cpp - Disassembler for MYRISCVX -------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file is part of the MYRISCVX Disassembler.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXRegisterInfo.h"
#include "MYRISCVXSubtarget.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "MYRISCVX-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {

/// MYRISCVXDisassemblerBase - a disasembler class for MYRISCVX.
class MYRISCVXDisassemblerBase : public MCDisassembler {
public:
  /// Constructor     - Initializes the disassembler.
  ///
  MYRISCVXDisassemblerBase(const MCSubtargetInfo &STI, MCContext &Ctx,
                           bool bigEndian) :
    MCDisassembler(STI, Ctx),
    IsBigEndian(bigEndian) {}

  virtual ~MYRISCVXDisassemblerBase() {}

protected:
  bool IsBigEndian;
};

/// MYRISCVXDisassembler - a disasembler class for MYRISCVX32.
class MYRISCVXDisassembler : public MYRISCVXDisassemblerBase {
public:
  /// Constructor     - Initializes the disassembler.
  ///
  MYRISCVXDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx, bool bigEndian)
      : MYRISCVXDisassemblerBase(STI, Ctx, bigEndian) {
  }

  /// getInstruction - See MCDisassembler.
  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &VStream,
                              raw_ostream &CStream) const override;
};

} // end anonymous namespace

// Decoder tables for GPR register
static const unsigned CPURegsTable[] = {

  MYRISCVX::ZERO, MYRISCVX::RA, MYRISCVX::SP, MYRISCVX::GP,
  MYRISCVX::TP, MYRISCVX::T0, MYRISCVX::T1, MYRISCVX::T2,
  MYRISCVX::S0, MYRISCVX::S1,
  MYRISCVX::A0, MYRISCVX::A1, MYRISCVX::A2, MYRISCVX::A3,
  MYRISCVX::A4, MYRISCVX::A5, MYRISCVX::A6, MYRISCVX::A7,
  MYRISCVX::S0, MYRISCVX::S1, MYRISCVX::S2, MYRISCVX::S3,
  MYRISCVX::S4, MYRISCVX::S5, MYRISCVX::S6, MYRISCVX::S7,
  MYRISCVX::S8, MYRISCVX::S9, MYRISCVX::S10, MYRISCVX::S11,
  MYRISCVX::T3, MYRISCVX::T4, MYRISCVX::T5, MYRISCVX::T6
};

// // Decoder tables for co-processor 0 register
// static const unsigned C0RegsTable[] = {
//   // MYRISCVX::PC, MYRISCVX::EPC
// };

static DecodeStatus DecodeCPURegsRegisterClass(MCInst &Inst,
                                               unsigned RegNo,
                                               uint64_t Address,
                                               const void *Decoder);
static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst,
                                           unsigned RegNo,
                                           uint64_t Address,
                                           const void *Decoder);
// static DecodeStatus DecodeSRRegisterClass(MCInst &Inst,
//                                                unsigned RegNo,
//                                                uint64_t Address,
//                                                const void *Decoder);
// static DecodeStatus DecodeC0RegsRegisterClass(MCInst &Inst,
//                                               unsigned RegNo,
//                                               uint64_t Address,
//                                               const void *Decoder);
static DecodeStatus DecodeBranch12Target(MCInst &Inst,
                                       unsigned Insn,
                                       uint64_t Address,
                                       const void *Decoder);
static DecodeStatus DecodeBranch20Target(MCInst &Inst,
                                       unsigned Insn,
                                       uint64_t Address,
                                       const void *Decoder);
static DecodeStatus DecodeJumpTarget(MCInst &Inst,
                                     unsigned Insn,
                                     uint64_t Address,
                                     const void *Decoder);
static DecodeStatus DecodeJumpFR(MCInst &Inst,
                                 unsigned Insn,
                                 uint64_t Address,
                                 const void *Decoder);

static DecodeStatus DecodeStore(MCInst &Inst,
                                unsigned Insn,
                                uint64_t Address,
                                const void *Decoder);
static DecodeStatus DecodeLoad (MCInst &Inst,
                                unsigned Insn,
                                uint64_t Address,
                                const void *Decoder);

static DecodeStatus DecodeSimm12(MCInst &Inst, unsigned Insn, uint64_t Address, const void *Decoder);
static DecodeStatus DecodeSimm20 (MCInst &Inst, unsigned Insn, uint64_t Address, const void *Decoder);

namespace llvm {
extern Target TheMYRISCVX32Target;
}

static MCDisassembler *createMYRISCVXDisassembler(
                       const Target &T,
                       const MCSubtargetInfo &STI,
                       MCContext &Ctx) {
  return new MYRISCVXDisassembler(STI, Ctx, true);
}

static MCDisassembler *createMYRISCVXelDisassembler(
                       const Target &T,
                       const MCSubtargetInfo &STI,
                       MCContext &Ctx) {
  return new MYRISCVXDisassembler(STI, Ctx, false);
}

extern "C" void LLVMInitializeMYRISCVXDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(TheMYRISCVX32Target,
                                         createMYRISCVXelDisassembler);
}

#include "MYRISCVXGenDisassemblerTables.inc"

/// Read four bytes from the ArrayRef and return 32 bit word sorted
/// according to the given endianess
static DecodeStatus readInstruction32(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn,
                                      bool IsBigEndian) {
  // We want to read exactly 4 Bytes of data.
  if (Bytes.size() < 4) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  if (IsBigEndian) {
    // Encoded as a big-endian 32-bit word in the stream.
    Insn = (Bytes[3] <<  0) |
           (Bytes[2] <<  8) |
           (Bytes[1] << 16) |
           (Bytes[0] << 24);
  }
  else {
    // Encoded as a small-endian 32-bit word in the stream.
    Insn = (Bytes[0] <<  0) |
           (Bytes[1] <<  8) |
           (Bytes[2] << 16) |
           (Bytes[3] << 24);
  }

  return MCDisassembler::Success;
}

DecodeStatus
MYRISCVXDisassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                              ArrayRef<uint8_t> Bytes,
                                              uint64_t Address,
                                              raw_ostream &VStream,
                                              raw_ostream &CStream) const {
  uint32_t Insn;

  DecodeStatus Result;

  Result = readInstruction32(Bytes, Address, Size, Insn, IsBigEndian);

  if (Result == MCDisassembler::Fail)
    return MCDisassembler::Fail;

  // Calling the auto-generated decoder function.
  Result = decodeInstruction(DecoderTableMYRISCVX32, Instr, Insn, Address,
                             this, STI);
  if (Result != MCDisassembler::Fail) {
    Size = 4;
    return Result;
  }

  return MCDisassembler::Fail;
}

static DecodeStatus DecodeCPURegsRegisterClass(MCInst &Inst,
                                               unsigned RegNo,
                                               uint64_t Address,
                                               const void *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;

  Inst.addOperand(MCOperand::createReg(CPURegsTable[RegNo]));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst,
                                           unsigned RegNo,
                                           uint64_t Address,
                                           const void *Decoder) {
  return DecodeCPURegsRegisterClass(Inst, RegNo, Address, Decoder);
}

// static DecodeStatus DecodeSRRegisterClass(MCInst &Inst,
//                                                unsigned RegNo,
//                                                uint64_t Address,
//                                                const void *Decoder) {
//   return DecodeCPURegsRegisterClass(Inst, RegNo, Address, Decoder);
// }

// static DecodeStatus DecodeC0RegsRegisterClass(MCInst &Inst,
//                                               unsigned RegNo,
//                                               uint64_t Address,
//                                               const void *Decoder) {
//   if (RegNo > 1)
//     return MCDisassembler::Fail;
//
//   Inst.addOperand(MCOperand::createReg(C0RegsTable[RegNo]));
//   return MCDisassembler::Success;
// }

// @DecodeStore {
static DecodeStatus DecodeStore(MCInst &Inst,
                                unsigned Insn,
                                uint64_t Address,
                                const void *Decoder) {
  // @DecodeStore body {
  int Offset = SignExtend32<12>((fieldFromInstruction(Insn,  25, 7) << 5) |
                                (fieldFromInstruction(Insn,   7, 5)));
  int Reg  = (int)fieldFromInstruction(Insn, 20, 5);
  int Base = (int)fieldFromInstruction(Insn, 15, 5);

  Inst.addOperand(MCOperand::createReg(CPURegsTable[Base]));
  Inst.addOperand(MCOperand::createReg(CPURegsTable[Reg]));
  Inst.addOperand(MCOperand::createImm(Offset));

  return MCDisassembler::Success;
}


// @DecodeLoad {
static DecodeStatus DecodeLoad (MCInst &Inst,
                                unsigned Insn,
                                uint64_t Address,
                                const void *Decoder) {
  // @DecodeLoad body {
  int Offset = SignExtend32<12>((Insn >> 20) & 0x0fff);
  int Dest = (int)fieldFromInstruction(Insn,  7, 5);
  int Base = (int)fieldFromInstruction(Insn, 15, 5);

  Inst.addOperand(MCOperand::createReg(CPURegsTable[Dest]));
  Inst.addOperand(MCOperand::createReg(CPURegsTable[Base]));
  Inst.addOperand(MCOperand::createImm(Offset));

  return MCDisassembler::Success;
}


static DecodeStatus DecodeBranch12Target(MCInst &Inst,
                                       unsigned Insn,
                                       uint64_t Address,
                                       const void *Decoder) {
  int BranchOffset = fieldFromInstruction(Insn, 0, 12);
  if (BranchOffset > 0x8ff)
  	BranchOffset = -1*(0x1000 - BranchOffset);
  Inst.addOperand(MCOperand::createImm(BranchOffset));
  return MCDisassembler::Success;
}

/* CBranch instruction define $ra and then imm24; The printOperand() print
operand 1 (operand 0 is $ra and operand 1 is imm24), so we Create register
operand first and create imm24 next, as follows,

// MYRISCVXInstrInfo.td
class CBranch<bits<8> op, string instr_asm, RegisterClass RC,
                   list<Register> UseRegs>:
  FJ<op, (outs), (ins RC:$ra, brtarget:$addr),
             !strconcat(instr_asm, "\t$addr"),
             [(brcond RC:$ra, bb:$addr)], IIBranch> {

// MYRISCVXAsmWriter.inc
void MYRISCVXInstPrinter::printInstruction(const MCInst *MI, raw_ostream &O) {
...
  case 3:
    // CMP, JEQ, JGE, JGT, JLE, JLT, JNE
    printOperand(MI, 1, O);
    break;
*/
static DecodeStatus DecodeBranch20Target(MCInst &Inst,
                                       unsigned Insn,
                                       uint64_t Address,
                                       const void *Decoder) {
  int BranchOffset = fieldFromInstruction(Insn, 0, 20);
  if (BranchOffset > 0x8ffff)
  	BranchOffset = -1*(0x100000 - BranchOffset);
  Inst.addOperand(MCOperand::createReg(MYRISCVX::SW));
  Inst.addOperand(MCOperand::createImm(BranchOffset));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeJumpTarget(MCInst &Inst,
                                     unsigned Insn,
                                     uint64_t Address,
                                     const void *Decoder) {

  unsigned JumpOffset = fieldFromInstruction(Insn, 0, 24);
  Inst.addOperand(MCOperand::createImm(JumpOffset));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeJumpFR(MCInst &Inst,
                                     unsigned Insn,
                                     uint64_t Address,
                                     const void *Decoder) {
  int Reg_a = (int)fieldFromInstruction(Insn, 20, 4);
  Inst.addOperand(MCOperand::createReg(CPURegsTable[Reg_a]));
  // exapin in http://jonathan2251.github.io/lbd/llvmstructure.html#jr-note
  if (CPURegsTable[Reg_a] == MYRISCVX::RA)
    Inst.setOpcode(MYRISCVX::JALR);
  else
    Inst.setOpcode(MYRISCVX::JALR);
  return MCDisassembler::Success;
}

static DecodeStatus DecodeSimm12(MCInst &Inst,
                                 unsigned Insn,
                                 uint64_t Address,
                                 const void *Decoder) {
  Inst.addOperand(MCOperand::createImm(SignExtend32<12>(Insn)));
  return MCDisassembler::Success;
}


static DecodeStatus DecodeSimm20(MCInst &Inst,
                                 unsigned Insn,
                                 uint64_t Address,
                                 const void *Decoder) {
  Inst.addOperand(MCOperand::createImm(SignExtend32<20>(Insn)));
  return MCDisassembler::Success;
}
