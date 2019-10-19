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

#include "MYRISCVX.h"

#include "MYRISCVXRegisterInfo.h"
#include "MYRISCVXSubtarget.h"
#include "MCTargetDesc/MYRISCVXMCTargetDesc.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include <cassert>
#include <cstdint>

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

static DecodeStatus DecodeCPURegsRegisterClass(MCInst &Inst,
                                               unsigned RegNo,
                                               uint64_t Address,
                                               const void *Decoder);
static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst,
                                           unsigned RegNo,
                                           uint64_t Address,
                                           const void *Decoder);
template <unsigned N>
static DecodeStatus decodeSImmOperand(MCInst &Inst,
                                      uint64_t Imm,
                                      int64_t Address,
                                      const void *Decoder);

static DecodeStatus DecodeJumpTarget(MCInst &Inst,
                                     unsigned Insn,
                                     uint64_t Address,
                                     const void *Decoder);

static MCDisassembler *createMYRISCVX32Disassembler(
    const Target &T,
    const MCSubtargetInfo &STI,
    MCContext &Ctx) {
  // Little Endian
  return new MYRISCVXDisassembler(STI, Ctx, false);
}

static MCDisassembler *createMYRISCVX64Disassembler(
    const Target &T,
    const MCSubtargetInfo &STI,
    MCContext &Ctx) {
  return new MYRISCVXDisassembler(STI, Ctx, false);
}

extern "C" void LLVMInitializeMYRISCVXDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheMYRISCVX32Target(),
                                         createMYRISCVX32Disassembler);
  TargetRegistry::RegisterMCDisassembler(getTheMYRISCVX32Target(),
                                         createMYRISCVX64Disassembler);
}

#include "MYRISCVXGenDisassemblerTables.inc"

DecodeStatus
MYRISCVXDisassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                     ArrayRef<uint8_t> Bytes,
                                     uint64_t Address,
                                     raw_ostream &VStream,
                                     raw_ostream &CStream) const {
  uint32_t Insn;

  DecodeStatus Result;

  Insn = support::endian::read32le(Bytes.data());

  // Calling the auto-generated decoder function.
  Result = decodeInstruction(DecoderTableMYRISCVX32, Instr, Insn, Address,
                             this, STI);
  if (Result != MCDisassembler::Fail) {
    Size = 4;
    return Result;
  }

  return MCDisassembler::Fail;
}


const unsigned int GPRDecodeTable [] = {
  MYRISCVX::ZERO, MYRISCVX::RA, MYRISCVX::SP,  MYRISCVX::GP,
  MYRISCVX::TP,   MYRISCVX::T6, MYRISCVX::T1,  MYRISCVX::T2,
  MYRISCVX::FP,   MYRISCVX::S1, MYRISCVX::A0,  MYRISCVX::A1,
  MYRISCVX::A2,   MYRISCVX::A3, MYRISCVX::A4,  MYRISCVX::A5,
  MYRISCVX::A6,   MYRISCVX::A7, MYRISCVX::S2,  MYRISCVX::S3,
  MYRISCVX::S4,   MYRISCVX::S5, MYRISCVX::S6,  MYRISCVX::S7,
  MYRISCVX::S8,   MYRISCVX::S9, MYRISCVX::S10, MYRISCVX::S11,
  MYRISCVX::T0,   MYRISCVX::T3, MYRISCVX::T4,  MYRISCVX::T5 };

static DecodeStatus DecodeCPURegsRegisterClass(MCInst &Inst,
                                               unsigned RegNo,
                                               uint64_t Address,
                                               const void *Decoder) {
  if (RegNo > sizeof(GPRDecodeTable))
    return MCDisassembler::Fail;

  unsigned int Reg = GPRDecodeTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst,
                                           unsigned RegNo,
                                           uint64_t Address,
                                           const void *Decoder) {
  return DecodeCPURegsRegisterClass(Inst, RegNo, Address, Decoder);
}


template <unsigned N>
static DecodeStatus decodeSImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address, const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  // Sign-extend the number in the bottom N bits of Imm
  Inst.addOperand(MCOperand::createImm(SignExtend64<N>(Imm)));
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
