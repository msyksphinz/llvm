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
// static DecodeStatus DecodeJumpFR(MCInst &Inst,
//                                  unsigned Insn,
//                                  uint64_t Address,
//                                  const void *Decoder);

// static DecodeStatus DecodeMem(MCInst &Inst,
//                               unsigned Insn,
//                               uint64_t Address,
//                               const void *Decoder);

static DecodeStatus DecodeSimm12(MCInst &Inst,
                                 unsigned Insn,
                                 uint64_t Address,
                                 const void *Decoder);

static DecodeStatus DecodeSimm20(MCInst &Inst,
                                 unsigned Insn,
                                 uint64_t Address,
                                 const void *Decoder);

namespace llvm {
extern Target TheMYRISCVX32Target, TheMYRISCVX64Target;
}

static MCDisassembler *createMYRISCVX32Disassembler(
    const Target &T,
    const MCSubtargetInfo &STI,
    MCContext &Ctx) {
  // Little Endian
  return new MYRISCVXDisassembler(STI, Ctx, false);
}

// static MCDisassembler *createMYRISCVX64Disassembler(
//     const Target &T,
//     const MCSubtargetInfo &STI,
//     MCContext &Ctx) {
//   return new MYRISCVXDisassembler(STI, Ctx, true);
// }

extern "C" void LLVMInitializeMYRISCVXDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheMYRISCVX32Target(),
                                         createMYRISCVX32Disassembler);
  // TargetRegistry::RegisterMCDisassembler(TheMYRISCVX64Target,
  //                                        createMYRISCVX64Disassembler);
}

#include "MYRISCVXGenDisassemblerTables.inc"

static unsigned getReg(const void *D, unsigned RC, unsigned RegNo) {
  const MYRISCVXDisassembler *Dis = static_cast<const MYRISCVXDisassembler*>(D);
  const MCRegisterInfo *RegInfo = Dis->getContext().getRegisterInfo();
  return *(RegInfo->getRegClass(RC).begin() + RegNo);
}

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

  RegNo = getReg(Decoder, MYRISCVX::GPRRegClassID, RegNo);

  Inst.addOperand(MCOperand::createReg(RegNo));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst,
                                              unsigned RegNo,
                                              uint64_t Address,
                                              const void *Decoder) {
  return DecodeCPURegsRegisterClass(Inst, RegNo, Address, Decoder);
}

//@DecodeMem {
// static DecodeStatus DecodeMem(MCInst &Inst,
//                               unsigned Insn,
//                               uint64_t Address,
//                               const void *Decoder) {
//   //@DecodeMem body {
//   int Offset = SignExtend32<16>(Insn & 0xffff);
//   int Reg    = (int)fieldFromInstruction(Insn, 20, 4);
//   int Base   = (int)fieldFromInstruction(Insn, 16, 4);
//
//   Reg  = getReg(Decoder, MYRISCVX::GPRRegClassID, Reg);
//   Base = getReg(Decoder, MYRISCVX::GPRRegClassID, Base);
//
//   Inst.addOperand(MCOperand::createReg(Reg));
//   Inst.addOperand(MCOperand::createReg(Base));
//   Inst.addOperand(MCOperand::createImm(Offset));
//
//   return MCDisassembler::Success;
// }

static DecodeStatus DecodeBranch12Target(MCInst &Inst,
                                         unsigned Insn,
                                         uint64_t Address,
                                         const void *Decoder) {
  int BranchOffset = SignExtend32<12>((fieldFromInstruction(Insn, 31, 1) << 12) |
                                      (fieldFromInstruction(Insn, 25, 6) <<  5) |
                                      (fieldFromInstruction(Insn,  8, 4) <<  1) |
                                      (fieldFromInstruction(Insn,  7, 1) << 11));
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
  int BranchOffset = SignExtend32<20>((fieldFromInstruction(Insn, 31,  1) << 20) |
                                      (fieldFromInstruction(Insn, 21, 10) <<  1) |
                                      (fieldFromInstruction(Insn, 20,  1) << 11) |
                                      (fieldFromInstruction(Insn, 12,  8) << 12));
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

// static DecodeStatus DecodeJumpFR(MCInst &Inst,
//                                  unsigned Insn,
//                                  uint64_t Address,
//                                  const void *Decoder) {
//   int Reg_a = (int)fieldFromInstruction(Insn, 20, 4);
//
//   Reg_a = getReg(Decoder, MYRISCVX::GPRRegClassID, Reg_a);
//   Inst.addOperand(MCOperand::createReg(Reg_a));
//   Inst.setOpcode(MYRISCVX::JR);
//   return MCDisassembler::Success;
// }

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
