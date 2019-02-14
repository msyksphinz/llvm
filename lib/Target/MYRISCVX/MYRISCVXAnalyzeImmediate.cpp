//===-- MYRISCVXAnalyzeImmediate.cpp - Analyze Immediates ---------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#include "MYRISCVXAnalyzeImmediate.h"

#include "MYRISCVX.h"

#include "llvm/Support/MathExtras.h"
using namespace llvm;
MYRISCVXAnalyzeImmediate::Inst::Inst(unsigned O, unsigned I) : Opc(O), ImmOpnd(I) {}

// Add I to the instruction sequences.
void MYRISCVXAnalyzeImmediate::ADDInstr(InstSeqLs &SeqLs, const Inst &I) {
  // Add an instruction seqeunce consisting of just I.
  if (SeqLs.empty()) {
    SeqLs.push_back(InstSeq(1, I));
    return;
  }
  for (InstSeqLs::iterator Iter = SeqLs.begin(); Iter != SeqLs.end(); ++Iter)
    Iter->push_back(I);
}


void MYRISCVXAnalyzeImmediate::GetInstSeqLsADDI(uint64_t Imm, unsigned RemSize,
                                                InstSeqLs &SeqLs) {
  GetInstSeqLs((Imm + 0x800ULL) & 0xfffffffffffff000ULL, RemSize, SeqLs);
  ADDInstr(SeqLs, Inst(MYRISCVX::ADDI, Imm & 0x0fffULL));
}


void MYRISCVXAnalyzeImmediate::GetInstSeqLsORI(uint64_t Imm, unsigned RemSize,
                                               InstSeqLs &SeqLs) {
  GetInstSeqLs(Imm & 0xfffffffffffff000ULL, RemSize, SeqLs);
  ADDInstr(SeqLs, Inst(MYRISCVX::ORI, Imm & 0x0fffULL));
}


void MYRISCVXAnalyzeImmediate::GetInstSeqLsSRL(uint64_t Imm, unsigned RemSize,
                                               InstSeqLs &SeqLs) {
  unsigned Shamt = countTrailingZeros(Imm);
  GetInstSeqLs(Imm >> Shamt, RemSize - Shamt, SeqLs);
  ADDInstr(SeqLs, Inst(MYRISCVX::SRL, Shamt));
}


void MYRISCVXAnalyzeImmediate::GetInstSeqLs(uint64_t Imm, unsigned RemSize,
                                            InstSeqLs &SeqLs) {
  uint64_t MaskedImm = Imm & (0xffffffffffffffffULL >> (64 - Size));
  // Do nothing if Imm is 0.
  if (!MaskedImm)
    return;
  // A single ADDI will do if RemSize <= 12.
  if (RemSize <= 12) {
    ADDInstr(SeqLs, Inst(MYRISCVX::ADDI, MaskedImm));
    return;
  }
  // Shift if the lower 12-bit is cleared.
  if (!(Imm & 0x0fff)) {
    GetInstSeqLsSRL(Imm, RemSize, SeqLs);
    return;

  }

  GetInstSeqLsADDI(Imm, RemSize, SeqLs);
  // If bit 11 is cleared, it doesn't make a difference whether the last
  // instruction is an ADDI or ORI. In that case, do not call GetInstSeqLsORI.
  if (Imm & 0x0800) {
    InstSeqLs SeqLsORI;
    GetInstSeqLsORI(Imm, RemSize, SeqLsORI);
    SeqLs.insert(SeqLs.end(), SeqLsORI.begin(), SeqLsORI.end());
  }
}


// Replace a ADDI & SRL pair with a LUI.
// e.g. the following two instructions
// ADDI 0x0111
// SRL 18
// are replaced with
// LUI 0x444
void MYRISCVXAnalyzeImmediate::ReplaceADDISRLWithLUI(InstSeq &Seq) {
  // Check if the first two instructions are ADDI and SRL and the shift amount
  // is at least 12.
  if ((Seq.size() < 2) || (Seq[0].Opc != MYRISCVX::ADDI) ||
      (Seq[1].Opc != MYRISCVX::SRL) || (Seq[1].ImmOpnd < 12))
    return;
  // Sign-extend and shift operand of ADDI and see if it still fits in 12-bit.
  int64_t Imm = SignExtend64<12>(Seq[0].ImmOpnd);
  int64_t ShiftedImm = (uint64_t)Imm << (Seq[1].ImmOpnd - 12);
  if (!isInt<12>(ShiftedImm))
    return;
  // Replace the first instruction and erase the second.
  Seq[0].Opc = MYRISCVX::LUI;
  Seq[0].ImmOpnd = (unsigned)(ShiftedImm & 0x0fff);
  Seq.erase(Seq.begin() + 1);
}


void MYRISCVXAnalyzeImmediate::GetShortestSeq(InstSeqLs &SeqLs, InstSeq &Insts) {
  InstSeqLs::iterator ShortestSeq = SeqLs.end();
  // The length of an instruction sequence is at most 7.
  unsigned ShortestLength = 8;
  for (InstSeqLs::iterator S = SeqLs.begin(); S != SeqLs.end(); ++S) {
    ReplaceADDISRLWithLUI(*S);
    assert(S->size() <= 7);
    if (S->size() < ShortestLength) {
      ShortestSeq = S;
      ShortestLength = S->size();
    }
  }
  Insts.clear();
  Insts.append(ShortestSeq->begin(), ShortestSeq->end());
}

const MYRISCVXAnalyzeImmediate::InstSeq
&MYRISCVXAnalyzeImmediate::Analyze(uint64_t Imm, unsigned Size,
                                   bool LastInstrIsADDI) {

  this->Size = Size;
  InstSeqLs SeqLs;
  // Get the list of instruction sequences.
  if (LastInstrIsADDI | !Imm)
    GetInstSeqLsADDI(Imm, Size, SeqLs);
  else
    GetInstSeqLs(Imm, Size, SeqLs);
  // Set Insts to the shortest instruction sequence.
  GetShortestSeq(SeqLs, Insts);
  return Insts;
}
