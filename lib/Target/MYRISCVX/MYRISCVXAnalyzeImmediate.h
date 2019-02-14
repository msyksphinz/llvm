//===-- MYRISCVXAnalyzeImmediate.h - Analyze Immediates ------------*- C++ -*--===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef MYRISCVX_ANALYZE_IMMEDIATE_H
#define MYRISCVX_ANALYZE_IMMEDIATE_H


#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/DataTypes.h"

namespace llvm {
  class MYRISCVXAnalyzeImmediate {
 public:
    struct Inst {
      unsigned Opc, ImmOpnd;
      Inst(unsigned Opc, unsigned ImmOpnd);
    };
    typedef SmallVector<Inst, 7 > InstSeq;
    /// Analyze - Get an instruction sequence to load immediate Imm. The last
    /// instruction in the sequence must be an ADDI if LastInstrIsADDI is
    /// true;
    const InstSeq &Analyze(uint64_t Imm, unsigned Size, bool LastInstrIsADDI);

 private:
    typedef SmallVector<InstSeq, 5> InstSeqLs;
    /// ADDInstr - Add I to all instruction sequences in SeqLs.
    void ADDInstr(InstSeqLs &SeqLs, const Inst &I);

    /// GetInstSeqLsADDI - Get instruction sequences which end with an ADDI to
    /// load immediate Imm
    void GetInstSeqLsADDI(uint64_t Imm, unsigned RemSize, InstSeqLs &SeqLs);

    /// GetInstSeqLsORI - Get instruction sequences which end with an ORI to
    /// load immediate Imm
    void GetInstSeqLsORI(uint64_t Imm, unsigned RemSize, InstSeqLs &SeqLs);

    /// GetInstSeqLsSRL - Get instruction sequences which end with a SRL to
    /// load immediate Imm
    void GetInstSeqLsSRL(uint64_t Imm, unsigned RemSize, InstSeqLs &SeqLs);

    /// GetInstSeqLs - Get instruction sequences to load immediate Imm.
    void GetInstSeqLs(uint64_t Imm, unsigned RemSize, InstSeqLs &SeqLs);

    /// ReplaceADDISRLWithLUI - Replace an ADDI & SRL pair with a LUI.
    void ReplaceADDISRLWithLUI(InstSeq &Seq);

    /// GetShortestSeq - Find the shortest instruction sequence in SeqLs and
    /// return it in Insts.
    void GetShortestSeq(InstSeqLs &SeqLs, InstSeq &Insts);
    unsigned Size;
    InstSeq Insts;
  };
}

#endif
