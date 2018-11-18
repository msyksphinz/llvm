//===-- RISCV64NaClELFStreamer.cpp - ELF Object Output for RISCV64 NaCl ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements MCELFStreamer for RISCV64 NaCl.  It emits .o object files
// as required by NaCl's SFI sandbox.  It inserts address-masking instructions
// before dangerous control-flow and memory access instructions.  It inserts
// address-masking instructions after instructions that change the stack
// pointer.  It ensures that the mask and the dangerous instruction are always
// emitted in the same bundle.  It aligns call + branch delay to the bundle end,
// so that return address is always aligned to the start of next bundle.
//
//===----------------------------------------------------------------------===//

#include "RISCV64.h"
#include "RISCV64ELFStreamer.h"
#include "RISCV64MCNaCl.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>

using namespace llvm;

#define DEBUG_TYPE "mips-mc-nacl"

namespace {

const unsigned IndirectBranchMaskReg = RISCV64::T6;
const unsigned LoadStoreStackMaskReg = RISCV64::T7;

/// Extend the generic MCELFStreamer class so that it can mask dangerous
/// instructions.

class RISCV64NaClELFStreamer : public RISCV64ELFStreamer {
public:
  RISCV64NaClELFStreamer(MCContext &Context, std::unique_ptr<MCAsmBackend> TAB,
                      std::unique_ptr<MCObjectWriter> OW,
                      std::unique_ptr<MCCodeEmitter> Emitter)
      : RISCV64ELFStreamer(Context, std::move(TAB), std::move(OW),
                        std::move(Emitter)) {}

  ~RISCV64NaClELFStreamer() override = default;

private:
  // Whether we started the sandboxing sequence for calls.  Calls are bundled
  // with branch delays and aligned to the bundle end.
  bool PendingCall = false;

  bool isIndirectJump(const MCInst &MI) {
    if (MI.getOpcode() == RISCV64::JALR) {
      // MIPS32r6/MIPS64r6 doesn't have a JR instruction and uses JALR instead.
      // JALR is an indirect branch if the link register is $0.
      assert(MI.getOperand(0).isReg());
      return MI.getOperand(0).getReg() == RISCV64::ZERO;
    }
    return MI.getOpcode() == RISCV64::JR;
  }

  bool isStackPointerFirstOperand(const MCInst &MI) {
    return (MI.getNumOperands() > 0 && MI.getOperand(0).isReg()
            && MI.getOperand(0).getReg() == RISCV64::SP);
  }

  bool isCall(const MCInst &MI, bool *IsIndirectCall) {
    unsigned Opcode = MI.getOpcode();

    *IsIndirectCall = false;

    switch (Opcode) {
    default:
      return false;

    case RISCV64::JAL:
    case RISCV64::BAL:
    case RISCV64::BAL_BR:
    case RISCV64::BLTZAL:
    case RISCV64::BGEZAL:
      return true;

    case RISCV64::JALR:
      // JALR is only a call if the link register is not $0. Otherwise it's an
      // indirect branch.
      assert(MI.getOperand(0).isReg());
      if (MI.getOperand(0).getReg() == RISCV64::ZERO)
        return false;

      *IsIndirectCall = true;
      return true;
    }
  }

  void emitMask(unsigned AddrReg, unsigned MaskReg,
                const MCSubtargetInfo &STI) {
    MCInst MaskInst;
    MaskInst.setOpcode(RISCV64::AND);
    MaskInst.addOperand(MCOperand::createReg(AddrReg));
    MaskInst.addOperand(MCOperand::createReg(AddrReg));
    MaskInst.addOperand(MCOperand::createReg(MaskReg));
    RISCV64ELFStreamer::EmitInstruction(MaskInst, STI);
  }

  // Sandbox indirect branch or return instruction by inserting mask operation
  // before it.
  void sandboxIndirectJump(const MCInst &MI, const MCSubtargetInfo &STI) {
    unsigned AddrReg = MI.getOperand(0).getReg();

    EmitBundleLock(false);
    emitMask(AddrReg, IndirectBranchMaskReg, STI);
    RISCV64ELFStreamer::EmitInstruction(MI, STI);
    EmitBundleUnlock();
  }

  // Sandbox memory access or SP change.  Insert mask operation before and/or
  // after the instruction.
  void sandboxLoadStoreStackChange(const MCInst &MI, unsigned AddrIdx,
                                   const MCSubtargetInfo &STI, bool MaskBefore,
                                   bool MaskAfter) {
    EmitBundleLock(false);
    if (MaskBefore) {
      // Sandbox memory access.
      unsigned BaseReg = MI.getOperand(AddrIdx).getReg();
      emitMask(BaseReg, LoadStoreStackMaskReg, STI);
    }
    RISCV64ELFStreamer::EmitInstruction(MI, STI);
    if (MaskAfter) {
      // Sandbox SP change.
      unsigned SPReg = MI.getOperand(0).getReg();
      assert((RISCV64::SP == SPReg) && "Unexpected stack-pointer register.");
      emitMask(SPReg, LoadStoreStackMaskReg, STI);
    }
    EmitBundleUnlock();
  }

public:
  /// This function is the one used to emit instruction data into the ELF
  /// streamer.  We override it to mask dangerous instructions.
  void EmitInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                       bool) override {
    // Sandbox indirect jumps.
    if (isIndirectJump(Inst)) {
      if (PendingCall)
        report_fatal_error("Dangerous instruction in branch delay slot!");
      sandboxIndirectJump(Inst, STI);
      return;
    }

    // Sandbox loads, stores and SP changes.
    unsigned AddrIdx;
    bool IsStore;
    bool IsMemAccess = isBasePlusOffsetMemoryAccess(Inst.getOpcode(), &AddrIdx,
                                                    &IsStore);
    bool IsSPFirstOperand = isStackPointerFirstOperand(Inst);
    if (IsMemAccess || IsSPFirstOperand) {
      bool MaskBefore = (IsMemAccess
                         && baseRegNeedsLoadStoreMask(Inst.getOperand(AddrIdx)
                                                          .getReg()));
      bool MaskAfter = IsSPFirstOperand && !IsStore;
      if (MaskBefore || MaskAfter) {
        if (PendingCall)
          report_fatal_error("Dangerous instruction in branch delay slot!");
        sandboxLoadStoreStackChange(Inst, AddrIdx, STI, MaskBefore, MaskAfter);
        return;
      }
      // fallthrough
    }

    // Sandbox calls by aligning call and branch delay to the bundle end.
    // For indirect calls, emit the mask before the call.
    bool IsIndirectCall;
    if (isCall(Inst, &IsIndirectCall)) {
      if (PendingCall)
        report_fatal_error("Dangerous instruction in branch delay slot!");

      // Start the sandboxing sequence by emitting call.
      EmitBundleLock(true);
      if (IsIndirectCall) {
        unsigned TargetReg = Inst.getOperand(1).getReg();
        emitMask(TargetReg, IndirectBranchMaskReg, STI);
      }
      RISCV64ELFStreamer::EmitInstruction(Inst, STI);
      PendingCall = true;
      return;
    }
    if (PendingCall) {
      // Finish the sandboxing sequence by emitting branch delay.
      RISCV64ELFStreamer::EmitInstruction(Inst, STI);
      EmitBundleUnlock();
      PendingCall = false;
      return;
    }

    // None of the sandboxing applies, just emit the instruction.
    RISCV64ELFStreamer::EmitInstruction(Inst, STI);
  }
};

} // end anonymous namespace

namespace llvm {

bool isBasePlusOffsetMemoryAccess(unsigned Opcode, unsigned *AddrIdx,
                                  bool *IsStore) {
  if (IsStore)
    *IsStore = false;

  switch (Opcode) {
  default:
    return false;

  // Load instructions with base address register in position 1.
  case RISCV64::LB:
  case RISCV64::LBu:
  case RISCV64::LH:
  case RISCV64::LHu:
  case RISCV64::LW:
  case RISCV64::LWC1:
  case RISCV64::LDC1:
  case RISCV64::LL:
  case RISCV64::LL_R6:
  case RISCV64::LWL:
  case RISCV64::LWR:
    *AddrIdx = 1;
    return true;

  // Store instructions with base address register in position 1.
  case RISCV64::SB:
  case RISCV64::SH:
  case RISCV64::SW:
  case RISCV64::SWC1:
  case RISCV64::SDC1:
  case RISCV64::SWL:
  case RISCV64::SWR:
    *AddrIdx = 1;
    if (IsStore)
      *IsStore = true;
    return true;

  // Store instructions with base address register in position 2.
  case RISCV64::SC:
  case RISCV64::SC_R6:
    *AddrIdx = 2;
    if (IsStore)
      *IsStore = true;
    return true;
  }
}

bool baseRegNeedsLoadStoreMask(unsigned Reg) {
  // The contents of SP and thread pointer register do not require masking.
  return Reg != RISCV64::SP && Reg != RISCV64::T8;
}

MCELFStreamer *createRISCV64NaClELFStreamer(MCContext &Context,
                                         std::unique_ptr<MCAsmBackend> TAB,
                                         std::unique_ptr<MCObjectWriter> OW,
                                         std::unique_ptr<MCCodeEmitter> Emitter,
                                         bool RelaxAll) {
  RISCV64NaClELFStreamer *S = new RISCV64NaClELFStreamer(
      Context, std::move(TAB), std::move(OW), std::move(Emitter));
  if (RelaxAll)
    S->getAssembler().setRelaxAll(true);

  // Set bundle-alignment as required by the NaCl ABI for the target.
  S->EmitBundleAlignMode(MIPS_NACL_BUNDLE_ALIGN);

  return S;
}

} // end namespace llvm
