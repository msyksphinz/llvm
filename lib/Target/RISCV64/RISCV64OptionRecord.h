//===- RISCV64OptionRecord.h - Abstraction for storing information -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// RISCV64OptionRecord - Abstraction for storing arbitrary information in
// ELF files. Arbitrary information (e.g. register usage) can be stored in RISCV64
// specific ELF sections like .RISCV64.options. Specific records should subclass
// RISCV64OptionRecord and provide an implementation to EmitRISCV64OptionRecord which
// basically just dumps the information into an ELF section. More information
// about .RISCV64.option can be found in the SysV ABI and the 64-bit ELF Object
// specification.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MIPS_MIPSOPTIONRECORD_H
#define LLVM_LIB_TARGET_MIPS_MIPSOPTIONRECORD_H

#include "MCTargetDesc/RISCV64MCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCRegisterInfo.h"
#include <cstdint>

namespace llvm {

class RISCV64ELFStreamer;

class RISCV64OptionRecord {
public:
  virtual ~RISCV64OptionRecord() = default;

  virtual void EmitRISCV64OptionRecord() = 0;
};

class RISCV64RegInfoRecord : public RISCV64OptionRecord {
public:
  RISCV64RegInfoRecord(RISCV64ELFStreamer *S, MCContext &Context)
      : Streamer(S), Context(Context) {
    ri_gprmask = 0;
    ri_cprmask[0] = ri_cprmask[1] = ri_cprmask[2] = ri_cprmask[3] = 0;
    ri_gp_value = 0;

    const MCRegisterInfo *TRI = Context.getRegisterInfo();
    GPR32RegClass = &(TRI->getRegClass(RISCV64::GPR32RegClassID));
    GPR64RegClass = &(TRI->getRegClass(RISCV64::GPR64RegClassID));
    FGR32RegClass = &(TRI->getRegClass(RISCV64::FGR32RegClassID));
    FGR64RegClass = &(TRI->getRegClass(RISCV64::FGR64RegClassID));
    AFGR64RegClass = &(TRI->getRegClass(RISCV64::AFGR64RegClassID));
    MSA128BRegClass = &(TRI->getRegClass(RISCV64::MSA128BRegClassID));
    COP0RegClass = &(TRI->getRegClass(RISCV64::COP0RegClassID));
    COP2RegClass = &(TRI->getRegClass(RISCV64::COP2RegClassID));
    COP3RegClass = &(TRI->getRegClass(RISCV64::COP3RegClassID));
  }

  ~RISCV64RegInfoRecord() override = default;

  void EmitRISCV64OptionRecord() override;
  void SetPhysRegUsed(unsigned Reg, const MCRegisterInfo *MCRegInfo);

private:
  RISCV64ELFStreamer *Streamer;
  MCContext &Context;
  const MCRegisterClass *GPR32RegClass;
  const MCRegisterClass *GPR64RegClass;
  const MCRegisterClass *FGR32RegClass;
  const MCRegisterClass *FGR64RegClass;
  const MCRegisterClass *AFGR64RegClass;
  const MCRegisterClass *MSA128BRegClass;
  const MCRegisterClass *COP0RegClass;
  const MCRegisterClass *COP2RegClass;
  const MCRegisterClass *COP3RegClass;
  uint32_t ri_gprmask;
  uint32_t ri_cprmask[4];
  int64_t ri_gp_value;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MIPS_MIPSOPTIONRECORD_H
