//===-- MYRISCVXAsmPrinter.cpp - MYRISCVX LLVM Assembly Printer -------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format MYRISCVX assembly language.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVXAsmPrinter.h"
#include "InstPrinter/MYRISCVXInstPrinter.h"
#include "MCTargetDesc/MYRISCVXBaseInfo.h"
#include "MYRISCVX.h"
#include "MYRISCVXInstrInfo.h"

#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/Twine.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Target/TargetLoweringObjectFile.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;
#define DEBUG_TYPE "myriscvx-asm-printer"

//@adjustFixupValue {
// Prepare value for the target space for it
static unsigned adjustFixupValue(const MCFixup &Fixup, uint64_t Value,
                                 MCContext *Ctx = nullptr) {

  unsigned Kind = Fixup.getKind();

  // Add/subtract and shift
  switch (Kind) {
  default:
    return 0;
  case FK_GPRel_4:
  case FK_Data_4:
  case MYRISCVX::fixup_MYRISCVX_CALL16:
  case MYRISCVX::fixup_MYRISCVX_LO16:
  case MYRISCVX::fixup_MYRISCVX_GOT_LO16:
    break;
  case MYRISCVX::fixup_MYRISCVX_PCREL_LO12_I:
  case MYRISCVX::fixup_MYRISCVX_PCREL_LO12_S:
    // So far we are only using this type for branches and jump.
    // For branches we start 1 instruction after the branch
    // so the displacement will be one instruction size less.
    Value -= 4;
    break;
  case MYRISCVX::fixup_MYRISCVX_HI16:
  case MYRISCVX::fixup_MYRISCVX_GOT:
  case MYRISCVX::fixup_MYRISCVX_GOT_HI16:
    // Get the higher 16-bits. Also add 1 if bit 15 is 1.
    Value = ((Value + 0x8000) >> 16) & 0xffff;
    break;
  }

  return Value;
}
//@adjustFixupValue }


#ifdef ENABLE_GPRESTORE
void MYRISCVXAsmPrinter::EmitInstrWithMacroNoAT(const MachineInstr *MI) {
  MCInst TmpInst;

  MCInstLowering.Lower(MI, TmpInst);
  OutStreamer->EmitRawText(StringRef("\t.set\tmacro"));
  // if (MYRISCVXFI->getEmitNOAT())
  //   OutStreamer->EmitRawText(StringRef("\t.set\tat"));
  // OutStreamer->EmitInstruction(TmpInst, getSubtargetInfo());
  // if (MYRISCVXFI->getEmitNOAT())
  //   OutStreamer->EmitRawText(StringRef("\t.set\tnoat"));
  OutStreamer->EmitRawText(StringRef("\t.set\tnomacro"));
}
#endif


bool MYRISCVXAsmPrinter::runOnMachineFunction(MachineFunction &MF) {
  MYRISCVXFI = MF.getInfo<MYRISCVXFunctionInfo>();
  AsmPrinter::runOnMachineFunction(MF);
  return true;
}


bool MYRISCVXAsmPrinter::lowerOperand(const MachineOperand &MO, MCOperand &MCOp) {
  MCOp = MCInstLowering.LowerOperand(MO);
  return MCOp.isValid();
}


#include "MYRISCVXGenMCPseudoLowering.inc"


#ifdef ENABLE_GPRESTORE
void MYRISCVXAsmPrinter::emitPseudoCPRestore(MCStreamer &OutStreamer,
                                              const MachineInstr *MI) {
  unsigned Opc = MI->getOpcode();
  SmallVector<MCInst, 4> MCInsts;
  const MachineOperand &MO = MI->getOperand(0);
  assert(MO.isImm() && "CPRESTORE's operand must be an immediate.");
  int64_t Offset = MO.getImm();

  if (OutStreamer.hasRawTextSupport()) {
    // output assembly
    if (!isInt<16>(Offset)) {
      EmitInstrWithMacroNoAT(MI);
      return;
    }
    MCInst TmpInst0;
    MCInstLowering.Lower(MI, TmpInst0);
    OutStreamer.EmitInstruction(TmpInst0, getSubtargetInfo());
  } else {
    // output elf
    MCInstLowering.LowerCPRESTORE(Offset, MCInsts);

    for (SmallVector<MCInst, 4>::iterator I = MCInsts.begin();
         I != MCInsts.end(); ++I)
      OutStreamer.EmitInstruction(*I, getSubtargetInfo());

    return;
  }
}
#endif



//@EmitInstruction {
//- EmitInstruction() must exists or will have run time error.
void MYRISCVXAsmPrinter::EmitInstruction(const MachineInstr *MI) {
  // Do any auto-generated pseudo lowerings.
  if (emitPseudoExpansionLowering(*OutStreamer, MI))
    return;

  //@EmitInstruction body {
  if (MI->isDebugValue()) {
    SmallString<128> Str;
    raw_svector_ostream OS(Str);
    PrintDebugValueComment(MI, OS);
    return;
  }
  //@print out instruction:
  // Print out both ordinary instruction and boudle instruction
  MachineBasicBlock::const_instr_iterator I = MI->getIterator();
  MachineBasicBlock::const_instr_iterator E = MI->getParent()->instr_end();
  do {
    // Do any auto-generated pseudo lowerings.
    if (emitPseudoExpansionLowering(*OutStreamer, &*I))
      continue;

#ifdef ENABLE_GPRESTORE
    if (I->getOpcode() == MYRISCVX::CPRESTORE) {
      emitPseudoCPRestore(*OutStreamer, &*I);
      continue;
    }
#endif

    if (I->isPseudo() && !isLongBranchPseudo(I->getOpcode())) {
      LLVM_DEBUG(dbgs() << "opcode = " << I->getOpcode() << '\n');
      llvm_unreachable("Pseudo opcode found in EmitInstruction()");
    }
    MCInst TmpInst0;
    MCInstLowering.Lower(&*I, TmpInst0);
    OutStreamer->EmitInstruction(TmpInst0, getSubtargetInfo());
  } while ((++I != E) && I->isInsideBundle()); // Delay slot check
}


//@EmitInstruction }
//===----------------------------------------------------------------------===//
//
// MYRISCVX Asm Directives
//
// -- Frame directive "frame Stackpointer, Stacksize, RARegister"
// Describe the stack frame.
//
// -- Mask directives "(f)mask bitmask, offset"
// Tells the assembler which registers are saved and where.
// bitmask - contain a little endian bitset indicating which registers are
// saved on function prologue (e.g. with a 0x80000000 mask, the
// assembler knows the register 31 (RA) is saved at prologue.
// offset - the position before stack pointer subtraction indicating where
// the first saved register on prologue is located. (e.g. with a
//
// Consider the following function prologue:
//
// .frame $fp,48,$ra
// .mask 0xc0000000,-8
// addiu $sp, $sp, -48
// st $ra, 40($sp)
// st $fp, 36($sp)
//
// With a 0xc0000000 mask, the assembler knows the register 31 (RA) and
// 30 (FP) are saved at prologue. As the save order on prologue is from
// left to right, RA is saved first. A -8 offset means that after the
// stack pointer subtration, the first register in the mask (RA) will be
// saved at address 48-8=40.
//
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// Mask directives
//===----------------------------------------------------------------------===//
// .frame $sp,8,$lr
//-> .mask 0x00000000,0
// .set noreorder
// .set nomacro
// Create a bitmask with all callee saved registers for CPU or Floating Point
// registers. For CPU registers consider LR, GP and FP for saving if necessary.

void MYRISCVXAsmPrinter::printSavedRegsBitmask(raw_ostream &O) {
  // CPU and FPU Saved Registers Bitmasks
  unsigned CPUBitmask = 0;
  int CPUTopSavedRegOff;
  // Set the CPU and FPU Bitmasks
  const MachineFrameInfo &MFI = MF->getFrameInfo();
  const TargetRegisterInfo *TRI = MF->getSubtarget().getRegisterInfo();
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  // size of stack area to which FP callee-saved regs are saved.
  unsigned CPURegSize = TRI->getRegSizeInBits(MYRISCVX::GPRRegClass) / 8;
  unsigned i = 0, e = CSI.size();
  // Set CPU Bitmask.
  for (; i != e; ++i) {
    unsigned Reg = CSI[i].getReg();
    unsigned RegNum = TRI->getEncodingValue(Reg);
    CPUBitmask |= (1 << RegNum);
  }
  CPUTopSavedRegOff = CPUBitmask ? -CPURegSize : 0;
  // Print CPUBitmask
  O << "\t.mask \t"; printHex32(CPUBitmask, O);
  O << ',' << CPUTopSavedRegOff << '\n';
}

// Print a 32 bit hex number with all numbers.
void MYRISCVXAsmPrinter::printHex32(unsigned Value, raw_ostream &O) {
  O << "0x";
  for (int i = 7; i >= 0; i--)
    O.write_hex((Value & (0xF << (i*4))) >> (i*4));
}

//===----------------------------------------------------------------------===//
// Frame and Set directives
//===----------------------------------------------------------------------===//
//-> .frame $sp,8,$lr
// .mask 0x00000000,0
// .set noreorder
// .set nomacro
/// Frame Directive
void MYRISCVXAsmPrinter::emitFrameDirective() {
  const TargetRegisterInfo &RI = *MF->getSubtarget().getRegisterInfo();
  unsigned stackReg = RI.getFrameRegister(*MF);
  unsigned returnReg = RI.getRARegister();
  unsigned stackSize = MF->getFrameInfo().getStackSize();
  if (OutStreamer->hasRawTextSupport())
    OutStreamer->EmitRawText("\t.frame\t$" +
                             StringRef(MYRISCVXInstPrinter::getRegisterName(stackReg)).lower() +
                             "," + Twine(stackSize) + ",$" +
                             StringRef(MYRISCVXInstPrinter::getRegisterName(returnReg)).lower());
}


/// Emit Set directives.
const char *MYRISCVXAsmPrinter::getCurrentABIString() const {
  switch (static_cast<MYRISCVXTargetMachine &>(TM).getABI().GetEnumValue()) {
    case MYRISCVXABIInfo::ABI::O32: return "abiO32";
    case MYRISCVXABIInfo::ABI::S32: return "abiS32";
    default: llvm_unreachable("Unknown MYRISCVX ABI");
  }
}

// .type main,@function
//-> .ent main # @main
// main:
void MYRISCVXAsmPrinter::EmitFunctionEntryLabel() {
  if (OutStreamer->hasRawTextSupport())
    OutStreamer->EmitRawText("\t.ent\t" + Twine(CurrentFnSym->getName()));
  OutStreamer->EmitLabel(CurrentFnSym);
}


// .frame $sp,8,$pc
// .mask 0x00000000,0
//-> .set noreorder
//@-> .set nomacro
/// EmitFunctionBodyStart - Targets can override this to emit stuff before
/// the first basic block in the function.
void MYRISCVXAsmPrinter::EmitFunctionBodyStart() {
  MCInstLowering.Initialize(&MF->getContext());

  emitFrameDirective();
  bool EmitCPLoad = (MF->getTarget().getRelocationModel() == Reloc::PIC_) &&
    MYRISCVXFI->globalBaseRegSet() &&
    MYRISCVXFI->globalBaseRegFixed();
  if (MYRISCVXNoCpload)
    EmitCPLoad = false;

  if (OutStreamer->hasRawTextSupport()) {
    SmallString<128> Str;
    raw_svector_ostream OS(Str);
    printSavedRegsBitmask(OS);
    OutStreamer->EmitRawText(OS.str());
    OutStreamer->EmitRawText(StringRef("\t.set\tnoreorder"));
    // Emit .cpload directive if needed.
    if (EmitCPLoad)
      OutStreamer->EmitRawText(StringRef("\t.cpload\t$t9"));
    OutStreamer->EmitRawText(StringRef("\t.set\tnomacro"));
    // if (MYRISCVXFI->getEmitNOAT())
    //   OutStreamer->EmitRawText(StringRef("\t.set\tnoat"));
  } else if (EmitCPLoad) {
    SmallVector<MCInst, 4> MCInsts;
    MCInstLowering.LowerCPLOAD(MCInsts);
    for (SmallVector<MCInst, 4>::iterator I = MCInsts.begin();
       I != MCInsts.end(); ++I)
      OutStreamer->EmitInstruction(*I, getSubtargetInfo());
  }
}


//-> .set macro
//-> .set reorder
//-> .end main
/// EmitFunctionBodyEnd - Targets can override this to emit stuff after
/// the last basic block in the function.
void MYRISCVXAsmPrinter::EmitFunctionBodyEnd() {
  // There are instruction for this macros, but they must
  // always be at the function end, and we can't emit and
  // break with BB logic.
  if (OutStreamer->hasRawTextSupport()) {
    OutStreamer->EmitRawText(StringRef("\t.set\tmacro"));
    OutStreamer->EmitRawText(StringRef("\t.set\treorder"));
    OutStreamer->EmitRawText("\t.end\t" + Twine(CurrentFnSym->getName()));
  }
}


// .section .mdebug.abi32
// .previous
void MYRISCVXAsmPrinter::EmitStartOfAsmFile(Module &M) {
  // FIXME: Use SwitchSection.
  // Tell the assembler which ABI we are using
  if (OutStreamer->hasRawTextSupport())
    OutStreamer->EmitRawText("\t.section .mdebug." +
                             Twine(getCurrentABIString()));
  // return to previous section
  if (OutStreamer->hasRawTextSupport())
    OutStreamer->EmitRawText(StringRef("\t.previous"));
}


void MYRISCVXAsmPrinter::PrintDebugValueComment(const MachineInstr *MI,
                                                raw_ostream &OS) {
  // TODO: implement
  OS << "PrintDebugValueComment()";
}


// Force static initialization.
extern "C" void LLVMInitializeMYRISCVXAsmPrinter() {
  RegisterAsmPrinter<MYRISCVXAsmPrinter> X(TheMYRISCVX32Target);
  RegisterAsmPrinter<MYRISCVXAsmPrinter> Y(TheMYRISCVX64Target);
}


bool MYRISCVXAsmPrinter::isLongBranchPseudo(int Opcode) const {
  return (Opcode == MYRISCVX::LONG_BRANCH_LUI ||
          Opcode == MYRISCVX::LONG_BRANCH_ADDI);
}


// Print out an operand for an inline asm expression.
bool MYRISCVXAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNum,
                                         unsigned AsmVariant,const char *ExtraCode,
                                         raw_ostream &O) {
  // Does this asm operand have a single letter operand modifier?
  if (ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0) return true; // Unknown modifier.

    const MachineOperand &MO = MI->getOperand(OpNum);
    switch (ExtraCode[0]) {
      default:
        // See if this is a generic print operand
        return AsmPrinter::PrintAsmOperand(MI,OpNum,AsmVariant,ExtraCode,O);
      case 'X': // hex const int
        if ((MO.getType()) != MachineOperand::MO_Immediate)
          return true;
        O << "0x" << StringRef(utohexstr(MO.getImm())).lower();
        return false;
      case 'x': // hex const int (low 16 bits)
        if ((MO.getType()) != MachineOperand::MO_Immediate)
          return true;
        O << "0x" << StringRef(utohexstr(MO.getImm() & 0xffff)).lower();
        return false;
      case 'd': // decimal const int
        if ((MO.getType()) != MachineOperand::MO_Immediate)
          return true;
        O << MO.getImm();
        return false;
      case 'm': // decimal const int minus 1
        if ((MO.getType()) != MachineOperand::MO_Immediate)
          return true;
        O << MO.getImm() - 1;
        return false;
      case 'z': {
        // $0 if zero, regular printing otherwise
        if (MO.getType() != MachineOperand::MO_Immediate)
          return true;
        int64_t Val = MO.getImm();
        if (Val)
          O << Val;
        else
          O << "$0";
        return false;
      }
    }
  }

  printOperand(MI, OpNum, O);
  return false;
}

bool MYRISCVXAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                               unsigned OpNum, unsigned AsmVariant,
                                               const char *ExtraCode,
                                               raw_ostream &O) {
  int Offset = 0;
  // Currently we are expecting either no ExtraCode or 'D'
  if (ExtraCode) {
    return true; // Unknown modifier.
  }

  const MachineOperand &MO = MI->getOperand(OpNum);
  assert(MO.isReg() && "unexpected inline asm memory operand");
  O << Offset << "($" << MYRISCVXInstPrinter::getRegisterName(MO.getReg()) << ")";

  return false;
}

void MYRISCVXAsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                      raw_ostream &O) {
  const MachineOperand &MO = MI->getOperand(opNum);
  bool closeP = false;

  if (MO.getTargetFlags())
    closeP = true;

  switch(MO.getTargetFlags()) {
    case MYRISCVXII::MO_GPREL:    O << "%gp_rel("; break;
    case MYRISCVXII::MO_GOT_CALL: O << "%call16("; break;
    case MYRISCVXII::MO_GOT:      O << "%got(";    break;
    case MYRISCVXII::MO_ABS_HI:   O << "%hi(";     break;
    case MYRISCVXII::MO_ABS_LO:   O << "%lo(";     break;
    case MYRISCVXII::MO_GOT_HI16: O << "%got_hi16("; break;
    case MYRISCVXII::MO_GOT_LO16: O << "%got_lo16("; break;
  }

  switch (MO.getType()) {
    case MachineOperand::MO_Register:
      O << '$'
        << StringRef(MYRISCVXInstPrinter::getRegisterName(MO.getReg())).lower();
      break;

    case MachineOperand::MO_Immediate:
      O << MO.getImm();
      break;

    case MachineOperand::MO_MachineBasicBlock:
      O << *MO.getMBB()->getSymbol();
      return;

    case MachineOperand::MO_GlobalAddress:
      O << *getSymbol(MO.getGlobal());
      break;

    case MachineOperand::MO_BlockAddress: {
      MCSymbol *BA = GetBlockAddressSymbol(MO.getBlockAddress());
      O << BA->getName();
      break;
    }

    case MachineOperand::MO_ExternalSymbol:
      O << *GetExternalSymbolSymbol(MO.getSymbolName());
      break;

    case MachineOperand::MO_JumpTableIndex:
      O << MAI->getPrivateGlobalPrefix() << "JTI" << getFunctionNumber()
        << '_' << MO.getIndex();
      break;

    case MachineOperand::MO_ConstantPoolIndex:
      O << MAI->getPrivateGlobalPrefix() << "CPI"
        << getFunctionNumber() << "_" << MO.getIndex();
      if (MO.getOffset())
        O << "+" << MO.getOffset();
      break;

    default:
      llvm_unreachable("<unknown operand type>");
  }

  if (closeP) O << ")";
}
