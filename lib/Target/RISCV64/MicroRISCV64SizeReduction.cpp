//=== MicroRISCV64SizeReduction.cpp - MicroRISCV64 size reduction pass --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///\file
/// This pass is used to reduce the size of instructions where applicable.
///
/// TODO: Implement microMIPS64 support.
//===----------------------------------------------------------------------===//
#include "RISCV64.h"
#include "RISCV64InstrInfo.h"
#include "RISCV64Subtarget.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "micromips-reduce-size"
#define MICROMIPS_SIZE_REDUCE_NAME "MicroRISCV64 instruction size reduce pass"

STATISTIC(NumReduced, "Number of instructions reduced (32-bit to 16-bit ones, "
                      "or two instructions into one");

namespace {

/// Order of operands to transfer
// TODO: Will be extended when additional optimizations are added
enum OperandTransfer {
  OT_NA,            ///< Not applicable
  OT_OperandsAll,   ///< Transfer all operands
  OT_Operands02,    ///< Transfer operands 0 and 2
  OT_Operand2,      ///< Transfer just operand 2
  OT_OperandsXOR,   ///< Transfer operands for XOR16
  OT_OperandsLwp,   ///< Transfer operands for LWP
  OT_OperandsSwp,   ///< Transfer operands for SWP
  OT_OperandsMovep, ///< Transfer operands for MOVEP
};

/// Reduction type
// TODO: Will be extended when additional optimizations are added
enum ReduceType {
  RT_TwoInstr, ///< Reduce two instructions into one instruction
  RT_OneInstr  ///< Reduce one instruction into a smaller instruction
};

// Information about immediate field restrictions
struct ImmField {
  ImmField() : ImmFieldOperand(-1), Shift(0), LBound(0), HBound(0) {}
  ImmField(uint8_t Shift, int16_t LBound, int16_t HBound,
           int8_t ImmFieldOperand)
      : ImmFieldOperand(ImmFieldOperand), Shift(Shift), LBound(LBound),
        HBound(HBound) {}
  int8_t ImmFieldOperand; // Immediate operand, -1 if it does not exist
  uint8_t Shift;          // Shift value
  int16_t LBound;         // Low bound of the immediate operand
  int16_t HBound;         // High bound of the immediate operand
};

/// Information about operands
// TODO: Will be extended when additional optimizations are added
struct OpInfo {
  OpInfo(enum OperandTransfer TransferOperands)
      : TransferOperands(TransferOperands) {}
  OpInfo() : TransferOperands(OT_NA) {}

  enum OperandTransfer
      TransferOperands; ///< Operands to transfer to the new instruction
};

// Information about opcodes
struct OpCodes {
  OpCodes(unsigned WideOpc, unsigned NarrowOpc)
      : WideOpc(WideOpc), NarrowOpc(NarrowOpc) {}

  unsigned WideOpc;   ///< Wide opcode
  unsigned NarrowOpc; ///< Narrow opcode
};

typedef struct ReduceEntryFunArgs ReduceEntryFunArgs;

/// ReduceTable - A static table with information on mapping from wide
/// opcodes to narrow
struct ReduceEntry {

  enum ReduceType eRType; ///< Reduction type
  bool (*ReduceFunction)(
      ReduceEntryFunArgs *Arguments); ///< Pointer to reduce function
  struct OpCodes Ops;                 ///< All relevant OpCodes
  struct OpInfo OpInf;                ///< Characteristics of operands
  struct ImmField Imm;                ///< Characteristics of immediate field

  ReduceEntry(enum ReduceType RType, struct OpCodes Op,
              bool (*F)(ReduceEntryFunArgs *Arguments), struct OpInfo OpInf,
              struct ImmField Imm)
      : eRType(RType), ReduceFunction(F), Ops(Op), OpInf(OpInf), Imm(Imm) {}

  unsigned NarrowOpc() const { return Ops.NarrowOpc; }
  unsigned WideOpc() const { return Ops.WideOpc; }
  int16_t LBound() const { return Imm.LBound; }
  int16_t HBound() const { return Imm.HBound; }
  uint8_t Shift() const { return Imm.Shift; }
  int8_t ImmField() const { return Imm.ImmFieldOperand; }
  enum OperandTransfer TransferOperands() const {
    return OpInf.TransferOperands;
  }
  enum ReduceType RType() const { return eRType; }

  // operator used by std::equal_range
  bool operator<(const unsigned int r) const { return (WideOpc() < r); }

  // operator used by std::equal_range
  friend bool operator<(const unsigned int r, const struct ReduceEntry &re) {
    return (r < re.WideOpc());
  }
};

// Function arguments for ReduceFunction
struct ReduceEntryFunArgs {
  MachineInstr *MI;         // Instruction
  const ReduceEntry &Entry; // Entry field
  MachineBasicBlock::instr_iterator
      &NextMII; // Iterator to next instruction in block

  ReduceEntryFunArgs(MachineInstr *argMI, const ReduceEntry &argEntry,
                     MachineBasicBlock::instr_iterator &argNextMII)
      : MI(argMI), Entry(argEntry), NextMII(argNextMII) {}
};

typedef llvm::SmallVector<ReduceEntry, 32> ReduceEntryVector;

class MicroRISCV64SizeReduce : public MachineFunctionPass {
public:
  static char ID;
  MicroRISCV64SizeReduce();

  static const RISCV64InstrInfo *RISCV64II;
  const RISCV64Subtarget *Subtarget;

  bool runOnMachineFunction(MachineFunction &MF) override;

  llvm::StringRef getPassName() const override {
    return "microMIPS instruction size reduction pass";
  }

private:
  /// Reduces width of instructions in the specified basic block.
  bool ReduceMBB(MachineBasicBlock &MBB);

  /// Attempts to reduce MI, returns true on success.
  bool ReduceMI(const MachineBasicBlock::instr_iterator &MII,
                MachineBasicBlock::instr_iterator &NextMII);

  // Attempts to reduce LW/SW instruction into LWSP/SWSP,
  // returns true on success.
  static bool ReduceXWtoXWSP(ReduceEntryFunArgs *Arguments);

  // Attempts to reduce two LW/SW instructions into LWP/SWP instruction,
  // returns true on success.
  static bool ReduceXWtoXWP(ReduceEntryFunArgs *Arguments);

  // Attempts to reduce LBU/LHU instruction into LBU16/LHU16,
  // returns true on success.
  static bool ReduceLXUtoLXU16(ReduceEntryFunArgs *Arguments);

  // Attempts to reduce SB/SH instruction into SB16/SH16,
  // returns true on success.
  static bool ReduceSXtoSX16(ReduceEntryFunArgs *Arguments);

  // Attempts to reduce two MOVE instructions into MOVEP instruction,
  // returns true on success.
  static bool ReduceMoveToMovep(ReduceEntryFunArgs *Arguments);

  // Attempts to reduce arithmetic instructions, returns true on success.
  static bool ReduceArithmeticInstructions(ReduceEntryFunArgs *Arguments);

  // Attempts to reduce ADDIU into ADDIUSP instruction,
  // returns true on success.
  static bool ReduceADDIUToADDIUSP(ReduceEntryFunArgs *Arguments);

  // Attempts to reduce ADDIU into ADDIUR1SP instruction,
  // returns true on success.
  static bool ReduceADDIUToADDIUR1SP(ReduceEntryFunArgs *Arguments);

  // Attempts to reduce XOR into XOR16 instruction,
  // returns true on success.
  static bool ReduceXORtoXOR16(ReduceEntryFunArgs *Arguments);

  // Changes opcode of an instruction, replaces an instruction with a
  // new one, or replaces two instructions with a new instruction
  // depending on their order i.e. if these are consecutive forward
  // or consecutive backward
  static bool ReplaceInstruction(MachineInstr *MI, const ReduceEntry &Entry,
                                 MachineInstr *MI2 = nullptr,
                                 bool ConsecutiveForward = true);

  // Table with transformation rules for each instruction.
  static ReduceEntryVector ReduceTable;
};

char MicroRISCV64SizeReduce::ID = 0;
const RISCV64InstrInfo *MicroRISCV64SizeReduce::RISCV64II;

// This table must be sorted by WideOpc as a main criterion and
// ReduceType as a sub-criterion (when wide opcodes are the same).
ReduceEntryVector MicroRISCV64SizeReduce::ReduceTable = {

    // ReduceType, OpCodes, ReduceFunction,
    // OpInfo(TransferOperands),
    // ImmField(Shift, LBound, HBound, ImmFieldPosition)
    {RT_OneInstr, OpCodes(RISCV64::ADDiu, RISCV64::ADDIUR1SP_MM),
     ReduceADDIUToADDIUR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(RISCV64::ADDiu, RISCV64::ADDIUSP_MM), ReduceADDIUToADDIUSP,
     OpInfo(OT_Operand2), ImmField(0, 0, 0, 2)},
    {RT_OneInstr, OpCodes(RISCV64::ADDiu_MM, RISCV64::ADDIUR1SP_MM),
     ReduceADDIUToADDIUR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(RISCV64::ADDiu_MM, RISCV64::ADDIUSP_MM),
     ReduceADDIUToADDIUSP, OpInfo(OT_Operand2), ImmField(0, 0, 0, 2)},
    {RT_OneInstr, OpCodes(RISCV64::ADDu, RISCV64::ADDU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(RISCV64::ADDu_MM, RISCV64::ADDU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(RISCV64::LBu, RISCV64::LBU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(0, -1, 15, 2)},
    {RT_OneInstr, OpCodes(RISCV64::LBu_MM, RISCV64::LBU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(0, -1, 15, 2)},
    {RT_OneInstr, OpCodes(RISCV64::LEA_ADDiu, RISCV64::ADDIUR1SP_MM),
     ReduceADDIUToADDIUR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(RISCV64::LEA_ADDiu_MM, RISCV64::ADDIUR1SP_MM),
     ReduceADDIUToADDIUR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(RISCV64::LHu, RISCV64::LHU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(RISCV64::LHu_MM, RISCV64::LHU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_TwoInstr, OpCodes(RISCV64::LW, RISCV64::LWP_MM), ReduceXWtoXWP,
     OpInfo(OT_OperandsLwp), ImmField(0, -2048, 2048, 2)},
    {RT_OneInstr, OpCodes(RISCV64::LW, RISCV64::LWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_TwoInstr, OpCodes(RISCV64::LW16_MM, RISCV64::LWP_MM), ReduceXWtoXWP,
     OpInfo(OT_OperandsLwp), ImmField(0, -2048, 2048, 2)},
    {RT_TwoInstr, OpCodes(RISCV64::LW_MM, RISCV64::LWP_MM), ReduceXWtoXWP,
     OpInfo(OT_OperandsLwp), ImmField(0, -2048, 2048, 2)},
    {RT_OneInstr, OpCodes(RISCV64::LW_MM, RISCV64::LWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_TwoInstr, OpCodes(RISCV64::MOVE16_MM, RISCV64::MOVEP_MM), ReduceMoveToMovep,
     OpInfo(OT_OperandsMovep), ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(RISCV64::SB, RISCV64::SB16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(0, 0, 16, 2)},
    {RT_OneInstr, OpCodes(RISCV64::SB_MM, RISCV64::SB16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(0, 0, 16, 2)},
    {RT_OneInstr, OpCodes(RISCV64::SH, RISCV64::SH16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(RISCV64::SH_MM, RISCV64::SH16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(RISCV64::SUBu, RISCV64::SUBU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(RISCV64::SUBu_MM, RISCV64::SUBU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_TwoInstr, OpCodes(RISCV64::SW, RISCV64::SWP_MM), ReduceXWtoXWP,
     OpInfo(OT_OperandsSwp), ImmField(0, -2048, 2048, 2)},
    {RT_OneInstr, OpCodes(RISCV64::SW, RISCV64::SWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_TwoInstr, OpCodes(RISCV64::SW16_MM, RISCV64::SWP_MM), ReduceXWtoXWP,
     OpInfo(OT_OperandsSwp), ImmField(0, -2048, 2048, 2)},
    {RT_TwoInstr, OpCodes(RISCV64::SW_MM, RISCV64::SWP_MM), ReduceXWtoXWP,
     OpInfo(OT_OperandsSwp), ImmField(0, -2048, 2048, 2)},
    {RT_OneInstr, OpCodes(RISCV64::SW_MM, RISCV64::SWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_OneInstr, OpCodes(RISCV64::XOR, RISCV64::XOR16_MM), ReduceXORtoXOR16,
     OpInfo(OT_OperandsXOR), ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(RISCV64::XOR_MM, RISCV64::XOR16_MM), ReduceXORtoXOR16,
     OpInfo(OT_OperandsXOR), ImmField(0, 0, 0, -1)}};
} // end anonymous namespace

INITIALIZE_PASS(MicroRISCV64SizeReduce, DEBUG_TYPE, MICROMIPS_SIZE_REDUCE_NAME,
                false, false)

// Returns true if the machine operand MO is register SP.
static bool IsSP(const MachineOperand &MO) {
  if (MO.isReg() && ((MO.getReg() == RISCV64::SP)))
    return true;
  return false;
}

// Returns true if the machine operand MO is register $16, $17, or $2-$7.
static bool isMMThreeBitGPRegister(const MachineOperand &MO) {
  if (MO.isReg() && RISCV64::GPRMM16RegClass.contains(MO.getReg()))
    return true;
  return false;
}

// Returns true if the machine operand MO is register $0, $17, or $2-$7.
static bool isMMSourceRegister(const MachineOperand &MO) {
  if (MO.isReg() && RISCV64::GPRMM16ZeroRegClass.contains(MO.getReg()))
    return true;
  return false;
}

// Returns true if the operand Op is an immediate value
// and writes the immediate value into variable Imm.
static bool GetImm(MachineInstr *MI, unsigned Op, int64_t &Imm) {

  if (!MI->getOperand(Op).isImm())
    return false;
  Imm = MI->getOperand(Op).getImm();
  return true;
}

// Returns true if the value is a valid immediate for ADDIUSP.
static bool AddiuspImmValue(int64_t Value) {
  int64_t Value2 = Value >> 2;
  if (((Value & (int64_t)maskTrailingZeros<uint64_t>(2)) == Value) &&
      ((Value2 >= 2 && Value2 <= 257) || (Value2 >= -258 && Value2 <= -3)))
    return true;
  return false;
}

// Returns true if the variable Value has the number of least-significant zero
// bits equal to Shift and if the shifted value is between the bounds.
static bool InRange(int64_t Value, unsigned short Shift, int LBound,
                    int HBound) {
  int64_t Value2 = Value >> Shift;
  if (((Value & (int64_t)maskTrailingZeros<uint64_t>(Shift)) == Value) &&
      (Value2 >= LBound) && (Value2 < HBound))
    return true;
  return false;
}

// Returns true if immediate operand is in range.
static bool ImmInRange(MachineInstr *MI, const ReduceEntry &Entry) {

  int64_t offset;

  if (!GetImm(MI, Entry.ImmField(), offset))
    return false;

  if (!InRange(offset, Entry.Shift(), Entry.LBound(), Entry.HBound()))
    return false;

  return true;
}

// Returns true if MI can be reduced to lwp/swp instruction
static bool CheckXWPInstr(MachineInstr *MI, bool ReduceToLwp,
                          const ReduceEntry &Entry) {

  if (ReduceToLwp &&
      !(MI->getOpcode() == RISCV64::LW || MI->getOpcode() == RISCV64::LW_MM ||
        MI->getOpcode() == RISCV64::LW16_MM))
    return false;

  if (!ReduceToLwp &&
      !(MI->getOpcode() == RISCV64::SW || MI->getOpcode() == RISCV64::SW_MM ||
        MI->getOpcode() == RISCV64::SW16_MM))
    return false;

  unsigned reg = MI->getOperand(0).getReg();
  if (reg == RISCV64::RA)
    return false;

  if (!ImmInRange(MI, Entry))
    return false;

  if (ReduceToLwp && (MI->getOperand(0).getReg() == MI->getOperand(1).getReg()))
    return false;

  return true;
}

// Returns true if the registers Reg1 and Reg2 are consecutive
static bool ConsecutiveRegisters(unsigned Reg1, unsigned Reg2) {
  static SmallVector<unsigned, 31> Registers = {
      RISCV64::AT, RISCV64::V0, RISCV64::V1, RISCV64::A0, RISCV64::A1, RISCV64::A2, RISCV64::A3,
      RISCV64::T0, RISCV64::T1, RISCV64::T2, RISCV64::T3, RISCV64::T4, RISCV64::T5, RISCV64::T6,
      RISCV64::T7, RISCV64::S0, RISCV64::S1, RISCV64::S2, RISCV64::S3, RISCV64::S4, RISCV64::S5,
      RISCV64::S6, RISCV64::S7, RISCV64::T8, RISCV64::T9, RISCV64::K0, RISCV64::K1, RISCV64::GP,
      RISCV64::SP, RISCV64::FP, RISCV64::RA};

  for (uint8_t i = 0; i < Registers.size() - 1; i++) {
    if (Registers[i] == Reg1) {
      if (Registers[i + 1] == Reg2)
        return true;
      else
        return false;
    }
  }
  return false;
}

// Returns true if registers and offsets are consecutive
static bool ConsecutiveInstr(MachineInstr *MI1, MachineInstr *MI2) {

  int64_t Offset1, Offset2;
  if (!GetImm(MI1, 2, Offset1))
    return false;
  if (!GetImm(MI2, 2, Offset2))
    return false;

  unsigned Reg1 = MI1->getOperand(0).getReg();
  unsigned Reg2 = MI2->getOperand(0).getReg();

  return ((Offset1 == (Offset2 - 4)) && (ConsecutiveRegisters(Reg1, Reg2)));
}

MicroRISCV64SizeReduce::MicroRISCV64SizeReduce() : MachineFunctionPass(ID) {}

bool MicroRISCV64SizeReduce::ReduceMI(const MachineBasicBlock::instr_iterator &MII,
                                   MachineBasicBlock::instr_iterator &NextMII) {

  MachineInstr *MI = &*MII;
  unsigned Opcode = MI->getOpcode();

  // Search the table.
  ReduceEntryVector::const_iterator Start = std::begin(ReduceTable);
  ReduceEntryVector::const_iterator End = std::end(ReduceTable);

  std::pair<ReduceEntryVector::const_iterator,
            ReduceEntryVector::const_iterator>
      Range = std::equal_range(Start, End, Opcode);

  if (Range.first == Range.second)
    return false;

  for (ReduceEntryVector::const_iterator Entry = Range.first;
       Entry != Range.second; ++Entry) {
    ReduceEntryFunArgs Arguments(&(*MII), *Entry, NextMII);
    if (((*Entry).ReduceFunction)(&Arguments))
      return true;
  }
  return false;
}

bool MicroRISCV64SizeReduce::ReduceXWtoXWSP(ReduceEntryFunArgs *Arguments) {

  MachineInstr *MI = Arguments->MI;
  const ReduceEntry &Entry = Arguments->Entry;

  if (!ImmInRange(MI, Entry))
    return false;

  if (!IsSP(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroRISCV64SizeReduce::ReduceXWtoXWP(ReduceEntryFunArgs *Arguments) {

  const ReduceEntry &Entry = Arguments->Entry;
  MachineBasicBlock::instr_iterator &NextMII = Arguments->NextMII;
  const MachineBasicBlock::instr_iterator &E =
      Arguments->MI->getParent()->instr_end();

  if (NextMII == E)
    return false;

  MachineInstr *MI1 = Arguments->MI;
  MachineInstr *MI2 = &*NextMII;

  // ReduceToLwp = true/false - reduce to LWP/SWP instruction
  bool ReduceToLwp = (MI1->getOpcode() == RISCV64::LW) ||
                     (MI1->getOpcode() == RISCV64::LW_MM) ||
                     (MI1->getOpcode() == RISCV64::LW16_MM);

  if (!CheckXWPInstr(MI1, ReduceToLwp, Entry))
    return false;

  if (!CheckXWPInstr(MI2, ReduceToLwp, Entry))
    return false;

  unsigned Reg1 = MI1->getOperand(1).getReg();
  unsigned Reg2 = MI2->getOperand(1).getReg();

  if (Reg1 != Reg2)
    return false;

  bool ConsecutiveForward = ConsecutiveInstr(MI1, MI2);
  bool ConsecutiveBackward = ConsecutiveInstr(MI2, MI1);

  if (!(ConsecutiveForward || ConsecutiveBackward))
    return false;

  NextMII = std::next(NextMII);
  return ReplaceInstruction(MI1, Entry, MI2, ConsecutiveForward);
}

bool MicroRISCV64SizeReduce::ReduceArithmeticInstructions(
    ReduceEntryFunArgs *Arguments) {

  MachineInstr *MI = Arguments->MI;
  const ReduceEntry &Entry = Arguments->Entry;

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)) ||
      !isMMThreeBitGPRegister(MI->getOperand(2)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroRISCV64SizeReduce::ReduceADDIUToADDIUR1SP(
    ReduceEntryFunArgs *Arguments) {

  MachineInstr *MI = Arguments->MI;
  const ReduceEntry &Entry = Arguments->Entry;

  if (!ImmInRange(MI, Entry))
    return false;

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) || !IsSP(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroRISCV64SizeReduce::ReduceADDIUToADDIUSP(ReduceEntryFunArgs *Arguments) {

  MachineInstr *MI = Arguments->MI;
  const ReduceEntry &Entry = Arguments->Entry;

  int64_t ImmValue;
  if (!GetImm(MI, Entry.ImmField(), ImmValue))
    return false;

  if (!AddiuspImmValue(ImmValue))
    return false;

  if (!IsSP(MI->getOperand(0)) || !IsSP(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroRISCV64SizeReduce::ReduceLXUtoLXU16(ReduceEntryFunArgs *Arguments) {

  MachineInstr *MI = Arguments->MI;
  const ReduceEntry &Entry = Arguments->Entry;

  if (!ImmInRange(MI, Entry))
    return false;

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroRISCV64SizeReduce::ReduceSXtoSX16(ReduceEntryFunArgs *Arguments) {

  MachineInstr *MI = Arguments->MI;
  const ReduceEntry &Entry = Arguments->Entry;

  if (!ImmInRange(MI, Entry))
    return false;

  if (!isMMSourceRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

// Returns true if Reg can be a source register
// of MOVEP instruction
static bool IsMovepSrcRegister(unsigned Reg) {

  if (Reg == RISCV64::ZERO || Reg == RISCV64::V0 || Reg == RISCV64::V1 ||
      Reg == RISCV64::S0 || Reg == RISCV64::S1 || Reg == RISCV64::S2 ||
      Reg == RISCV64::S3 || Reg == RISCV64::S4)
    return true;

  return false;
}

// Returns true if Reg can be a destination register
// of MOVEP instruction
static bool IsMovepDestinationReg(unsigned Reg) {

  if (Reg == RISCV64::A0 || Reg == RISCV64::A1 || Reg == RISCV64::A2 ||
      Reg == RISCV64::A3 || Reg == RISCV64::S5 || Reg == RISCV64::S6)
    return true;

  return false;
}

// Returns true if the registers can be a pair of destination
// registers in MOVEP instruction
static bool IsMovepDestinationRegPair(unsigned R0, unsigned R1) {

  if ((R0 == RISCV64::A0 && R1 == RISCV64::S5) ||
      (R0 == RISCV64::A0 && R1 == RISCV64::S6) ||
      (R0 == RISCV64::A0 && R1 == RISCV64::A1) ||
      (R0 == RISCV64::A0 && R1 == RISCV64::A2) ||
      (R0 == RISCV64::A0 && R1 == RISCV64::A3) ||
      (R0 == RISCV64::A1 && R1 == RISCV64::A2) ||
      (R0 == RISCV64::A1 && R1 == RISCV64::A3) ||
      (R0 == RISCV64::A2 && R1 == RISCV64::A3))
    return true;

  return false;
}

bool MicroRISCV64SizeReduce::ReduceMoveToMovep(ReduceEntryFunArgs *Arguments) {

  const ReduceEntry &Entry = Arguments->Entry;
  MachineBasicBlock::instr_iterator &NextMII = Arguments->NextMII;
  const MachineBasicBlock::instr_iterator &E =
      Arguments->MI->getParent()->instr_end();

  if (NextMII == E)
    return false;

  MachineInstr *MI1 = Arguments->MI;
  MachineInstr *MI2 = &*NextMII;

  unsigned RegDstMI1 = MI1->getOperand(0).getReg();
  unsigned RegSrcMI1 = MI1->getOperand(1).getReg();

  if (!IsMovepSrcRegister(RegSrcMI1))
    return false;

  if (!IsMovepDestinationReg(RegDstMI1))
    return false;

  if (MI2->getOpcode() != Entry.WideOpc())
    return false;

  unsigned RegDstMI2 = MI2->getOperand(0).getReg();
  unsigned RegSrcMI2 = MI2->getOperand(1).getReg();

  if (!IsMovepSrcRegister(RegSrcMI2))
    return false;

  bool ConsecutiveForward;
  if (IsMovepDestinationRegPair(RegDstMI1, RegDstMI2)) {
    ConsecutiveForward = true;
  } else if (IsMovepDestinationRegPair(RegDstMI2, RegDstMI1)) {
    ConsecutiveForward = false;
  } else
    return false;

  NextMII = std::next(NextMII);
  return ReplaceInstruction(MI1, Entry, MI2, ConsecutiveForward);
}

bool MicroRISCV64SizeReduce::ReduceXORtoXOR16(ReduceEntryFunArgs *Arguments) {

  MachineInstr *MI = Arguments->MI;
  const ReduceEntry &Entry = Arguments->Entry;

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)) ||
      !isMMThreeBitGPRegister(MI->getOperand(2)))
    return false;

  if (!(MI->getOperand(0).getReg() == MI->getOperand(2).getReg()) &&
      !(MI->getOperand(0).getReg() == MI->getOperand(1).getReg()))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroRISCV64SizeReduce::ReduceMBB(MachineBasicBlock &MBB) {
  bool Modified = false;
  MachineBasicBlock::instr_iterator MII = MBB.instr_begin(),
                                    E = MBB.instr_end();
  MachineBasicBlock::instr_iterator NextMII;

  // Iterate through the instructions in the basic block
  for (; MII != E; MII = NextMII) {
    NextMII = std::next(MII);
    MachineInstr *MI = &*MII;

    // Don't reduce bundled instructions or pseudo operations
    if (MI->isBundle() || MI->isTransient())
      continue;

    // Try to reduce 32-bit instruction into 16-bit instruction
    Modified |= ReduceMI(MII, NextMII);
  }

  return Modified;
}

bool MicroRISCV64SizeReduce::ReplaceInstruction(MachineInstr *MI,
                                             const ReduceEntry &Entry,
                                             MachineInstr *MI2,
                                             bool ConsecutiveForward) {

  enum OperandTransfer OpTransfer = Entry.TransferOperands();

  LLVM_DEBUG(dbgs() << "Converting 32-bit: " << *MI);
  ++NumReduced;

  if (OpTransfer == OT_OperandsAll) {
    MI->setDesc(RISCV64II->get(Entry.NarrowOpc()));
    LLVM_DEBUG(dbgs() << "       to 16-bit: " << *MI);
    return true;
  } else {
    MachineBasicBlock &MBB = *MI->getParent();
    const MCInstrDesc &NewMCID = RISCV64II->get(Entry.NarrowOpc());
    DebugLoc dl = MI->getDebugLoc();
    MachineInstrBuilder MIB = BuildMI(MBB, MI, dl, NewMCID);
    switch (OpTransfer) {
    case OT_Operand2:
      MIB.add(MI->getOperand(2));
      break;
    case OT_Operands02: {
      MIB.add(MI->getOperand(0));
      MIB.add(MI->getOperand(2));
      break;
    }
    case OT_OperandsXOR: {
      if (MI->getOperand(0).getReg() == MI->getOperand(2).getReg()) {
        MIB.add(MI->getOperand(0));
        MIB.add(MI->getOperand(1));
        MIB.add(MI->getOperand(2));
      } else {
        MIB.add(MI->getOperand(0));
        MIB.add(MI->getOperand(2));
        MIB.add(MI->getOperand(1));
      }
      break;
    }
    case OT_OperandsMovep:
    case OT_OperandsLwp:
    case OT_OperandsSwp: {
      if (ConsecutiveForward) {
        MIB.add(MI->getOperand(0));
        MIB.add(MI2->getOperand(0));
        MIB.add(MI->getOperand(1));
        if (OpTransfer == OT_OperandsMovep)
          MIB.add(MI2->getOperand(1));
        else
          MIB.add(MI->getOperand(2));
      } else { // consecutive backward
        MIB.add(MI2->getOperand(0));
        MIB.add(MI->getOperand(0));
        MIB.add(MI2->getOperand(1));
        if (OpTransfer == OT_OperandsMovep)
          MIB.add(MI->getOperand(1));
        else
          MIB.add(MI2->getOperand(2));
      }

      LLVM_DEBUG(dbgs() << "and converting 32-bit: " << *MI2
                        << "       to: " << *MIB);

      MBB.erase_instr(MI);
      MBB.erase_instr(MI2);
      return true;
    }
    default:
      llvm_unreachable("Unknown operand transfer!");
    }

    // Transfer MI flags.
    MIB.setMIFlags(MI->getFlags());

    LLVM_DEBUG(dbgs() << "       to 16-bit: " << *MIB);
    MBB.erase_instr(MI);
    return true;
  }
  return false;
}

bool MicroRISCV64SizeReduce::runOnMachineFunction(MachineFunction &MF) {

  Subtarget = &static_cast<const RISCV64Subtarget &>(MF.getSubtarget());

  // TODO: Add support for the subtarget microMIPS32R6.
  if (!Subtarget->inMicroRISCV64Mode() || !Subtarget->hasRISCV6432r2() ||
      Subtarget->hasRISCV6432r6())
    return false;

  RISCV64II = static_cast<const RISCV64InstrInfo *>(Subtarget->getInstrInfo());

  bool Modified = false;
  MachineFunction::iterator I = MF.begin(), E = MF.end();

  for (; I != E; ++I)
    Modified |= ReduceMBB(*I);
  return Modified;
}

/// Returns an instance of the MicroRISCV64 size reduction pass.
FunctionPass *llvm::createMicroRISCV64SizeReducePass() {
  return new MicroRISCV64SizeReduce();
}
