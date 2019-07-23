//===-- MYRISCVXAsmParser.cpp - Parse MYRISCVX assembly to MCInst instructions ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MYRISCVX.h"

#include "MCTargetDesc/MYRISCVXMCExpr.h"
#include "MCTargetDesc/MYRISCVXMCTargetDesc.h"
#include "MYRISCVXRegisterInfo.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "MYRISCVX-asm-parser"

namespace {
class MYRISCVXAssemblerOptions {
 public:
  MYRISCVXAssemblerOptions():
      reorder(true), macro(true) {
  }

  bool isReorder() {return reorder;}
  void setReorder() {reorder = true;}
  void setNoreorder() {reorder = false;}

  bool isMacro() {return macro;}
  void setMacro() {macro = true;}
  void setNomacro() {macro = false;}

 private:
  bool reorder;
  bool macro;
};
}

namespace {
class MYRISCVXAsmParser : public MCTargetAsmParser {
  MCAsmParser &Parser;
  MYRISCVXAssemblerOptions Options;

#define GET_ASSEMBLER_HEADER
#include "MYRISCVXGenAsmMatcher.inc"

  SMLoc getLoc() const { return getParser().getTok().getLoc(); }

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                     SMLoc &EndLoc) override;
  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool parseMathOperation(StringRef Name, SMLoc NameLoc,
                          OperandVector &Operands);

  bool ParseDirective(AsmToken DirectiveID) override;

  OperandMatchResultTy parseMemOperand(OperandVector &);

  bool ParseOperand(OperandVector &Operands, StringRef Mnemonic);

  bool needsExpansion(MCInst &Inst);

  void expandInstruction(MCInst &Inst, SMLoc IDLoc,
                         SmallVectorImpl<MCInst> &Instructions);
  void expandLoadImm(MCInst &Inst, SMLoc IDLoc,
                     SmallVectorImpl<MCInst> &Instructions);
  void expandLoadAddressImm(MCInst &Inst, SMLoc IDLoc,
                            SmallVectorImpl<MCInst> &Instructions);
  void expandLoadAddressReg(MCInst &Inst, SMLoc IDLoc,
                            SmallVectorImpl<MCInst> &Instructions);
  bool reportParseError(StringRef ErrorMsg);

  bool parseMemOffset(const MCExpr *&Res);
  bool parseRelocOperand(const MCExpr *&Res);

  const MCExpr *evaluateRelocExpr(const MCExpr *Expr, StringRef RelocStr);

  bool parseDirectiveSet();

  OperandMatchResultTy parseImmediate(OperandVector &Operands);
  OperandMatchResultTy parseRegister(OperandVector &Operands);

  bool parseSetAtDirective();
  bool parseSetNoAtDirective();
  bool parseSetMacroDirective();
  bool parseSetNoMacroDirective();
  bool parseSetReorderDirective();
  bool parseSetNoReorderDirective();

  unsigned MatchRegisterAltName(StringRef Symbol);

  unsigned getReg(int RC,int RegNo);

 public:
  MYRISCVXAsmParser(const MCSubtargetInfo &STI, MCAsmParser &parser,
                    const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII), Parser(parser) {
    // Initialize the set of available features.
    setAvailableFeatures(ComputeAvailableFeatures(getSTI().getFeatureBits()));
  }

  MCAsmParser &getParser() const { return Parser; }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }

};
}

namespace {

/// MYRISCVXOperand - Instances of this class represent a parsed MYRISCVX machine
/// instruction.
class MYRISCVXOperand : public MCParsedAsmOperand {

  enum KindTy {
    k_Immediate,
    k_Memory,
    k_Register,
    k_Token
  } Kind;

 public:
  MYRISCVXOperand(KindTy K) : MCParsedAsmOperand(), Kind(K) {}

  struct Token {
    const char *Data;
    unsigned Length;
  };
  struct PhysRegOp {
    unsigned RegNum; /// Register Number
  };
  struct ImmOp {
    const MCExpr *Val;
  };
  struct MemOp {
    unsigned Base;
    const MCExpr *Off;
  };

  union {
    struct Token Tok;
    struct PhysRegOp Reg;
    struct ImmOp Imm;
    struct MemOp Mem;
  };

  SMLoc StartLoc, EndLoc;

 public:
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const{
    // Add as immediate when possible.  Null MCExpr = 0.
    if (Expr == 0)
      Inst.addOperand(MCOperand::createImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    const MCExpr *Expr = getImm();
    addExpr(Inst,Expr);
  }

  void addMemOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getMemBase()));

    const MCExpr *Expr = getMemOff();
    addExpr(Inst,Expr);
  }

  bool isReg() const override { return Kind == k_Register; }
  bool isImm() const override { return Kind == k_Immediate; }
  bool isToken() const override { return Kind == k_Token; }
  bool isMem() const override { return Kind == k_Memory; }

  StringRef getToken() const {
    assert(Kind == k_Token && "Invalid access!");
    return StringRef(Tok.Data, Tok.Length);
  }

  unsigned getReg() const override {
    assert((Kind == k_Register) && "Invalid access!");
    return Reg.RegNum;
  }

  const MCExpr *getImm() const {
    assert((Kind == k_Immediate) && "Invalid access!");
    return Imm.Val;
  }

  unsigned getMemBase() const {
    assert((Kind == k_Memory) && "Invalid access!");
    return Mem.Base;
  }

  const MCExpr *getMemOff() const {
    assert((Kind == k_Memory) && "Invalid access!");
    return Mem.Off;
  }

  static std::unique_ptr<MYRISCVXOperand> CreateToken(StringRef Str, SMLoc S) {
    auto Op = make_unique<MYRISCVXOperand>(k_Token);
    Op->Tok.Data = Str.data();
    Op->Tok.Length = Str.size();
    Op->StartLoc = S;
    Op->EndLoc = S;
    return Op;
  }

  /// Internal constructor for register kinds
  static std::unique_ptr<MYRISCVXOperand> createReg(unsigned RegNum, SMLoc S,
                                                    SMLoc E) {
    auto Op = make_unique<MYRISCVXOperand>(k_Register);
    Op->Reg.RegNum = RegNum;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<MYRISCVXOperand> createImm(const MCExpr *Val, SMLoc S, SMLoc E) {
    auto Op = make_unique<MYRISCVXOperand>(k_Immediate);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<MYRISCVXOperand> CreateMem(unsigned Base, const MCExpr *Off,
                                                    SMLoc S, SMLoc E) {
    auto Op = make_unique<MYRISCVXOperand>(k_Memory);
    Op->Mem.Base = Base;
    Op->Mem.Off = Off;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  /// getStartLoc - Get the location of the first token of this operand.
  SMLoc getStartLoc() const override { return StartLoc; }
  /// getEndLoc - Get the location of the last token of this operand.
  SMLoc getEndLoc() const override { return EndLoc; }

  void print(raw_ostream &OS) const override {
    switch (Kind) {
      case k_Immediate:
        OS << "Imm<";
        OS << *Imm.Val;
        OS << ">";
        break;
      case k_Memory:
        OS << "Mem<";
        OS << Mem.Base;
        OS << ", ";
        OS << *Mem.Off;
        OS << ">";
        break;
      case k_Register:
        OS << "Register<" << Reg.RegNum << ">";
        break;
      case k_Token:
        OS << Tok.Data;
        break;
    }
  }
};
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "MYRISCVXGenAsmMatcher.inc"

void printMYRISCVXOperands(OperandVector &Operands) {
  for (size_t i = 0; i < Operands.size(); i++) {
    MYRISCVXOperand* op = static_cast<MYRISCVXOperand*>(&*Operands[i]);
    assert(op != nullptr);
    LLVM_DEBUG(dbgs() << "<" << *op << ">");
  }
  LLVM_DEBUG(dbgs() << "\n");
}

//@1 {
bool MYRISCVXAsmParser::needsExpansion(MCInst &Inst) {

  switch(Inst.getOpcode()) {
    case MYRISCVX::LoadImm32Reg:
    case MYRISCVX::LoadAddr32Imm:
      // case MYRISCVX::LoadAddr32Reg:
      return true;
    default:
      return false;
  }
}

void MYRISCVXAsmParser::expandInstruction(MCInst &Inst, SMLoc IDLoc,
                                          SmallVectorImpl<MCInst> &Instructions){
  switch(Inst.getOpcode()) {
    case MYRISCVX::LoadImm32Reg:
      return expandLoadImm(Inst, IDLoc, Instructions);
    case MYRISCVX::LoadAddr32Imm:
      return expandLoadAddressImm(Inst,IDLoc,Instructions);
    // case MYRISCVX::LoadAddr32Reg:
    //   return expandLoadAddressReg(Inst,IDLoc,Instructions);
  }
}
//@1 }

void MYRISCVXAsmParser::expandLoadImm(MCInst &Inst, SMLoc IDLoc,
                                      SmallVectorImpl<MCInst> &Instructions){
  MCInst tmpInst;
  const MCOperand &ImmOp = Inst.getOperand(1);
  assert(ImmOp.isImm() && "expected immediate operand kind");
  const MCOperand &RegOp = Inst.getOperand(0);
  assert(RegOp.isReg() && "expected register operand kind");

  int ImmValue = ImmOp.getImm();
  tmpInst.setLoc(IDLoc);
  if ( 0 <= ImmValue && ImmValue <= 65535) {
    // for 0 <= j <= 65535.
    // li d,j => ori d,$zero,j
    tmpInst.setOpcode(MYRISCVX::ORI);
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(
        MCOperand::createReg(MYRISCVX::ZERO));
    tmpInst.addOperand(MCOperand::createImm(ImmValue));
    Instructions.push_back(tmpInst);
  } else if ( ImmValue < 0 && ImmValue >= -32768) {
    // for -32768 <= j < 0.
    // li d,j => addiu d,$zero,j
    tmpInst.setOpcode(MYRISCVX::ADDI); //TODO:no ADDiu64 in td files?
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(
        MCOperand::createReg(MYRISCVX::ZERO));
    tmpInst.addOperand(MCOperand::createImm(ImmValue));
    Instructions.push_back(tmpInst);
  } else {
    // for any other value of j that is representable as a 32-bit integer.
    // li d,j => lui d,hi16(j)
    //           ori d,d,lo16(j)
    tmpInst.setOpcode(MYRISCVX::LUI);
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::createImm((ImmValue & 0xffff0000) >> 16));
    Instructions.push_back(tmpInst);
    tmpInst.clear();
    tmpInst.setOpcode(MYRISCVX::ORI);
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::createImm(ImmValue & 0xffff));
    tmpInst.setLoc(IDLoc);
    Instructions.push_back(tmpInst);
  }
}

void MYRISCVXAsmParser::expandLoadAddressReg(MCInst &Inst, SMLoc IDLoc,
                                             SmallVectorImpl<MCInst> &Instructions){
  MCInst tmpInst;
  const MCOperand &ImmOp = Inst.getOperand(2);
  assert(ImmOp.isImm() && "expected immediate operand kind");
  const MCOperand &SrcRegOp = Inst.getOperand(1);
  assert(SrcRegOp.isReg() && "expected register operand kind");
  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");
  int ImmValue = ImmOp.getImm();
  if ( -32768 <= ImmValue && ImmValue <= 32767) {
    // for -32768 <= j < 32767.
    //la d,j(s) => addiu d,s,j
    tmpInst.setOpcode(MYRISCVX::ADDI); //TODO:no ADDiu64 in td files?
    tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
    tmpInst.addOperand(MCOperand::createReg(SrcRegOp.getReg()));
    tmpInst.addOperand(MCOperand::createImm(ImmValue));
    Instructions.push_back(tmpInst);
  } else {
    // for any other value of j that is representable as a 32-bit integer.
    // la d,j(s) => lui d,hi16(j)
    //              ori d,d,lo16(j)
    //              add d,d,s
    tmpInst.setOpcode(MYRISCVX::LUI);
    tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
    tmpInst.addOperand(MCOperand::createImm((ImmValue & 0xffff0000) >> 16));
    Instructions.push_back(tmpInst);
    tmpInst.clear();
    tmpInst.setOpcode(MYRISCVX::ORI);
    tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
    tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
    tmpInst.addOperand(MCOperand::createImm(ImmValue & 0xffff));
    Instructions.push_back(tmpInst);
    tmpInst.clear();
    tmpInst.setOpcode(MYRISCVX::ADD);
    tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
    tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
    tmpInst.addOperand(MCOperand::createReg(SrcRegOp.getReg()));
    Instructions.push_back(tmpInst);
  }
}

void MYRISCVXAsmParser::expandLoadAddressImm(MCInst &Inst, SMLoc IDLoc,
                                             SmallVectorImpl<MCInst> &Instructions){
  MCInst tmpInst;
  const MCOperand &ImmOp = Inst.getOperand(1);
  assert(ImmOp.isImm() && "expected immediate operand kind");
  const MCOperand &RegOp = Inst.getOperand(0);
  assert(RegOp.isReg() && "expected register operand kind");
  int ImmValue = ImmOp.getImm();
  if ( -32768 <= ImmValue && ImmValue <= 32767) {
    // for -32768 <= j < 32767.
    //la d,j => addiu d,$zero,j
    tmpInst.setOpcode(MYRISCVX::ADDI);
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(
        MCOperand::createReg(MYRISCVX::ZERO));
    tmpInst.addOperand(MCOperand::createImm(ImmValue));
    Instructions.push_back(tmpInst);
  } else {
    // for any other value of j that is representable as a 32-bit integer.
    // la d,j => lui d,hi16(j)
    //           ori d,d,lo16(j)
    tmpInst.setOpcode(MYRISCVX::LUI);
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::createImm((ImmValue & 0xffff0000) >> 16));
    Instructions.push_back(tmpInst);
    tmpInst.clear();
    tmpInst.setOpcode(MYRISCVX::ORI);
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::createImm(ImmValue & 0xffff));
    Instructions.push_back(tmpInst);
  }
}

//@2 {
bool MYRISCVXAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                                OperandVector &Operands,
                                                MCStreamer &Out,
                                                uint64_t &ErrorInfo,
                                                bool MatchingInlineAsm) {
  printMYRISCVXOperands(Operands);
  MCInst Inst;
  unsigned MatchResult = MatchInstructionImpl(Operands, Inst, ErrorInfo,
                                              MatchingInlineAsm);
  switch (MatchResult) {
    default: break;
    case Match_Success: {
      if (needsExpansion(Inst)) {
        SmallVector<MCInst, 4> Instructions;
        expandInstruction(Inst, IDLoc, Instructions);
        for(unsigned i =0; i < Instructions.size(); i++){
          Out.EmitInstruction(Instructions[i], getSTI());
        }
      } else {
        Inst.setLoc(IDLoc);
        Out.EmitInstruction(Inst, getSTI());
      }
      return false;
    }
      //@2 }
    case Match_MissingFeature:
      Error(IDLoc, "instruction requires a CPU feature not currently enabled");
      return true;
    case Match_InvalidOperand: {
      SMLoc ErrorLoc = IDLoc;
      if (ErrorInfo != ~0U) {
        if (ErrorInfo >= Operands.size())
          return Error(IDLoc, "too few operands for instruction");

        ErrorLoc = ((MYRISCVXOperand &)*Operands[ErrorInfo]).getStartLoc();
        if (ErrorLoc == SMLoc()) ErrorLoc = IDLoc;
      }

      return Error(ErrorLoc, "invalid operand for instruction");
    }
    case Match_MnemonicFail:
      return Error(IDLoc, "invalid instruction");
  }
  return true;
}

unsigned MYRISCVXAsmParser::MatchRegisterAltName(StringRef Name) {

  int CC;
  CC = StringSwitch<unsigned>(Name)
      .Case("zero",  MYRISCVX::ZERO)
      .Case("ra",  MYRISCVX::RA)
      .Case("sp",  MYRISCVX::SP)
      .Case("gp",  MYRISCVX::GP)
      .Case("tp",  MYRISCVX::TP)
      .Case("t0",  MYRISCVX::T0)
      .Case("t1",  MYRISCVX::T1)
      .Case("t2",  MYRISCVX::T2)
      .Case("s0",  MYRISCVX::FP)
      .Case("s1",  MYRISCVX::S1)
      .Case("a0",  MYRISCVX::A0)
      .Case("a1",  MYRISCVX::A1)
      .Case("a2",  MYRISCVX::A2)
      .Case("a3",  MYRISCVX::A3)
      .Case("a4",  MYRISCVX::A4)
      .Case("a5",  MYRISCVX::A5)
      .Case("a6",  MYRISCVX::A6)
      .Case("a7",  MYRISCVX::A7)
      .Case("s2",  MYRISCVX::S2)
      .Case("s3",  MYRISCVX::S3)
      .Case("s4",  MYRISCVX::S4)
      .Case("s5",  MYRISCVX::S5)
      .Case("s6",  MYRISCVX::S6)
      .Case("s7",  MYRISCVX::S7)
      .Case("s8",  MYRISCVX::S8)
      .Case("s9",  MYRISCVX::S9)
      .Case("s10", MYRISCVX::S10)
      .Case("s11", MYRISCVX::S11)
      .Case("t3",  MYRISCVX::T3)
      .Case("t4",  MYRISCVX::T4)
      .Case("t5",  MYRISCVX::T5)
      .Case("t6",  MYRISCVX::T6)
      .Default(0);

  if (CC != 0)
    return CC;

  return 0;
}

unsigned MYRISCVXAsmParser::getReg(int RC,int RegNo) {
  return *(getContext().getRegisterInfo()->getRegClass(RC).begin() + RegNo);
}



bool MYRISCVXAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                      SMLoc &EndLoc) {
  const AsmToken &Tok = getParser().getTok();
  StartLoc = Tok.getLoc();
  EndLoc = Tok.getEndLoc();
  RegNo = 0;
  StringRef Name = getLexer().getTok().getIdentifier();

  if (!MatchRegisterName(Name)) {
    getParser().Lex(); // Eat identifier token.n
    return false;
  }

  return Error(StartLoc, "invalid register name");
}


bool MYRISCVXAsmParser::ParseOperand(OperandVector &Operands,
                                     StringRef Mnemonic) {
  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  // OperandMatchResultTy ResTy = MatchOperandParserImpl(Operands, Mnemonic);
  // if (ResTy == MatchOperand_Success)
  //   return false;

  // If there wasn't a custom match, try the generic matcher below. Otherwise,
  // there was a match, but an error occurred, in which case, just return that
  // the operand parsing failed.
  // if (ResTy == MatchOperand_ParseFail)
  //   return true;

  LLVM_DEBUG(dbgs() << ".. Generic Parser\n");

  // Attempt to parse token as a register.
  if (parseRegister(Operands) == MatchOperand_Success)
    return false;

  // Attempt to parse token as an immediate
  if (parseImmediate(Operands) == MatchOperand_Success) {
    // Parse memory base register if present
    if (getLexer().is(AsmToken::LParen)) {
      return parseMemOperand(Operands) != MatchOperand_Success;
    }
    return false;
  }

  LLVM_DEBUG(dbgs() << "unknown operand\n");
  return true;
}

const MCExpr *MYRISCVXAsmParser::evaluateRelocExpr(const MCExpr *Expr,
                                                   StringRef RelocStr) {
  MYRISCVXMCExpr::MYRISCVXExprKind Kind =
      StringSwitch<MYRISCVXMCExpr::MYRISCVXExprKind>(RelocStr)
      .Case("call16", MYRISCVXMCExpr::CEK_GOT_CALL)
      .Case("call_hi", MYRISCVXMCExpr::CEK_CALL_HI16)
      .Case("call_lo", MYRISCVXMCExpr::CEK_CALL_LO16)
      .Case("dtp_hi", MYRISCVXMCExpr::CEK_DTP_HI)
      .Case("dtp_lo", MYRISCVXMCExpr::CEK_DTP_LO)
      .Case("got", MYRISCVXMCExpr::CEK_GOT)
      .Case("got_hi", MYRISCVXMCExpr::CEK_GOT_HI20)
      .Case("got_lo", MYRISCVXMCExpr::CEK_GOT_LO12)
      .Case("gottprel", MYRISCVXMCExpr::CEK_GOTTPREL)
      .Case("gp_rel", MYRISCVXMCExpr::CEK_GPREL)
      .Case("hi", MYRISCVXMCExpr::CEK_ABS_HI)
      .Case("lo", MYRISCVXMCExpr::CEK_ABS_LO)
      .Case("tlsgd", MYRISCVXMCExpr::CEK_TLSGD)
      .Case("tlsldm", MYRISCVXMCExpr::CEK_TLSLDM)
      .Case("tp_hi", MYRISCVXMCExpr::CEK_TP_HI)
      .Case("tp_lo", MYRISCVXMCExpr::CEK_TP_LO)
      .Default(MYRISCVXMCExpr::CEK_None);

  assert(Kind != MYRISCVXMCExpr::CEK_None);
  return MYRISCVXMCExpr::create(Kind, Expr, getContext());
}

bool MYRISCVXAsmParser::parseRelocOperand(const MCExpr *&Res) {

  Parser.Lex(); // eat % token
  const AsmToken &Tok = Parser.getTok(); // get next token, operation
  if (Tok.isNot(AsmToken::Identifier))
    return true;

  std::string Str = Tok.getIdentifier().str();

  Parser.Lex(); // eat identifier
  // now make expression from the rest of the operand
  const MCExpr *IdVal;
  SMLoc EndLoc;

  if (getLexer().getKind() == AsmToken::LParen) {
    while (1) {
      Parser.Lex(); // eat '(' token
      if (getLexer().getKind() == AsmToken::Percent) {
        Parser.Lex(); // eat % token
        const AsmToken &nextTok = Parser.getTok();
        if (nextTok.isNot(AsmToken::Identifier))
          return true;
        Str += "(%";
        Str += nextTok.getIdentifier();
        Parser.Lex(); // eat identifier
        if (getLexer().getKind() != AsmToken::LParen)
          return true;
      } else
        break;
    }
    if (getParser().parseParenExpression(IdVal,EndLoc))
      return true;

    while (getLexer().getKind() == AsmToken::RParen)
      Parser.Lex(); // eat ')' token

  } else
    return true; // parenthesis must follow reloc operand

  Res = evaluateRelocExpr(IdVal, Str);
  return false;
}

OperandMatchResultTy MYRISCVXAsmParser::parseRegister(OperandVector &Operands)
{
  switch (getLexer().getKind()) {
    default:
      return MatchOperand_NoMatch;
    case AsmToken::Identifier:
      StringRef Name = getLexer().getTok().getIdentifier();
      unsigned RegNo = MatchRegisterName(Name);
      if (RegNo == 0) {
        RegNo = MatchRegisterAltName(Name);
        if (RegNo == 0) {
          return MatchOperand_NoMatch;
        }
      }
      SMLoc S = getLoc();
      SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
      getLexer().Lex();
      Operands.push_back(MYRISCVXOperand::createReg(RegNo, S, E));
  }

  return MatchOperand_Success;
}


OperandMatchResultTy MYRISCVXAsmParser::parseImmediate(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Integer:
  case AsmToken::String:
  case AsmToken::Identifier:
    if (getParser().parseExpression(Res))
      return MatchOperand_ParseFail;
    break;
  // case AsmToken::Percent:
  //   return parseOperandWithModifier(Operands);
  }

  Operands.push_back(MYRISCVXOperand::createImm(Res, S, E));
  return MatchOperand_Success;
}


// OperandMatchResultTy
// MYRISCVXAsmParser::parseOperandWithModifier(OperandVector &Operands) {
//   SMLoc S = getLoc();
//   SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
//
//   if (getLexer().getKind() != AsmToken::Percent) {
//     Error(getLoc(), "expected '%' for operand modifier");
//     return MatchOperand_ParseFail;
//   }
//
//   getParser().Lex(); // Eat '%'
//
//   if (getLexer().getKind() != AsmToken::Identifier) {
//     Error(getLoc(), "expected valid identifier for operand modifier");
//     return MatchOperand_ParseFail;
//   }
//   StringRef Identifier = getParser().getTok().getIdentifier();
//   MYRISCVXMCExpr::VariantKind VK = MYRISCVXMCExpr::getVariantKindForName(Identifier);
//   if (VK == MYRISCVXMCExpr::VK_MYRISCVX_Invalid) {
//     Error(getLoc(), "unrecognized operand modifier");
//     return MatchOperand_ParseFail;
//   }
//
//   getParser().Lex(); // Eat the identifier
//   if (getLexer().getKind() != AsmToken::LParen) {
//     Error(getLoc(), "expected '('");
//     return MatchOperand_ParseFail;
//   }
//   getParser().Lex(); // Eat '('
//
//   const MCExpr *SubExpr;
//   if (getParser().parseParenExpression(SubExpr, E)) {
//     return MatchOperand_ParseFail;
//   }
//
//   const MCExpr *ModExpr = MYRISCVXMCExpr::create(SubExpr, VK, getContext());
//   Operands.push_back(MYRISCVXOperand::createImm(ModExpr, S, E, isRV64()));
//   return MatchOperand_Success;
// }


bool MYRISCVXAsmParser::parseMemOffset(const MCExpr *&Res) {

  switch (getLexer().getKind()) {
    default:
      return true;
    case AsmToken::Integer:
    case AsmToken::Minus:
    case AsmToken::Plus:
      return (getParser().parseExpression(Res));
    case AsmToken::Percent:
      return parseRelocOperand(Res);
    case AsmToken::LParen:
      return false;  // it's probably assuming 0
  }
  return true;
}


OperandMatchResultTy MYRISCVXAsmParser::parseMemOperand(OperandVector &Operands) {

  const MCExpr *IdVal = 0;
  SMLoc S;
  // first operand is the offset
  S = Parser.getTok().getLoc();

  if (parseMemOffset(IdVal))
    return MatchOperand_ParseFail;

  const AsmToken &Tok = Parser.getTok(); // get next token
  if (Tok.isNot(AsmToken::LParen)) {
    Error(Parser.getTok().getLoc(), "'(' expected");
    return MatchOperand_ParseFail;
  }

  Parser.Lex(); // Eat '(' token.
  Operands.push_back(MYRISCVXOperand::CreateToken("(", getLoc()));

  const AsmToken &Tok1 = Parser.getTok(); // get next token
  if (parseRegister(Operands)) {
    Error(Parser.getTok().getLoc(), "unexpected token in operand");
  }

  const AsmToken &Tok2 = Parser.getTok(); // get next token
  if (Tok2.isNot(AsmToken::RParen)) {
    Error(Parser.getTok().getLoc(), "')' expected");
    return MatchOperand_ParseFail;
  }

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

  Parser.Lex(); // Eat ')' token.
  Operands.push_back(MYRISCVXOperand::CreateToken(")", getLoc()));

  if (!IdVal)
    IdVal = MCConstantExpr::create(0, getContext());

  return MatchOperand_Success;
}

bool MYRISCVXAsmParser::
parseMathOperation(StringRef Name, SMLoc NameLoc,
                   OperandVector &Operands) {
  // split the format
  size_t Start = Name.find('.'), Next = Name.rfind('.');
  StringRef Format1 = Name.slice(Start, Next);
  // and add the first format to the operands
  Operands.push_back(MYRISCVXOperand::CreateToken(Format1, NameLoc));
  // now for the second format
  StringRef Format2 = Name.slice(Next, StringRef::npos);
  Operands.push_back(MYRISCVXOperand::CreateToken(Format2, NameLoc));

  // set the format for the first register
  //  setFpFormat(Format1);

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    if (ParseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }

    if (getLexer().isNot(AsmToken::Comma)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");

    }
    Parser.Lex();  // Eat the comma.

    // Parse and remember the operand.
    if (ParseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    Parser.eatToEndOfStatement();
    return Error(Loc, "unexpected token in argument list");
  }

  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool MYRISCVXAsmParser::
ParseInstruction(ParseInstructionInfo &Info, StringRef Name, SMLoc NameLoc,
                 OperandVector &Operands) {

  // Create the leading tokens for the mnemonic, split by '.' characters.
  size_t Start = 0, Next = Name.find('.');
  StringRef Mnemonic = Name.slice(Start, Next);

  // Refer to the explanation in source code of function DecodeJumpFR(...) in
  // MYRISCVXDisassembler.cpp
  if (Mnemonic == "ret")
    Mnemonic = "jr";

  Operands.push_back(MYRISCVXOperand::CreateToken(Mnemonic, NameLoc));

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    if (ParseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }

    while (getLexer().is(AsmToken::Comma) ) {
      Parser.Lex();  // Eat the comma.

      // Parse and remember the operand.
      if (ParseOperand(Operands, Name)) {
        SMLoc Loc = getLexer().getLoc();
        Parser.eatToEndOfStatement();
        return Error(Loc, "unexpected token in argument list");
      }
    }
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    Parser.eatToEndOfStatement();
    return Error(Loc, "unexpected token in argument list");
  }

  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool MYRISCVXAsmParser::reportParseError(StringRef ErrorMsg) {
  SMLoc Loc = getLexer().getLoc();
  Parser.eatToEndOfStatement();
  return Error(Loc, ErrorMsg);
}

bool MYRISCVXAsmParser::parseSetReorderDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token in statement");
    return false;
  }
  Options.setReorder();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool MYRISCVXAsmParser::parseSetNoReorderDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token in statement");
    return false;
  }
  Options.setNoreorder();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool MYRISCVXAsmParser::parseSetMacroDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token in statement");
    return false;
  }
  Options.setMacro();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool MYRISCVXAsmParser::parseSetNoMacroDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("`noreorder' must be set before `nomacro'");
    return false;
  }
  if (Options.isReorder()) {
    reportParseError("`noreorder' must be set before `nomacro'");
    return false;
  }
  Options.setNomacro();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}
bool MYRISCVXAsmParser::parseDirectiveSet() {

  // get next token
  const AsmToken &Tok = Parser.getTok();

  if (Tok.getString() == "reorder") {
    return parseSetReorderDirective();
  } else if (Tok.getString() == "noreorder") {
    return parseSetNoReorderDirective();
  } else if (Tok.getString() == "macro") {
    return parseSetMacroDirective();
  } else if (Tok.getString() == "nomacro") {
    return parseSetNoMacroDirective();
  }
  return true;
}

bool MYRISCVXAsmParser::ParseDirective(AsmToken DirectiveID) {

  if (DirectiveID.getString() == ".ent") {
    // ignore this directive for now
    Parser.Lex();
    return false;
  }

  if (DirectiveID.getString() == ".end") {
    // ignore this directive for now
    Parser.Lex();
    return false;
  }

  if (DirectiveID.getString() == ".frame") {
    // ignore this directive for now
    Parser.eatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".set") {
    return parseDirectiveSet();
  }

  if (DirectiveID.getString() == ".fmask") {
    // ignore this directive for now
    Parser.eatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".mask") {
    // ignore this directive for now
    Parser.eatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".gpword") {
    // ignore this directive for now
    Parser.eatToEndOfStatement();
    return false;
  }

  return true;
}

extern "C" void LLVMInitializeMYRISCVXAsmParser() {
  RegisterMCAsmParser<MYRISCVXAsmParser> X(getTheMYRISCVX32Target());
}
