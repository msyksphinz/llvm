//===-- MYRISCVXInstrFormats.td - MYRISCVX Instruction Formats -----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Describe MYRISCVX instructions format
//
//  CPU INSTRUCTION FORMATS
//
//  opcode  - operation code.
//  ra      - dst reg, only used on 3 regs instr.
//  rb      - src reg.
//  rc      - src reg (on a 3 reg instr).
//  cx      - immediate
//
//===----------------------------------------------------------------------===//

// Format specifies the encoding used by the instruction.  This is part of the
// ad-hoc solution used to emit machine instruction encodings by our machine
// code emitter.
class Format<bits<4> val> {
  bits<4> Value = val;
}

def Pseudo : Format<0>;
def FrmR   : Format<1>;
def FrmI   : Format<2>;
def FrmS   : Format<3>;
def FrmU   : Format<4>; // Instruction w/ a custom format


// Generic MYRISCVX Format
class MYRISCVXInst<dag outs, dag ins, string asmstr, list<dag> pattern,
                   InstrItinClass itin, Format f>: Instruction
{
  // Inst and Size: for tablegen(... -gen-emitter) and
  // tablegen(... -gen-disassembler) in CMakeLists.txt
  field bits<32> Inst;
  Format Form = f;

  let Namespace = "MYRISCVX";

  let Size = 4;

  bits<7> Opcode = 0;

  // Bottom 7 bits are the 'opcode' field
  let Inst{6-0} = Opcode;

  let OutOperandList = outs;
  let InOperandList  = ins;

  let AsmString   = asmstr;
  let Pattern     = pattern;
  let Itinerary   = itin;

  //
  // Attributes specific to MYRISCVX instructions...
  //
  bits<4> FormBits = Form.Value;

  // TSFlags layout should be kept in sync with MYRISCVXInstrInfo.h.
  let TSFlags{3-0}   = FormBits;

  let DecoderNamespace = "MYRISCVX";

  field bits<32> SoftFail = 0;
}

//===----------------------------------------------------------------------===//
// R-Type instruction class in MYRISCVX : <|opcode|ra|rb|rc|cx|>
//===----------------------------------------------------------------------===//

class FR<bits<7> opcode, bits<3> funct3, bits<7> funct7,
         dag outs, dag ins, string asmstr,
         list<dag> pattern, InstrItinClass itin>:
      MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5>  rd;
  bits<5>  rs1;
  bits<5>  rs2;

  let Opcode = opcode;

  let Inst{31-25} = funct7;
  let Inst{24-20} = rs2;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7}  = rd;
}

//@class FI {
//===----------------------------------------------------------------------===//
// I-Type instruction class in MYRISCVX : <|opcode|ra|rb|cx|>
//===----------------------------------------------------------------------===//
class FI<bits<7> opcode, bits<3> funct3,
         dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<5>  rd;
  bits<5>  rs1;
  bits<12> imm12;

  let Opcode = opcode;

  let Inst{31-20} = imm12;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7}  = rd;
}
//@class FI }


//@class FIShift {
//===----------------------------------------------------------------------===//
// I-Type instruction class in MYRISCVX : <|opcode|ra|rb|cx|>
//===----------------------------------------------------------------------===//
class FIShift<bits<7> opcode, bits<3> funct3, bit arithshift,
         dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<5> rs1;
  bits<5> rd;
  bits<6> shamt;

  let Opcode = opcode;

  let Inst{31} = 0;
  let Inst{30} = arithshift;
  let Inst{29-26} = 0;
  let Inst{25-20} = shamt;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7}  = rd;
}
//@class FIShift }


//@class FS {
//===----------------------------------------------------------------------===//
// S-Type instruction class in MYRISCVX : <|opcode|ra|rb|cx|>
//===----------------------------------------------------------------------===//

class FS<bits<7> opcode, bits<3> funct3,
         dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmS>
{
  bits<5>  rs2;
  bits<5>  rs1;
  bits<12> simm12;

  let Opcode = opcode;

  let Inst{31-25} = simm12{11-5};
  let Inst{19-15} = rs1;
  let Inst{24-20} = rs2;
  let Inst{14-12} = funct3;
  let Inst{11-7}  = simm12{4-0};

  let DecoderMethod = "DecodeStore";
}
//@class FS }


//@class FU {
//===----------------------------------------------------------------------===//
// I-Type instruction class in MYRISCVX : <|opcode|ra|rb|cx|>
//===----------------------------------------------------------------------===//

class FU<bits<7> opcode,
         dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<5>  rd;
  bits<20> imm20;

  let Opcode = opcode;

  let Inst{31-12} = imm20;
  let Inst{11-7}  = rd;
}
//@class FU }


//@class FB {
//===----------------------------------------------------------------------===//
// B-Type instruction class in MYRISCVX : <|opcode|ra|rb|cx|>
//===----------------------------------------------------------------------===//

class FB<bits<7> opcode, bits<3> funct3,
         dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<12> imm12;
  bits<5>  rs2;
  bits<5>  rs1;

  let Opcode = opcode;

  let Inst{31}    = imm12{11};
  let Inst{30-25} = imm12{9-4};
  let Inst{14-12} = funct3;
  let Inst{11-8}  = imm12{3-0};
  let Inst{7}     = imm12{10};
}
//@class FB }


//@class FJ {
//===----------------------------------------------------------------------===//
// J-Type instruction class in MYRISCVX : <|opcode|ra|rb|cx|>
//===----------------------------------------------------------------------===//

class FJ<bits<7> opcode,
         dag outs, dag ins, string asmstr, list<dag> pattern,
         InstrItinClass itin>: MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<20> imm20;

  let Opcode = opcode;

  let Inst{31}    = imm20{19};
  let Inst{30-21} = imm20{9-0};
  let Inst{20}    = imm20{10};
  let Inst{19-12} = imm20{18-11};
}
//@class FJ }


// MYRISCVX Pseudo Instructions Format
class MYRISCVXPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>:
                     MYRISCVXInst<outs, ins, asmstr, pattern, IIPseudo, Pseudo> {
  let isCodeGenOnly = 1;
  let isPseudo = 1;
}
