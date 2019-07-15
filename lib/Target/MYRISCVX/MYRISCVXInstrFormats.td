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
//===----------------------------------------------------------------------===//

// Format specifies the encoding used by the instruction.  This is part of the
// ad-hoc solution used to emit machine instruction encodings by our machine
// code emitter.
class Format<bits<3> val> {
  bits<3> Value = val;
}


def Pseudo : Format<0>;
def FrmR   : Format<1>;
def FrmI   : Format<2>;
def FrmS   : Format<3>;
def FrmU   : Format<4>;
def FrmB   : Format<5>;
def FrmJ   : Format<6>;


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
  bits<3> FormBits = Form.Value;

  // TSFlags layout should be kept in sync with MYRISCVXInstrInfo.h.
  let TSFlags{2-0}   = FormBits;

  let DecoderNamespace = "MYRISCVX";

  field bits<32> SoftFail = 0;
}


//===----------------------------------------------------------------------===//
// R-Type instruction class in MYRISCVX : <|opcode|funct7|funct3|rd|rs1|rs2|>
//===----------------------------------------------------------------------===//

class MYRISCVX_R<bits<7> opcode, bits<3> funct3, bits<7> funct7,
                dag outs, dag ins, string asmstr,
                list<dag> pattern, InstrItinClass itin>:
      MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmR>
{
  bits<5>  rd;
  bits<5>  rs1;
  bits<5>  rs2;

  let Inst{31-25} = funct7;
  let Inst{24-20} = rs2;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7}  = rd;
  let Inst{6-0}   = opcode;
}


//===----------------------------------------------------------------------===//
// I-Type instruction class in MYRISCVX : <|opcode|funct3|rd|rs1|imm12|>
//===----------------------------------------------------------------------===//
class MYRISCVX_I<bits<7> opcode, bits<3> funct3,
                 dag outs, dag ins, string asmstr, list<dag> pattern,
                 InstrItinClass itin>:
  MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<5>  rd;
  bits<5>  rs1;
  bits<12> simm12;

  let Inst{31-20} = simm12;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7}  = rd;
  let Inst{6-0}   = opcode;
}


//===----------------------------------------------------------------------===//
// S-Type instruction class in MYRISCVX : <|opcode|rs2|rs1|width|offset>
//===----------------------------------------------------------------------===//
class MYRISCVX_S<bits<7> opcode, bits<3> funct3,
                 dag outs, dag ins, string asmstr, list<dag> pattern,
                 InstrItinClass itin>:
  MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmS>
{
  bits<5>  rs2;
  bits<5>  rs1;
  bits<12> simm12;

  let Inst{31-25} = simm12{11-5};
  let Inst{19-15} = rs1;
  let Inst{24-20} = rs2;
  let Inst{14-12} = funct3;
  let Inst{11-7}  = simm12{4-0};
  let Inst{6-0}   = opcode;
}



//===----------------------------------------------------------------------===//
// U-Type instruction class in MYRISCVX : <|opcode|rd|imm31-12>
//===----------------------------------------------------------------------===//
class MYRISCVX_U<bits<7> opcode,
                 dag outs, dag ins, string asmstr, list<dag> pattern,
                 InstrItinClass itin>:
  MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmU>
{
  bits<5>  rd;
  bits<20> imm20;

  let Inst{31-12} = imm20;
  let Inst{11-7}  = rd;
  let Inst{6-0}   = opcode;
}


//===----------------------------------------------------------------------===//
// B-Type instruction class in MYRISCVX : <|opcode|funct3|rs1|rs2|imm12|>
//===----------------------------------------------------------------------===//
class MYRISCVX_B<bits<7> opcode, bits<3> funct3,
                 dag outs, dag ins, string asmstr, list<dag> pattern,
                 InstrItinClass itin>:
  MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmB>
{
  bits<12> imm12;
  bits<5>  rs2;
  bits<5>  rs1;

  let Inst{31}    = imm12{11};
  let Inst{30-25} = imm12{9-4};
  let Inst{14-12} = funct3;
  let Inst{11-8}  = imm12{3-0};
  let Inst{7}     = imm12{10};
  let Inst{6-0}   = opcode;
}


//===----------------------------------------------------------------------===//
// J-Type instruction class in MYRISCVX : <|opcode|rd|imm20|>
//===----------------------------------------------------------------------===//
class MYRISCVX_J<bits<7> opcode,
                 dag outs, dag ins, string asmstr, list<dag> pattern,
                 InstrItinClass itin>:
  MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmJ>
{
  bits<5>  rd;
  bits<20> imm20;

  let Inst{31}    = imm20{19};
  let Inst{30-21} = imm20{9-0};
  let Inst{20}    = imm20{10};
  let Inst{19-12} = imm20{18-11};
  let Inst{11-7}  = rd;
  let Inst{6-0}   = opcode;
}


//===----------------------------------------------------------------------===//
// I-Type instruction class in MYRISCVX : <|opcode|ra|rb|cx|>
//===----------------------------------------------------------------------===//
class MYRISCVX_ISHIFT<bits<7> opcode, bits<3> funct3, bit arithshift,
                      dag outs, dag ins, string asmstr, list<dag> pattern,
                      InstrItinClass itin>:
  MYRISCVXInst<outs, ins, asmstr, pattern, itin, FrmI>
{
  bits<5> rs1;
  bits<5> rd;
  bits<6> shamt;

  let Inst{31} = 0;
  let Inst{30} = arithshift;
  let Inst{29-26} = 0;
  let Inst{25-20} = shamt;
  let Inst{19-15} = rs1;
  let Inst{14-12} = funct3;
  let Inst{11-7}  = rd;
  let Inst{6-0}   = opcode;
}


//===-------------------------------===//
// MYRISCVX Pseudo Instructions Format
//===-------------------------------===//
class MYRISCVXPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>:
      MYRISCVXInst<outs, ins, asmstr, pattern, IIPseudo, Pseudo> {
  let isCodeGenOnly = 1;
  let isPseudo = 1;
}
