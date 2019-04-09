set(LLVM_TARGET_DEFINITIONS MYRISCVXOther.td)

# Generate MYRISCVXGenRegisterInfo.inc and MYRISCVXGenInstrInfo.inc which included by
# your hand code C++ files.
# MYRISCVXGenRegisterInfo.inc came from MYRISCVXRegisterInfo.td, MYRISCVXGenInstrInfo.inc
# came from MYRISCVXInstrInfo.td.
tablegen(LLVM MYRISCVXGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM MYRISCVXGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM MYRISCVXGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM MYRISCVXGenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM MYRISCVXGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM MYRISCVXGenCallingConv.inc -gen-callingconv)
tablegen(LLVM MYRISCVXGenCodeEmitter.inc -gen-emitter)
tablegen(LLVM MYRISCVXGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM MYRISCVXGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM MYRISCVXGenDisassemblerTables.inc -gen-disassembler)

set(LLVM_TARGET_DEFINITIONS MYRISCVXAsm.td)
tablegen(LLVM MYRISCVXGenAsmMatcher.inc -gen-asm-matcher)

# MYRISCVXCommonTableGen must be defined
add_public_tablegen_target(MYRISCVXCommonTableGen)

# MYRISCVXCodeGen should match with LLVMBuild.txt MYRISCVXCodeGen
add_llvm_target(MYRISCVXCodeGen
  MYRISCVXTargetMachine.cpp
  MYRISCVXSEFrameLowering.cpp
  MYRISCVXFrameLowering.cpp
  MYRISCVXSEInstrInfo.cpp
  MYRISCVXInstrInfo.cpp
  MYRISCVXMachineFunction.cpp
  MYRISCVXSERegisterInfo.cpp
  MYRISCVXRegisterInfo.cpp
  MYRISCVXISelLowering.cpp
  MYRISCVXSEISelLowering.cpp
  MYRISCVXSubtarget.cpp
  MYRISCVXTargetObjectFile.cpp
  MYRISCVXAsmPrinter.cpp
  MYRISCVXMCInstLower.cpp
  MYRISCVXISelDAGToDAG.cpp
  MYRISCVXSEISelDAGToDAG.cpp
  MYRISCVXAnalyzeImmediate.cpp
  MYRISCVXLongBranch.cpp
  MYRISCVXDelUselessJMP.cpp
  MYRISCVXEmitGPRestore.cpp
)

# Should match with "subdirectories = MCTargetDesc TargetInfo" in LLVMBuild.txt
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
add_subdirectory(InstPrinter)
add_subdirectory(Disassembler)
add_subdirectory(AsmParser)
