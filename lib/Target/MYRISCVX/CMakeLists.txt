set(LLVM_TARGET_DEFINITIONS MYRISCVX.td)

# Generate MYRISCVX_GenRegisterInfo.inc and MYRISCVX_GenInstrInfo.inc which included by
#  your hand code C++ files.
# MYRISCVX_GenRegisterInfo.inc came from MYRISCVXRegisterInfo.td, MYRISCVX_GenInstrInfo.inc
#  came from MYRISCVXInstrInfo.td.
tablegen(LLVM MYRISCVXGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM MYRISCVXGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM MYRISCVXGenCodeEmitter.inc -gen-emitter)
tablegen(LLVM MYRISCVXGenMCCodeEmitter.inc -gen-emitter)

# tablegen(LLVM MYRISCVXGenAsmWriter.inc -gen-asm-writer)
# tablegen(LLVM MYRISCVXGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM MYRISCVXGenCallingConv.inc -gen-callingconv)
tablegen(LLVM MYRISCVXGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM MYRISCVXGenMCPseudoLowering.inc -gen-pseudo-lowering)

# MYRISCVXCommonTableGen must be defined
add_public_tablegen_target(MYRISCVXCommonTableGen)

# MYRISCVXCodeGen should match with LLVMBuild.txt MYRISCVXCodeGen
add_llvm_target(MYRISCVXCodeGen
  MYRISCVXTargetMachine.cpp
  MYRISCVXTargetObjectFile.cpp
  MYRISCVXSubtarget.cpp
  MYRISCVXISelLowering.cpp
  MYRISCVXSEISelLowering.cpp
  MYRISCVXRegisterInfo.cpp
  MYRISCVXSERegisterInfo.cpp
  MYRISCVXMachineFunction.cpp
  MYRISCVXInstrInfo.cpp
  MYRISCVXSEInstrInfo.cpp
  MYRISCVXFrameLowering.cpp
  MYRISCVXSEFrameLowering.cpp
  )

# Should match with "subdirectories =  MCTargetDesc TargetInfo" in LLVMBuild.txt
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
