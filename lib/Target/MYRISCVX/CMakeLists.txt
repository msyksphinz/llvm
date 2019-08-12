set(LLVM_TARGET_DEFINITIONS MYRISCVX.td)

# Generate MYRISCVXGenRegisterInfo.inc and MYRISCVXGenInstrInfo.inc which included by
#  your hand code C++ files.
# MYRISCVXGenRegisterInfo.inc came from MYRISCVXRegisterInfo.td, MYRISCVXGenInstrInfo.inc
#  came from MYRISCVXInstrInfo.td.
tablegen(LLVM MYRISCVXGenRegisterInfo.inc -gen-register-info)
# tablegen(LLVM MYRISCVXGenInstrInfo.inc -gen-instr-info)
# tablegen(LLVM MYRISCVXGenSubtargetInfo.inc -gen-subtarget)
# tablegen(LLVM MYRISCVXGenMCPseudoLowering.inc -gen-pseudo-lowering)

# MYRISCVXCommonTableGen must be defined
add_public_tablegen_target(MYRISCVXCommonTableGen)

# MYRISCVXCodeGen should match with LLVMBuild.txt MYRISCVXCodeGen
add_llvm_target(MYRISCVXCodeGen
  MYRISCVXTargetMachine.cpp
)

# Should match with "subdirectories =  MCTargetDesc TargetInfo" in LLVMBuild.txt
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
