set(LLVM_TARGET_DEFINITIONS RISCV_msyksphinz.td)

# Generate RISCV_msyksphinz_GenRegisterInfo.inc and RISCV_msyksphinz_GenInstrInfo.inc which included by
#  your hand code C++ files.
# RISCV_msyksphinz_GenRegisterInfo.inc came from RISCV_msyksphinzRegisterInfo.td, RISCV_msyksphinz_GenInstrInfo.inc
#  came from RISCV_msyksphinzInstrInfo.td.
tablegen(LLVM RISCV_msyksphinzGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM RISCV_msyksphinzGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM RISCV_msyksphinzGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM RISCV_msyksphinzGenMCPseudoLowering.inc -gen-pseudo-lowering)

# RISCV_msyksphinzCommonTableGen must be defined
add_public_tablegen_target(RISCV_msyksphinzCommonTableGen)

# RISCV_msyksphinzCodeGen should match with LLVMBuild.txt RISCV_msyksphinzCodeGen
add_llvm_target(RISCV_msyksphinzCodeGen
  RISCV_msyksphinzTargetMachine.cpp
  RISCV_msyksphinzTargetObjectFile.cpp
  )

# Should match with "subdirectories =  MCTargetDesc TargetInfo" in LLVMBuild.txt
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
