set(LLVM_TARGET_DEFINITIONS RISCV64.td)

tablegen(LLVM RISCV64GenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM RISCV64GenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM RISCV64GenCallingConv.inc -gen-callingconv)
tablegen(LLVM RISCV64GenDAGISel.inc -gen-dag-isel)
tablegen(LLVM RISCV64GenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM RISCV64GenFastISel.inc -gen-fast-isel)
tablegen(LLVM RISCV64GenGlobalISel.inc -gen-global-isel)
tablegen(LLVM RISCV64GenInstrInfo.inc -gen-instr-info)
tablegen(LLVM RISCV64GenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM RISCV64GenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM RISCV64GenRegisterBank.inc -gen-register-bank)
tablegen(LLVM RISCV64GenRegisterInfo.inc -gen-register-info)
tablegen(LLVM RISCV64GenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(RISCV64CommonTableGen)

add_llvm_target(RISCV64CodeGen
  RISCV6416FrameLowering.cpp
  RISCV6416HardFloat.cpp
  RISCV6416HardFloatInfo.cpp
  RISCV6416InstrInfo.cpp
  RISCV6416ISelDAGToDAG.cpp
  RISCV6416ISelLowering.cpp
  RISCV6416RegisterInfo.cpp
  RISCV64AnalyzeImmediate.cpp
  RISCV64AsmPrinter.cpp
  RISCV64CallLowering.cpp
  RISCV64CCState.cpp
  RISCV64ConstantIslandPass.cpp
  RISCV64DelaySlotFiller.cpp
  RISCV64ExpandPseudo.cpp
  RISCV64FastISel.cpp
  RISCV64InstrInfo.cpp
  RISCV64InstructionSelector.cpp
  RISCV64ISelDAGToDAG.cpp
  RISCV64ISelLowering.cpp
  RISCV64FrameLowering.cpp
  RISCV64LegalizerInfo.cpp
  RISCV64BranchExpansion.cpp
  RISCV64MCInstLower.cpp
  RISCV64MachineFunction.cpp
  RISCV64ModuleISelDAGToDAG.cpp
  RISCV64OptimizePICCall.cpp
  RISCV64Os16.cpp
  RISCV64RegisterBankInfo.cpp
  RISCV64RegisterInfo.cpp
  RISCV64SEFrameLowering.cpp
  RISCV64SEInstrInfo.cpp
  RISCV64SEISelDAGToDAG.cpp
  RISCV64SEISelLowering.cpp
  RISCV64SERegisterInfo.cpp
  RISCV64Subtarget.cpp
  RISCV64TargetMachine.cpp
  RISCV64TargetObjectFile.cpp
  MicroRISCV64SizeReduction.cpp
  )

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(InstPrinter)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
