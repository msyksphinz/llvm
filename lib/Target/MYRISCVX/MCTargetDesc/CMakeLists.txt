# MCTargetDesc/CMakeLists.txt
add_llvm_library(LLVMMYRISCVXDesc
  MYRISCVXMCTargetDesc.cpp
  MYRISCVXABIInfo.cpp
  MYRISCVXMCAsmInfo.cpp
  MYRISCVXAsmBackend.cpp
  MYRISCVXMCCodeEmitter.cpp
  MYRISCVXMCExpr.cpp
  MYRISCVXELFObjectWriter.cpp
  MYRISCVXTargetStreamer.cpp
)
