# MCTargetDesc/CMakeLists.txt
add_llvm_library(LLVMMYRISCVXDesc
  MYRISCVXMCTargetDesc.cpp
  MYRISCVXMCAsmInfo.cpp
  MYRISCVXABIInfo.cpp
  MYRISCVXAsmBackend.cpp
  MYRISCVXMCCodeEmitter.cpp
  MYRISCVXMCExpr.cpp
  MYRISCVXELFObjectWriter.cpp
  MYRISCVXTargetStreamer.cpp
)
