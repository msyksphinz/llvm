# MCTargetDesc/CMakeLists.txt
add_llvm_library(LLVMMYRISCVXDesc
  MYRISCVXMCTargetDesc.cpp
  MYRISCVXABIInfo.cpp
  MYRISCVXMCAsmInfo.cpp
  MYRISCVXMCCodeEmitter.cpp
  MYRISCVXMCExpr.cpp
  MYRISCVXTargetStreamer.cpp
)
