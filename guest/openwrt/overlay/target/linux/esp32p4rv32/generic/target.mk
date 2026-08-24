ARCH:=riscv32
SUBTARGET:=generic
BOARDNAME:=Generic RV32IMA guest
CPU_TYPE:=rv32ima
ARCH_PACKAGES:=riscv32_rv32ima

define Target/Description
	Build packages for the RV32IMA Linux guest emulated by ESP32-P4.
endef
