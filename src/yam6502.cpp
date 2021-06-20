#include "yam6502.h"
#include <array>

// Common data for the 6502 emulation that is always the same no matter how
// the templated class is instantiated.
namespace m65xx {
	const char OpNames[][4] = {
		"ADC",
		"AND",
		"ASL",
		"BCC",
		"BCS",
		"BEQ",
		"BIT",
		"BMI",
		"BNE",
		"BPL",
		"BRK",
		"BVC",
		"BVS",
		"CLC",
		"CLD",
		"CLI",
		"CLV",
		"CMP",
		"CPX",
		"CPY",
		"DEC",
		"DEX",
		"DEY",
		"EOR",
		"INC",
		"INX",
		"INY",
		"JMP",
		"JSR",
		"LDA",
		"LDX",
		"LDY",
		"LSR",
		"NOP",
		"ORA",
		"PHA",
		"PHP",
		"PLA",
		"PLP",
		"ROL",
		"ROR",
		"RTI",
		"RTS",
		"SBC",
		"SEC",
		"SED",
		"SEI",
		"STA",
		"STX",
		"STY",
		"TAX",
		"TAY",
		"TSX",
		"TXA",
		"TXS",
		"TYA",
		// Illegal operations
		"AHX",
		"ALR",
		"ANC",
		"ARR",
		"AXS",
		"DCP",
		"ISC",
		"KIL",
		"LAS",
		"LAX",
		"LAX",
		"RLA",
		"RRA",
		"SAX",
		"SHX",
		"SHY",
		"SLO",
		"SRE",
		"TAS",
		"XAA",
	};
	static_assert(std::size(OpNames) == static_cast<int>(op::COUNT));

	// Opcode definitions
	const BaseOpcode Opc6502[256] = {
		{ op::BRK, am::brk },		// 00
		{ op::ORA, am::izx },		// 01
		{ op::KIL, am::imp },		// 02
		{ op::SLO, am::izx_rmw },	// 03
		{ op::NOP, am::zp },		// 04
		{ op::ORA, am::zp },		// 05
		{ op::ASL, am::zp_rmw },	// 06
		{ op::SLO, am::zp_rmw },	// 07
		{ op::PHP, am::push },		// 08
		{ op::ORA, am::imm },		// 09
		{ op::ASL, am::acc },		// 0A
		{ op::ANC, am::imm },		// 0B
		{ op::NOP, am::abs },		// 0C
		{ op::ORA, am::abs },		// 0D
		{ op::ASL, am::abs_rmw },	// 0E
		{ op::SLO, am::abs_rmw },	// 0F

		{ op::BPL, am::rel },		// 10
		{ op::ORA, am::izy },		// 11
		{ op::KIL, am::imp },		// 12
		{ op::SLO, am::izy_rmw },	// 13
		{ op::NOP, am::zpx },		// 14
		{ op::ORA, am::zpx },		// 15
		{ op::ASL, am::zpx_rmw },	// 16
		{ op::SLO, am::zpx_rmw },	// 17
		{ op::CLC, am::imp },		// 18
		{ op::ORA, am::aby },		// 19
		{ op::NOP, am::imp },		// 1A
		{ op::SLO, am::aby_rmw },	// 1B
		{ op::NOP, am::abx },		// 1C
		{ op::ORA, am::abx },		// 1D
		{ op::ASL, am::abx_rmw },	// 1E
		{ op::SLO, am::abx_rmw },	// 1F

		{ op::JSR, am::call },		// 20
		{ op::AND, am::izx },		// 21
		{ op::KIL, am::imp },		// 22
		{ op::RLA, am::izx_rmw },	// 23
		{ op::BIT, am::zp },		// 24
		{ op::AND, am::zp },		// 25
		{ op::ROL, am::zp_rmw },	// 26
		{ op::RLA, am::zp_rmw },	// 27
		{ op::PLP, am::pull },		// 28
		{ op::AND, am::imm },		// 29
		{ op::ROL, am::acc },		// 2A
		{ op::ANC, am::imm },		// 2B
		{ op::BIT, am::abs },		// 2C
		{ op::AND, am::abs },		// 2D
		{ op::ROL, am::abs_rmw },	// 2E
		{ op::RLA, am::abs_rmw },	// 2F

		{ op::BMI, am::rel },		// 30
		{ op::AND, am::izy },		// 31
		{ op::KIL, am::imp },		// 32
		{ op::RLA, am::izy_rmw },	// 33
		{ op::NOP, am::zpx },		// 34
		{ op::AND, am::zpx },		// 35
		{ op::ROL, am::zpx_rmw },	// 36
		{ op::RLA, am::zpx_rmw },	// 37
		{ op::SEC, am::imp },		// 38
		{ op::AND, am::aby },		// 39
		{ op::NOP, am::imp },		// 3A
		{ op::RLA, am::aby_rmw },	// 3B
		{ op::NOP, am::abx },		// 3C
		{ op::AND, am::abx },		// 3D
		{ op::ROL, am::abx_rmw },	// 3E
		{ op::RLA, am::abx_rmw },	// 3F

		{ op::RTI, am::rti },		// 40
		{ op::EOR, am::izx },		// 41
		{ op::KIL, am::imp },		// 42
		{ op::SRE, am::izx_rmw },	// 43
		{ op::NOP, am::zp },		// 44
		{ op::EOR, am::zp },		// 45
		{ op::LSR, am::zp_rmw },	// 46
		{ op::SRE, am::zp_rmw },	// 47
		{ op::PHA, am::push },		// 48
		{ op::EOR, am::imm },		// 49
		{ op::LSR, am::acc },		// 4A
		{ op::ALR, am::imm },		// 4B
		{ op::JMP, am::jmp },		// 4C
		{ op::EOR, am::abs },		// 4D
		{ op::LSR, am::abs_rmw },	// 4E
		{ op::SRE, am::abs_rmw },	// 4F

		{ op::BVC, am::rel },		// 50
		{ op::EOR, am::izy },		// 51
		{ op::KIL, am::imp },		// 52
		{ op::SRE, am::izy_rmw },	// 53
		{ op::NOP, am::zpx },		// 54
		{ op::EOR, am::zpx },		// 55
		{ op::LSR, am::zpx_rmw },	// 56
		{ op::SRE, am::zpx_rmw },	// 57
		{ op::CLI, am::imp },		// 58
		{ op::EOR, am::aby },		// 59
		{ op::NOP, am::imp },		// 5A
		{ op::SRE, am::aby_rmw },	// 5B
		{ op::NOP, am::abx },		// 5C
		{ op::EOR, am::abx },		// 5D
		{ op::LSR, am::abx_rmw },	// 5E
		{ op::SRE, am::abx_rmw },	// 5F

		{ op::RTS, am::rts },		// 60
		{ op::ADC, am::izx },		// 61
		{ op::KIL, am::imp },		// 62
		{ op::RRA, am::izx_rmw },	// 63
		{ op::NOP, am::zp },		// 64
		{ op::ADC, am::zp },		// 65
		{ op::ROR, am::zp_rmw },	// 66
		{ op::RRA, am::zp_rmw },	// 67
		{ op::PLA, am::pull },		// 68
		{ op::ADC, am::imm },		// 69
		{ op::ROR, am::acc },		// 6A
		{ op::ARR, am::imm },		// 6B
		{ op::JMP, am::jmp_ind },	// 6C
		{ op::ADC, am::abs },		// 6D
		{ op::ROR, am::abs_rmw },	// 6E
		{ op::RRA, am::abs_rmw },	// 6F

		{ op::BVS, am::rel },		// 70
		{ op::ADC, am::izy },		// 71
		{ op::KIL, am::imp },		// 72
		{ op::RRA, am::izy_rmw },	// 73
		{ op::NOP, am::zpx },		// 74
		{ op::ADC, am::zpx },		// 75
		{ op::ROR, am::zpx_rmw },	// 76
		{ op::RRA, am::zpx_rmw },	// 77
		{ op::SEI, am::imp },		// 78
		{ op::ADC, am::aby },		// 79
		{ op::NOP, am::imp },		// 7A
		{ op::RRA, am::aby_rmw },	// 7B
		{ op::NOP, am::abx },		// 7C
		{ op::ADC, am::abx },		// 7D
		{ op::ROR, am::abx_rmw },	// 7E
		{ op::RRA, am::abx_rmw },	// 7F

		{ op::NOP, am::imm },		// 80
		{ op::STA, am::izx_sto },	// 81
		{ op::NOP, am::imm },		// 82
		{ op::SAX, am::izx_sto },	// 83
		{ op::STY, am::zp_sto },	// 84
		{ op::STA, am::zp_sto },	// 85
		{ op::STX, am::zp_sto },	// 86
		{ op::SAX, am::zp_sto },	// 87
		{ op::DEY, am::imp },		// 88
		{ op::NOP, am::imm },		// 89
		{ op::TXA, am::imp },		// 8A
		{ op::XAA, am::imm },		// 8B
		{ op::STY, am::abs_sto },	// 8C
		{ op::STA, am::abs_sto },	// 8D
		{ op::STX, am::abs_sto },	// 8E
		{ op::SAX, am::abs_sto },	// 8F

		{ op::BCC, am::rel },		// 90
		{ op::STA, am::izy_sto },	// 91
		{ op::KIL, am::imp },		// 92
		{ op::AHX, am::izy_sto },	// 93
		{ op::STY, am::zpx_sto },	// 94
		{ op::STA, am::zpx_sto },	// 95
		{ op::STX, am::zpy_sto },	// 96
		{ op::SAX, am::zpy_sto },	// 97
		{ op::TYA, am::imp },		// 98
		{ op::STA, am::aby_sto },	// 99
		{ op::TXS, am::imp },		// 9A
		{ op::TAS, am::aby_sto },	// 9B
		{ op::SHY, am::abx_sto },	// 9C
		{ op::STA, am::abx_sto },	// 9D
		{ op::SHX, am::aby_sto },	// 9E
		{ op::AHX, am::aby_sto },	// 9F

		{ op::LDY, am::imm },		// A0
		{ op::LDA, am::izx },		// A1
		{ op::LDX, am::imm },		// A2
		{ op::LAX, am::izx },		// A3
		{ op::LDY, am::zp },		// A4
		{ op::LDA, am::zp },		// A5
		{ op::LDX, am::zp },		// A6
		{ op::LAX, am::zp },		// A7
		{ op::TAY, am::imp },		// A8
		{ op::LDA, am::imm },		// A9
		{ op::TAX, am::imp },		// AA
		{ op::LAX_IMM, am::imm },	// AB
		{ op::LDY, am::abs },		// AC
		{ op::LDA, am::abs },		// AD
		{ op::LDX, am::abs },		// AE
		{ op::LAX, am::abs },		// AF

		{ op::BCS, am::rel },		// B0
		{ op::LDA, am::izy },		// B1
		{ op::KIL, am::imp },		// B2
		{ op::LAX, am::izy },		// B3
		{ op::LDY, am::zpx },		// B4
		{ op::LDA, am::zpx },		// B5
		{ op::LDX, am::zpy },		// B6
		{ op::LAX, am::zpy },		// B7
		{ op::CLV, am::imp },		// B8
		{ op::LDA, am::aby },		// B9
		{ op::TSX, am::imp },		// BA
		{ op::LAS, am::aby },		// BB
		{ op::LDY, am::abx },		// BC
		{ op::LDA, am::abx },		// BD
		{ op::LDX, am::aby },		// BE
		{ op::LAX, am::aby },		// BF

		{ op::CPY, am::imm },		// C0
		{ op::CMP, am::izx },		// C1
		{ op::NOP, am::imm },		// C2
		{ op::DCP, am::izx_rmw },	// C3
		{ op::CPY, am::zp },		// C4
		{ op::CMP, am::zp },		// C5
		{ op::DEC, am::zp_rmw },	// C6
		{ op::DCP, am::zp_rmw },	// C7
		{ op::INY, am::imp },		// C8
		{ op::CMP, am::imm },		// C9
		{ op::DEX, am::imp },		// CA
		{ op::AXS, am::imm },		// CB
		{ op::CPY, am::abs },		// CC
		{ op::CMP, am::abs },		// CD
		{ op::DEC, am::abs_rmw },	// CE
		{ op::DCP, am::abs_rmw },	// CF

		{ op::BNE, am::rel },		// D0
		{ op::CMP, am::izy },		// D1
		{ op::KIL, am::imp },		// D2
		{ op::DCP, am::izy_rmw },	// D3
		{ op::NOP, am::zpx },		// D4
		{ op::CMP, am::zpx },		// D5
		{ op::DEC, am::zpx_rmw },	// D6
		{ op::DCP, am::zpx_rmw },	// D7
		{ op::CLD, am::imp },		// D8
		{ op::CMP, am::aby },		// D9
		{ op::NOP, am::imp },		// DA
		{ op::DCP, am::aby_rmw },	// DB
		{ op::NOP, am::abx },		// DC
		{ op::CMP, am::abx },		// DD
		{ op::DEC, am::abx_rmw },	// DE
		{ op::DCP, am::abx_rmw },	// DF

		{ op::CPX, am::imm },		// E0
		{ op::SBC, am::izx },		// E1
		{ op::NOP, am::imm },		// E2
		{ op::ISC, am::izx_rmw },	// E3
		{ op::CPX, am::zp },		// E4
		{ op::SBC, am::zp },		// E5
		{ op::INC, am::zp_rmw },	// E6
		{ op::ISC, am::zp_rmw },	// E7
		{ op::INX, am::imp },		// E8
		{ op::SBC, am::imm },		// E9
		{ op::NOP, am::imp },		// EA
		{ op::SBC, am::imm },		// EB
		{ op::CPX, am::abs },		// EC
		{ op::SBC, am::abs },		// ED
		{ op::INC, am::abs_rmw },	// EE
		{ op::ISC, am::abs_rmw },	// EF

		{ op::BEQ, am::rel },		// F0
		{ op::SBC, am::izy },		// F1
		{ op::KIL, am::imp },		// F2
		{ op::ISC, am::izy_rmw },	// F3
		{ op::NOP, am::zpx },		// F4
		{ op::SBC, am::zpx },		// F5
		{ op::INC, am::zpx_rmw },	// F6
		{ op::ISC, am::zpx_rmw },	// F7
		{ op::SED, am::imp },		// F8
		{ op::SBC, am::aby },		// F9
		{ op::NOP, am::imp },		// FA
		{ op::ISC, am::aby_rmw },	// FB
		{ op::NOP, am::abx },		// FC
		{ op::SBC, am::abx },		// FD
		{ op::INC, am::abx_rmw },	// FE
		{ op::ISC, am::abx_rmw }	// FF
	};

	const DisasmInfo ModeTable[] = {
		{ 0, "" },			// implicit
		{ 0, "A" },			// accumulator

		{ 1, "#$%02X" },	// #$00
		{ 1, "$%02X" },		// $00
		{ 1, "$%02X,X" },	// $00,X
		{ 1, "$%02X,Y" },	// $00,Y
		{ 1, "($%02X,X)" },	// ($00,X)
		{ 1, "($%02X),Y" },	// ($00),Y
		{ 2, "$%04X" },		// $0000
		{ 2, "$%04X,X" },	// $0000,X
		{ 2, "$%04X,Y" },	// $0000,Y

		// Internal execution on memory data
		{ 1, "$%02X" },		// STx $00
		{ 2, "$%04X" },		// STx $0000
		{ 1, "($%02X,X)" },	// STx ($00,X)
		{ 2, "$%04X,X" },	// STx $0000,X
		{ 2, "$%04X,Y" },	// STx $0000,Y
		{ 1, "$%02X,X" },	// STx $00,X
		{ 1, "$%02X,Y" },	// STx $00,Y
		{ 1, "($%02X),Y" },	// STx ($00),Y

		// Read-Modify-Write operations
		{ 1, "$%02X" },		// RMW $00
		{ 2, "$%04X" },		// RMW $0000
		{ 1, "$%02X,X" },	// RMW $00,X
		{ 2, "$%04X,X" },	// RMW $0000,X
		{ 2, "$%04X,Y" },	// RMW $0000,Y [illegal]
		{ 1, "($%02X,X)" },	// RMW ($00,X) [illegal]
		{ 1, "($%02X),Y" },	// RMW ($00),Y [illegal]

		// Miscellaneous Operations
		{ 0, "" },			// Push to stack
		{ 0, "" },			// Pull from stack
		{ 2, "$%04X" },		// JMP $0000
		{ 2, "($%04X)" },	// JMP ($0000)
		{ 2, "$%04X" },		// JSR $0000
		{ 0, "" },			// Return from Subroutine
		{ 1, "$%04X" },		// PC-relative
		{ 1, "#$%02X" },	// BRK/IRQ/NMI/RESET sequence
		{ 0, "" },			// Return from Interrupt
		{ 0, "" },			// Halt the processor
	};
	static_assert(std::size(ModeTable) == static_cast<int>(am::COUNT));
}
