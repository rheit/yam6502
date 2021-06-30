#include "stdint.h"
#include "yam6502.h"
#include <stdio.h>
#include <memory>
#include <array>

// This is a simple bus that contains 64k of RAM with no I/O.
struct RAMBus {
	[[maybe_unused]] uint8_t ReadAddr(uint16_t addr) const { return Memory[addr]; }
	void WriteAddr(uint16_t addr, uint8_t val) { Memory[addr] = val; }
	uint8_t &operator[](uint16_t addr) { return Memory[addr]; }

	std::array<uint8_t, 65536> Memory = { 0 };
};

// This is a slightly specialized version of RAMBus that, when address $000F
// is written to, it also sends that character to stdout.
struct CharOutBus : RAMBus {
	void WriteAddr(uint16_t addr, uint8_t val)
	{
		Memory[addr] = val;
		if (addr == 0x0F) {
			putc(val, stdout);
		}
	}
};

int main()
{
	CharOutBus mem;
	yam::M6502<CharOutBus *> cpu(&mem);

	// Small routine to print a zero terminated string beginning
	// at address $40 by writing the characters to an output
	// port at address $0F.
	static const uint8_t code[] = {
		0xA2, 0x00,		// LDX #0
		0xB5, 0x40,		// LDA $40
		0xF0, 0x05,		// BEQ $60B (+5)
		0x85, 0x0F,		// STA $0F
		0xE8,			// INX
		0xD0, 0xF7,		// BNE $602 (-9)
		0x00			// BRK
	};
	static const char message[] = "Hello, world!\nThese two lines of text were output by the following 6502 code:\n\n";

	// Copy the program to address $0600
	std::copy(std::begin(code), std::end(code), &mem[0x600]);

	// Copy the message to address $0040
	std::copy(std::begin(message), std::end(message), &mem[0x40]);

	// Set the reset vector to begin executing our code at $0600
	const auto resetvec = cpu.opToVector(yam::op::RESET);
	mem[resetvec] = 0x00;
	mem[resetvec + 1] = 0x06;

	// Set the break vector to point to $FF00
	const auto breakvec = cpu.opToVector(yam::op::BRK);
	mem[breakvec] = 0x00;
	mem[breakvec + 1] = 0xFF;

	// Loop until the PC hits $FF00. This will happen when the BRK
	// at the end of our routine is executed.
	for (auto pc = cpu.getPC(); pc != 0xFF00; pc = cpu.getPC()) {
		cpu.Tick();
	}

	// Now disassemble the program
	for (uint16_t codeptr = 0x600; codeptr < 0x600 + sizeof(code); ) {
		puts(cpu.DisasmStep(codeptr, true).c_str());
	}
}
