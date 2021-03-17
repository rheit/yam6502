#include "stdint.h"
#include "yam6502.h"
#include <stdio.h>
#include <memory>
#include <array>


struct RAMBus {
	[[maybe_unused]] uint8_t readAddr(uint16_t addr) const { return Memory[addr]; }
	void writeAddr(uint16_t addr, uint8_t val) { Memory[addr] = val; }
	uint8_t &operator[](uint16_t addr) { return Memory[addr]; }

	std::array<uint8_t, 65536> Memory = { 0 };
};

struct CharOutBus : RAMBus {
	void writeAddr(uint16_t addr, uint8_t val)
	{
		Memory[addr] = val;
		if (addr == 0x0F) {
			putc(val, stdout);
		}
	}
};

template<typename T>
using m6502 = m65xx::M6502<T>;

int main()
{
	CharOutBus mem;
	m6502<CharOutBus *> cpu(&mem);

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

	std::copy(std::begin(code), std::end(code), &mem[0x600]);
	std::copy(std::begin(message), std::end(message), &mem[0x40]);

	// Set the reset vector to begin executing our code at $600
	const auto resetvec = cpu.opToVector(m65xx::op::RESET);
	mem[resetvec] = 0x00;
	mem[resetvec + 1] = 0x06;

	// Set the break vector to point to $FF00
	const auto breakvec = cpu.opToVector(m65xx::op::BRK);
	mem[breakvec] = 0x00;
	mem[breakvec + 1] = 0xFF;

	// Loop until the PC hits $FF00. This will happen when the BRK
	// at the end of our routine is executed.
	for (auto pc = cpu.getPC(); pc != 0xFF00; pc = cpu.getPC()) {
		cpu.tick();
	}

	for (uint16_t codeptr = 0x600; codeptr < 0x600 + sizeof(code); ) {
		puts(cpu.disasmStep(codeptr, true).c_str());
	}
}
