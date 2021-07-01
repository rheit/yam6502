/*
** helloworld.cpp
** Simple example showing basic usage of the 6502 emulator.
**
** Copyright(c) 2021 Marisa Heit
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*/

#include "stdint.h"
#include "yam6502.hpp"
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
