#include "stdint.h"
#include "yam6502.h"
#include <stdio.h>
#include <memory>


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

static constexpr char message[] = "'Hello, world!'\nThe emulation is at least capable of outputting text.\n";

int main()
{
	auto mem = std::make_unique<CharOutBus>();
	m6502<CharOutBus *> cpu(mem.get());

	/*
	mem[0x600] = 0xa2;	// LDX
	mem[0x601] = 0x00;	//   #0
	mem[0x602] = 0xb5;	// LDA ZP,X
	mem[0x603] = 0x40;	//   $40
	mem[0x604] = 0x85;	// STA
	mem[0x605] = 0x0f;	//   $F
	mem[0x606] = 0xe8;	// INX
	mem[0x607] = 0xe0;	// CPX
	mem[0x608] = 0x0f;	//   #15
	mem[0x609] = 0xd0;	// BNE
	mem[0x60a] = 0xf7;	//   0x602
	*/
	(*mem)[0x600] = 0xa2;	// LDX
	(*mem)[0x601] = 0x00;	//   #0
	(*mem)[0x602] = 0xb5;	// LDA ZP,X
	(*mem)[0x603] = 0x40;	//   $40
	(*mem)[0x604] = 0xf0;	// BEQ
	(*mem)[0x605] = 0x05;	//   $0x60b
	(*mem)[0x606] = 0x85;	// STA ZP
	(*mem)[0x607] = 0x0f;	//   $F
	(*mem)[0x608] = 0xe8;	// INX
	(*mem)[0x609] = 0xd0;	// BNE
	(*mem)[0x60A] = 0xf7;  //   0x602

	std::copy(message, message + sizeof(message), &(*mem)[0x40]);

	const uint16_t resetvec = cpu.opToVector(m65xx::op::RESET);
	(*mem)[resetvec] = 0x00;
	(*mem)[resetvec + 1] = 0x06;

	cpu.setPC(0x1234);
	for (uint16_t pc = cpu.getPC(); pc != 0; pc = cpu.getPC()) {
		cpu.tick();
	}
}
