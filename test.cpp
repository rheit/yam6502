#include <type_traits>
#include <fstream>
#include <iostream>
#include <conio.h>
#include <cerrno>

#include "yam6502.h"

enum class IO : uint16_t {
	GetChar = 0xff84,
	PutChar = 0xff81,
	Done = 0xff88,
	ErrorTrap = 0xff80
};

struct TestBus {
	uint8_t readAddr(uint16_t addr)
	{
		last_addr = addr;
		if (addr == static_cast<uint16_t>(IO::GetChar)) {
			return static_cast<uint8_t>(_getch());
		}
		return memory[addr];
	}
	void writeAddr(uint16_t addr, uint8_t val)
	{
		last_addr = addr;
		if (addr == static_cast<uint16_t>(IO::PutChar)) {
			std::cout << static_cast<char>(val);
			return;
		}
		memory[addr] = val;
	}
	void dump_mem(uint16_t start, uint16_t end) const
	{
		for (auto i = start; i < end; ++i) {
			if (!(i & 15)) {
				printf("\n%04X:", i);
			}
			else if (!(i & 3)) {
				printf(" ");
			}
			printf(" %02X", memory[i]);
		}
		printf("\n");
	}
	uint8_t memory[65536];
	uint16_t last_addr = ~0;
};

#define zero_page 0
#define zp_bss_end 0x52

#define data_segment 0x200
#define data_bss_end 0x27b

void run_functional_test()
{
	TestBus bus;
	{
		std::basic_ifstream<uint8_t> input("tests/bin/6502_functional_test.bin", std::ios_base::binary);
		if (input.read(bus.memory, 65536).gcount() != 65536) {
			std::cerr << "Failed reading 65536 bytes from 6502_functional_test.bin\n";
		}
	}
	m65xx::M6502<TestBus *> cpu(&bus);
	cpu.setPC(0x400);
	while (bus.last_addr != static_cast<uint16_t>(IO::Done)) {
		cpu.tick();
		if (bus.last_addr == static_cast<uint16_t>(IO::ErrorTrap)) {
			bus.dump_mem(zero_page, zp_bss_end);
			bus.dump_mem(data_segment, data_bss_end);
			int i = 0;	// Error!
		}
		if (0 && cpu.getSync()) {
			/*
			auto p = cpu.getP();
			printf("A=%02X X=%02X Y=%02X S=%02X P=%02X [%c%c%c%c%c%c]  %s\n",
				cpu.getA(),
				cpu.getX(),
				cpu.getY(),
				cpu.getSP(),
				p,
				p & m65xx::FLAG_N ? 'N' : '.',
				p & m65xx::FLAG_V ? 'V' : '.',
				p & m65xx::FLAG_D ? 'D' : '.',
				p & m65xx::FLAG_I ? 'I' : '.',
				p & m65xx::FLAG_Z ? 'Z' : '.',
				p & m65xx::FLAG_C ? 'C' : '.',
				cpu.disasmOp(cpu.getPC() - 1, true).c_str());
				*/
		}
	}
}

int main()
{
	run_functional_test();
	return 0;
}