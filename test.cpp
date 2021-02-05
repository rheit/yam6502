#include <type_traits>
#include <fstream>
#include <iostream>
#include <cerrno>

#include "yam6502.h"

#ifdef _WIN32
#include <conio.h>
#else
#include <unistd.h>
#include <termios.h>

char _getch()
{
	char buf = 0;
	struct termios old {};
	fflush(stdout);
	if (tcgetattr(0, &old) < 0)
		perror("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0)
		perror("tcsetattr ICANON");
	if (read(0, &buf, 1) < 0)
		perror("read()");
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0)
		perror("tcsetattr ~ICANON");
	return buf;
}
#endif

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

void print_p(unsigned p)
{
	printf("P=%02X [%c%c%c%c%c%c]",
		p & 0xFF,
		p & m65xx::FLAG_N ? 'N' : '.',
		p & m65xx::FLAG_V ? 'V' : '.',
		p & m65xx::FLAG_D ? 'D' : '.',
		p & m65xx::FLAG_I ? 'I' : '.',
		p & m65xx::FLAG_Z ? 'Z' : '.',
		p & m65xx::FLAG_C ? 'C' : '.'
	);
}

void run_functional_test()
{
	const uint16_t zero_page = 0;
	const uint16_t zp_bss_end = 0x52;

	const uint16_t data_segment = 0x200;
	const uint16_t data_bss_end = 0x27b;

	TestBus bus;
	{
		std::ifstream input("tests/bin/6502_functional_test.bin", std::ios::binary);
		if (input.read(reinterpret_cast<char *>(bus.memory), 65536).gcount() != 65536) {
			std::cerr << "Failed reading 65536 bytes from 6502_functional_test.bin\n";
			return;
		}
	}
	m65xx::M6502<TestBus *> cpu(&bus);
	cpu.setPC(0x400);
	while (bus.last_addr != static_cast<uint16_t>(IO::Done)) {
		cpu.tick();
		if (bus.last_addr == static_cast<uint16_t>(IO::ErrorTrap)) {
			bus.dump_mem(zero_page, zp_bss_end);
			bus.dump_mem(data_segment, data_bss_end);
			[[maybe_unused]] int i = 0;	// Error!
		}
		if (0 && cpu.getSync()) {
			printf("A=%02X X=%02X Y=%02X S=%02X ",
				cpu.getA(), cpu.getX(), cpu.getY(), cpu.getSP());
			print_p(cpu.getP());
			printf("  %s\n", cpu.disasmOp(cpu.getPC() - 1, true).c_str());
		}
	}
}

void run_decimal_test()
{
	const auto decimal_org = 0x200;
	enum {
		N1, N2, HA, HNVZC, DA, DNVZC, AR, NF, VF, ZF, CF, ERROR
	};

	TestBus bus;
	{
		std::ifstream input("tests/bin/6502_decimal_test.bin", std::ios::binary | std::ios::in);
		if (!input.read(reinterpret_cast<char *>(bus.memory + decimal_org), 65536 - decimal_org) && input.bad()) {
			std::cerr << "Failed reading 6502_decimal_test.bin\n";
		}
	}
	m65xx::M6502<TestBus *> cpu(&bus);
	cpu.setPC(decimal_org);
	while (bus.last_addr != static_cast<uint16_t>(IO::Done)) {
		cpu.tick();
	}
	if (bus.memory[ERROR]) {
		printf("ERROR for $%02X + $%02x + %d:\n", bus.memory[N1], bus.memory[N2], cpu.getY());
		auto expect_p = bus.memory[NF] | bus.memory[VF] | bus.memory[ZF] | bus.memory[CF];
		printf("   Expected: A=$%02X, ", bus.memory[AR]);
		print_p(expect_p);
		printf("\n Got binary: A=$%02X, ", bus.memory[HA]);
		print_p(bus.memory[HNVZC]);
		printf("\nGot decimal: A=$%02X, ", bus.memory[DA]);
		print_p(bus.memory[DNVZC]);
		printf("\n");
	}
}

int main()
{
	run_functional_test();
	run_decimal_test();
	return 0;
}