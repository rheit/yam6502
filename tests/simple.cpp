#include <fstream>
#include <iostream>
#include <chrono>

#include "yam6502.h"
#include "test.h"

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
	ErrorTrap = 0xff80,
	PutChar = 0xff81,
	GetChar = 0xff84,
	Done = 0xff88,
	InterruptPort = 0xff8c,
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
		::dump_mem(stdout, memory, start, end);
	}
#if 0
	void syncHandler(m65xx::M6502<TestBus *> &cpu, uint16_t pc)
	{
		printf("A=%02X X=%02X Y=%02X S=%02X ",
			cpu.getA(), cpu.getX(), cpu.getY(), cpu.getSP());
		print_p(cpu.getP());
		printf("  %s\n", cpu.disasmOp(pc, true).c_str());
	}
#endif
	uint8_t memory[65536]{};
	uint16_t last_addr = ~0;
};

struct TestBusWithInterrupts : public TestBus {
	[[nodiscard]] bool getIRQB() const
	{
		return !(memory[static_cast<int>(IO::InterruptPort)] & 1);
	}
	[[nodiscard]] bool getNMIB() const
	{
		return !(memory[static_cast<int>(IO::InterruptPort)] & 2);
	}
};

void run_functional_test()
{
	const uint16_t zero_page = 0;
	const uint16_t zp_bss_end = 0x52;

	const uint16_t data_segment = 0x200;
	const uint16_t data_bss_end = 0x27b;

	printf("Running functional tests\n");
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
	unsigned long long clocks = 0;
	auto start = std::chrono::steady_clock::now();
	while (bus.last_addr != static_cast<uint16_t>(IO::Done)) {
		clocks++;
		cpu.tick();
		if (bus.last_addr == static_cast<uint16_t>(IO::ErrorTrap)) {
			bus.dump_mem(zero_page, zp_bss_end);
			bus.dump_mem(data_segment, data_bss_end);
			[[maybe_unused]] int i = 0;	// Error!
		}
	}
	std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
	std::cout << clocks << " cycles in " << diff.count() << " sec ("
		<< clocks / diff.count() / 1000000 << " MHz)\n";
}

void run_interrupt_test()
{
	const uint16_t code_segment = 0x400;

	printf("Running interrupt tests\n");
	TestBusWithInterrupts bus;
	{
		std::ifstream input("tests/bin/6502_interrupt_test.bin", std::ios::binary);
		if (input.read(reinterpret_cast<char *>(bus.memory + code_segment), 65536 - code_segment) && input.bad()) {
			std::cerr << "Failed reding 6502_interrupt_test.bin\n";
			return;
		}
	}
	m65xx::M6502<decltype(&bus)> cpu(&bus);
	cpu.setPC(code_segment);
	unsigned long long clocks = 0;
	auto start = std::chrono::steady_clock::now();
	while (bus.last_addr != static_cast<uint16_t>(IO::Done)) {
		++clocks;
		cpu.tick();
#if 0
		const uint16_t zero_page = 0;
		const uint16_t zp_bss = 6;

		const uint16_t data_segment = 0x200;
		const uint16_t data_bss = 0x204;

		if (bus.last_addr == static_cast<uint16_t>(IO::ErrorTrap)) {
			bus.dump_mem(zero_page, zp_bss);
			bus.dump_mem(data_segment, data_bss);
			[[maybe_unused]] int i = 0;	// Error!
		}
#endif
	}
	std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
	std::cout << clocks << " cycles in " << diff.count() << " sec ("
		<< clocks / diff.count() / 1000000 << " MHz)\n";
}

void run_decimal_test()
{
	const auto decimal_org = 0x200;
	enum {
		N1, N2, HA, HNVZC, DA, DNVZC, AR, NF, VF, ZF, CF, ERROR
	};

	printf("Running decimal tests\n");
	TestBus bus;
	{
		std::ifstream input("tests/bin/6502_decimal_test.bin", std::ios::binary | std::ios::in);
		if (!input.read(reinterpret_cast<char *>(bus.memory + decimal_org), 65536 - decimal_org) && input.bad()) {
			std::cerr << "Failed reading 6502_decimal_test.bin\n";
			return;
		}
	}
	m65xx::M6502<TestBus *> cpu(&bus);
	cpu.setPC(decimal_org);
	unsigned long long clocks = 0;
	auto start = std::chrono::steady_clock::now();
	while (bus.last_addr != static_cast<uint16_t>(IO::Done)) {
		++clocks;
		cpu.tick();
	}
	std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
	std::cout << clocks << " cycles in " << diff.count() << " sec ("
		<< clocks / diff.count() / 1000000 << " MHz)\n";
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
