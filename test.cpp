#include <type_traits>
#include <fstream>
#include <iostream>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string_view>
#include <filesystem>
#include <algorithm>

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
	ErrorTrap = 0xff80,
	PutChar = 0xff81,
	GetChar = 0xff84,
	Done = 0xff88,
	InterruptPort = 0xff8c,
};

void dump_mem(FILE *out, const uint8_t *memory, uint16_t start, uint16_t end)
{
	for (auto i = start; i < end; ++i) {
		if (!(i & 15)) {
			fprintf(out, "\n%04X:", i);
		}
		else if (!(i & 3)) {
			fputc(' ', out);
		}
		fprintf(out, " %02X", memory[i]);
	}
	fputc('\n', out);
}

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

class MiniCIA {
public:
	// Control register bits
	  static constexpr inline uint8_t START		= 0x01;
	//static constexpr inline uint8_t PBON		= 0x02;
	//static constexpr inline uint8_t OUTMODE	= 0x04;
	  static constexpr inline uint8_t ONESHOT	= 0x08;
	  static constexpr inline uint8_t LOAD		= 0x10;
	//static constexpr inline uint8_t INMODE	= 0x20;		// only phi2 (0) supported
	//static constexpr inline uint8_t SPMODE	= 0x40;
	//static constexpr inline uint8_t TODIN		= 0x80;

	// Different bits for Control Register B
	  static constexpr inline uint8_t INMODEB	= 0x40;
	//static constexpr inline uint8_t ALARM		= 0x80;

	[[nodiscard]] uint8_t readReg(int reg)
	{
		switch (reg) {
		case 4:		return TimerA & 0xFF;
		case 5:		return TimerA >> 8;
		case 6:		return TimerB & 0xFF;
		case 7:		return TimerB >> 8;
		case 13:  { uint8_t ret = IrqData & IrqMask; IrqData = 0; return ret; }
		case 14:	return CRA;
		case 15:	return CRB;
		}
		return 0;
	}

	void writeReg(int reg, uint8_t data)
	{
		switch (reg) {
		case 4:		setTimerLo(data, TimerALatch); break;
		case 5:		setTimerHi(data, TimerALatch, CRA); break;
		case 6:		setTimerLo(data, TimerBLatch); break;
		case 7:		setTimerHi(data, TimerBLatch, CRB); break;
		case 13:	writeIrqMask(data); break;
		case 14:	CRA = data; break;
		case 15:	CRB = data; break;
		}
	}

	void tick()
	{
		runTimerA();
		runTimerB();
	}

	[[nodiscard]] uint16_t getTimerA() const
	{
		return TimerA;
	}

	[[nodiscard]] uint16_t getTimerB() const
	{
		return TimerB;
	}

	[[nodiscard]] bool getIRQB() const
	{
		return !(IrqData & IrqMask);
	}

private:
	uint16_t TimerALatch = 0, TimerA = 0;
	uint16_t TimerBLatch = 0, TimerB = 0;
	uint8_t CRA = 0, CRB = 0;
	uint8_t IrqData = 0, IrqMask = 0x81;
	bool TimerAOut = false;

	void setTimerLo(uint8_t data, uint16_t &latch)
	{
		latch = (latch & 0xFF00) | data;
	}

	void setTimerHi(uint8_t data, uint16_t &latch, uint8_t &cr)
	{
		latch = (latch & 0x00FF) | (data << 8);
		if (!(cr & START)) {
			cr |= LOAD;
		}
	}

	void writeIrqMask(uint8_t mask)
	{
		if (mask & 0x80) {
			IrqMask |= mask & 0x03;
		}
		else {
			IrqMask &= ~(mask & 0x03);
		}
		if (IrqMask & 0x7F) {
			IrqMask |= 0x80;
		}
		else {
			IrqMask = 0;
		}
	}

	void runTimerA() {
		TimerAOut = false;
		if (CRA & LOAD) {
			TimerA = TimerALatch;
			CRA &= ~LOAD;
			return;
		}
		if (CRA & START) {
			if (TimerA-- == 0) {
				TimerA = TimerALatch;
				TimerAOut = true;
				IrqData |= 0x81;
				if (CRA & ONESHOT) {
					CRA &= ~START;
				}
			}
		}
	}

	void runTimerB() {
		if (CRB & LOAD) {
			TimerB = TimerBLatch;
			CRB &= ~LOAD;
			return;
		}
		if (CRB & START) {
			if ((!(CRB & INMODEB) || TimerAOut) && TimerB-- == 0) {
				TimerB = TimerBLatch;
				IrqData |= 0x82;
				if (CRB & ONESHOT) {
					CRB &= ~START;
				}
			}
		}
	}
};

struct MiniC64Bus {
	constexpr static inline int TRAP = 2;

	constexpr static inline uint16_t FACHO = 0x62;

	constexpr static inline uint16_t FNLEN = 0xB7;
	constexpr static inline uint16_t SA = 0xB9;			// Secondary address for IO
	constexpr static inline uint16_t FNADR = 0xBB;
	constexpr static inline uint16_t PNTR = 0xD3;		// Cursor column on current line

	constexpr static inline uint16_t CBINV = 0x0316;	// BRK interrupt vector

	constexpr static inline uint16_t DONEADDR = 0x8000;	// Tests jump here when done
	//constexpr static inline uint16_t WARMVEC = 0xA002;	// BASIC warm start vector
	//constexpr static inline uint16_t READY = 0xA474;	// Enter BASIC immediate mode

	constexpr static inline uint16_t STROUT = 0xAB1E;	// Print 0-terminated string at (Y,A)
	constexpr static inline uint16_t FLOATC = 0xBC49;	// Float unsigned value in FAC+1,2
	constexpr static inline uint16_t FMULT = 0xBA28;	// Multiply FP accum with memory
	constexpr static inline uint16_t MOVAF = 0xBC0C;	// Move FP accum to FP arg
	constexpr static inline uint16_t FADDT = 0xB86A;	// Add FP arg to FP accum
	constexpr static inline uint16_t LINPRNT = 0xBDCD;	// Print XA as unsigned integer
	constexpr static inline uint16_t FOUT = 0xBDDD;		// FP accum to string at bottom of stack

	constexpr static inline uint16_t SCROLY = 0xD011;

	constexpr static inline uint16_t CIA2CRA = 0xDD0E;	// CIA 2 control register A
	constexpr static inline uint16_t CIA2CRB = 0xDD0F;	// CIA 2 control register B

	constexpr static inline uint16_t PULS = 0xFF48;		// IRQ handler
	constexpr static inline uint16_t SETNAM = 0xFDF9;	// Set filename parameters

	constexpr static inline uint16_t VEC_IOINIT = 0xFF84;
	constexpr static inline uint16_t VEC_RESTOR = 0xFF8A;	// Restore default system and interrupt vectors
	constexpr static inline uint16_t VEC_SETLFS = 0xFFBA;	// Set up a logical file
	constexpr static inline uint16_t VEC_SETNAM = 0xFFBD;
	constexpr static inline uint16_t VEC_GETIN = 0xFFE4;
	constexpr static inline uint16_t VEC_CHROUT = 0xFFD2;
	constexpr static inline uint16_t VEC_LOAD = 0xFFD5;	// Load RAM from a device

	using cputype = m65xx::M6502<MiniC64Bus *, TRAP>;

	uint8_t memory[65536]{};
	MiniCIA CIA1;
	MiniCIA CIA2;
	double fp_accum = 0;
	double fp_arg = 0;
	bool DontTrapBreak = false;

	enum class State {
		Running,
		Passed,
		Failed
	} state = State::Running;

	void tick();
	void init(cputype &cpu);
	void startTest(cputype &cpu, uint16_t loadaddr);
	int loadTest(std::string_view testname, bool reloc, uint16_t loadaddr);
	static std::string pet2ascii(uint8_t pet);
	bool breakHandler(cputype &cpu, uint16_t addr);
	bool trap(cputype &cpu, uint16_t addr);
	[[nodiscard]] uint16_t readWord(uint16_t addr) const;
	void writeWord(uint16_t addr, uint16_t data);
	uint8_t readAddr(uint16_t addr);
	void writeAddr(uint16_t addr, uint8_t data);

	[[nodiscard]] bool getIRQB() const
	{
		return CIA1.getIRQB();
	}
	[[nodiscard]] bool getNMIB() const
	{
		return CIA2.getIRQB();
	}
};

void MiniC64Bus::tick()
{
	CIA1.tick();
	CIA2.tick();
}

void MiniC64Bus::init(cputype &cpu)
{
	memory[DONEADDR] = TRAP;

	memory[VEC_SETNAM] = 0x4C;	// JMP
	writeWord(VEC_SETNAM + 1, SETNAM);

	memory[VEC_IOINIT] = 0x60;		// RTS because not relevant here

	for (auto traploc :
		{ VEC_RESTOR, VEC_LOAD, VEC_CHROUT,
			FLOATC, FMULT, MOVAF, FADDT, FOUT, STROUT, LINPRNT }) {
		memory[traploc] = TRAP;
		memory[traploc + 1] = 0x60;	// RTS
	}

	// The secondary address, which indicates whether to relocate
	// the load, is the only thing we care about for SETLFS.
	memory[VEC_SETLFS] = 0x84;		// STY zp
	memory[VEC_SETLFS + 1] = SA;
	memory[VEC_SETLFS + 2] = 0x60;	// RTS

	// Make GETIN always return #3
	memory[VEC_GETIN] = 0xA9;	// LDA #
	memory[VEC_GETIN + 1] = 3;
	memory[VEC_GETIN + 2] = 0x60;	// RTS

	// Copy SETNAM routine
	static const uint8_t setnam[] = {
		0x85, FNLEN,		// STA FNLEN
		0x86, FNADR,		// STX FNADR
		0x84, FNADR + 1,	// STY FNADR+1
		0x60				// RTS
	};
	std::copy(setnam, setnam + sizeof(setnam), &memory[SETNAM]);

	// Setup vectors
	//writeWord(WARMVEC, FAILADDR);
	writeWord(cpu.opToVector(m65xx::op::IRQ), PULS);
	writeWord(CBINV, DONEADDR);

	// Copy IRQ handler
	static const uint8_t puls[] = {
		0x48,				// PHA
		0x8A,				// TXA
		0x48,				// PHA
		0x98,				// TYA
		0x48,				// PHA
		0xBA,				// TSX
		0xBD, 0x04, 0x01,	// LDA $0104,X
		0x29, 0x10,			// AND #$10
		0xF0, 0x03,			// BEQ $FF58
		0x6C, 0x16, 0x03,	// JMP ($0316)
		0x6C, 0x14, 0x03,	// JMP ($0314)
	};
	std::copy(puls, puls + sizeof(puls), &memory[PULS]);
}

void MiniC64Bus::startTest(cputype &cpu, uint16_t loadaddr)
{
	init(cpu);

	// Set up top of stack so RTS goes to DONEADDR
	writeWord(0x1FE, DONEADDR - 1);
	cpu.setSP(0x1FD);
	cpu.setP(m65xx::FLAG_I);

	// Find the real program immediately after the BASIC stub.
	auto addr = readWord(loadaddr);
	while (auto next = readWord(addr)) {
		if (next != 0) {
			addr = next;
		}
	}
	// Start is the first non-0 byte
	addr += 2;
	while (memory[addr] == 0) {
		++addr;
	}
	cpu.setPC(addr);
}

// Returns address file was loaded to, or -1 on failure.
int MiniC64Bus::loadTest(std::string_view testname, bool reloc, uint16_t loadaddr)
{
	try {
		std::filesystem::path path = "tests/bin/lorenz";
		// Convert file name from PETSCII to ASCII. In practice, this
		// amounts to converting from uppercase to uppercase.
		std::string name_ascii;
		name_ascii.reserve(testname.size());
		for (auto c : testname) {
			name_ascii += pet2ascii(c);
		}
		path /= name_ascii;
		path += ".prg";

		// The TRAP* tests expect to get a BRK sequence.
		DontTrapBreak = testname.compare(0, 4, "TRAP", 4) == 0;

		auto size = std::filesystem::file_size(path) - 2;
		std::ifstream input(path, std::ios::binary);
		uint8_t loadlo, loadhi;
		input >> loadlo >> loadhi;
		if (!reloc) {
			loadaddr = (loadhi << 8) | loadlo;
		}
		size = std::min<decltype(size)>(65536 - loadaddr, size);
		if (input.read(reinterpret_cast<char *>(&memory[loadaddr]), size)) {
			return loadaddr;
		}
	}
	catch (const std::runtime_error &error) {
		std::cerr << error.what() << '\n';
	}
	return -1;
}

std::string MiniC64Bus::pet2ascii(uint8_t pet)
{
	if (pet == 147/* Clear screen */ || pet == 14/* Switch to lower case */) {
		return "";
	}
	if (pet == 145) {	// Cursor up
		return "\33[A";
	}
	if (pet >= 'A' && pet <= 'Z') {
		return std::string(1, pet + ('a' - 'A'));
	}
	if (pet >= 'a' && pet <= 'z') {
		return std::string(1, pet + ('A' - 'a'));
	}
	if (pet >= 0xC1 && pet <= 0xDA) {
		return std::string(1, pet - (0xC1 - 'A'));
	}
	if (pet == '\r') {
		return "\n";
	}
	return std::string(1, pet);
}

bool MiniC64Bus::breakHandler(cputype &cpu, uint16_t addr)
{
	if (!DontTrapBreak && addr >= DONEADDR) {
		fprintf(stderr, "Unhandled routine at $%04X\n", addr);
		fprintf(stderr, "Zero page dump:\n");
		dump_mem(stderr, memory, 0, 256);
		state = State::Failed;
		return true;
	}
	return false;
}

bool MiniC64Bus::trap(cputype &cpu, uint16_t addr)
{
	switch (addr) {
	//case READY: 
//			state = State::Passed;
//		return true;

	case DONEADDR:
		state = State::Failed;
		return true;

	case VEC_RESTOR:
		init(cpu);
		return true;

	case VEC_LOAD: {
		std::string_view test{ reinterpret_cast<char *>(&memory[readWord(FNADR)]), memory[FNLEN] };
		if (loadTest(test, !!memory[SA], (cpu.getX() << 8) | cpu.getY()) < 0) {
			std::cerr << "Failed loading " << test << '\n';
			state = State::Failed;
		}
		return true;
	}

	case VEC_CHROUT:
		std::cout << pet2ascii(cpu.getA());
		return true;

	// Various BASIC routines cputiming uses to print a 32-bit number
	case FLOATC:
		// FACHO[0] and FACHO[1] contain an unsigned 16-bit integer
		// in big-endian order. X contains the MBF (Microsoft Binary
		// Format) exponent. C is set for a positive number, clear for
		// a negative number. We store the result in a private
		// accumulator in the host's native floating point format
		// rather than the zero page MBF accumulator.
		fp_accum = ((memory[FACHO] << 8) | memory[FACHO + 1]) * std::exp2(cpu.getX() - (128 + 16));
		if (!(cpu.getP() & m65xx::FLAG_C)) {
			fp_accum = -fp_accum;
		}
		return true;

	case FMULT: {
		static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
		// A is the low byte of the memory arg's address
		// Y is the high byte of the memory arg's address
		//
		// On the C64, this is a 5-byte MBF floating point number,
		// which is very similar to IEEE-754. The first byte is the
		// exponent, 128 biased. The next bit is the sign. Then the next
		// 32 bits are are the mantissa, in big-endian format. There is an
		// implicit 1 just after the radix point (compared to just before
		// it for IEEE-754).
		uint16_t addr = (cpu.getY() << 8) | cpu.getA();
		const uint8_t *mbf = &memory[addr];
		if (mbf[0] == 0) {
			fp_arg = 0;
		} else {
			uint64_t fpbits =
				(uint64_t(mbf[1] & 0x80) << 56)
				| (uint64_t(mbf[0] + (1023 - 129)) << 52)
				| (uint64_t(mbf[1] & 0x7F) << 45)
				| (uint64_t(mbf[2]) << 37)
				| (uint64_t(mbf[3]) << 29)
				| (uint64_t(mbf[4]) << 21);
			std::memcpy(&fp_arg, &fpbits, 8);
			}
		fp_accum *= fp_arg;
		return true;
	}

	case MOVAF:
		fp_arg = fp_accum;
		return true;

	case FADDT:
		fp_accum += fp_arg;
		return true;

	case FOUT:
		snprintf(reinterpret_cast<char *>(&memory[0x100]), 32, "% g", fp_accum);
		cpu.setY(1);
		cpu.setA(0);
		return true;

	case STROUT:
		printf("%s", &memory[(cpu.getY() << 8) | cpu.getA()]);
		return true;

	case LINPRNT:
		printf("%u", (cpu.getA() << 8) | cpu.getX());
		return true;
	}
	return false;
}

[[nodiscard]] uint16_t MiniC64Bus::readWord(uint16_t addr) const
{
	return memory[addr] + (memory[addr + 1] << 8);
}

void MiniC64Bus::writeWord(uint16_t addr, uint16_t data)
{
	memory[addr] = data & 0xFF;
	memory[addr + 1] = data >> 8;
}

uint8_t MiniC64Bus::readAddr(uint16_t addr)
{
	if (addr == SCROLY) {
		// Always say we're inside the border
		return 0x80;
	}
	if ((addr & 0xFF00) == 0xDC00) {
		return CIA1.readReg(addr & 15);
	}
	if ((addr & 0xFF00) == 0xDD00) {
		return CIA2.readReg(addr & 15);
	}
	return memory[addr];
}

void MiniC64Bus::writeAddr(uint16_t addr, uint8_t data)
{
	if (addr == PNTR) {
		std::cout << '\r';
		if (data != 0) {
			printf("\33[%uC", data);
		}
	}
	else if ((addr & 0xFF00) == 0xDC00) {
		CIA1.writeReg(addr & 15, data);
	}
	else if ((addr & 0xFF00) == 0xDD00) {
		CIA2.writeReg(addr & 15, data);
	}
	else {
		memory[addr] = data;
	}
}

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
	while (bus.last_addr != static_cast<uint16_t>(IO::Done)) {
		cpu.tick();
		if (bus.last_addr == static_cast<uint16_t>(IO::ErrorTrap)) {
			bus.dump_mem(zero_page, zp_bss_end);
			bus.dump_mem(data_segment, data_bss_end);
			[[maybe_unused]] int i = 0;	// Error!
		}
		if (false && cpu.getSync()) {
			printf("A=%02X X=%02X Y=%02X S=%02X ",
				cpu.getA(), cpu.getX(), cpu.getY(), cpu.getSP());
			print_p(cpu.getP());
			printf("  %s\n", cpu.disasmOp(cpu.getPC() - 1, true).c_str());
		}
	}
}

void run_interrupt_test()
{
	const uint16_t zero_page = 0;
	const uint16_t zp_bss = 6;

	const uint16_t data_segment = 0x200;
	const uint16_t data_bss = 0x204;

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
	while (bus.last_addr != static_cast<uint16_t>(IO::Done)) {
		cpu.tick();
#if 0
		if (bus.last_addr == static_cast<uint16_t>(IO::ErrorTrap)) {
			bus.dump_mem(zero_page, zp_bss);
			bus.dump_mem(data_segment, data_bss);
			[[maybe_unused]] int i = 0;	// Error!
		}
		if (1 && cpu.getSync()) {
			printf("A=%02X X=%02X Y=%02X S=%02X ",
				cpu.getA(), cpu.getX(), cpu.getY(), cpu.getSP());
			print_p(cpu.getP());
			printf("  %s\n", cpu.disasmOp(cpu.getPC() - 1, true).c_str());
		}
#endif
	}
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

void run_lorenz_tests()
{
	printf("Running Wolfgang Lorenz's test suite\n");
	MiniC64Bus bus;
	MiniC64Bus::cputype cpu(&bus);
	if (auto loadaddr = bus.loadTest("START", false, 0)) {
		bus.startTest(cpu, loadaddr);
		while (bus.state == MiniC64Bus::State::Running) {
			cpu.tick();
			bus.tick();
			if (false && cpu.getSync()) {
				printf("A=%02X X=%02X Y=%02X S=%02X",
					cpu.getA(), cpu.getX(), cpu.getY(), cpu.getSP());
				print_p(cpu.getP());
				printf(" T1A=%-5u %s\n", bus.CIA1.getTimerA(),
					cpu.disasmOp(cpu.getPC() - 1, true).c_str());
			}
		}
		if (bus.state == MiniC64Bus::State::Failed) {
			std::cout << "  Failed\n";
		}
		else {
			std::cout << "  Ok\n";
		}
	}
}

int main()
{
	run_functional_test();
	run_decimal_test();
	run_interrupt_test();
	run_lorenz_tests();
	return 0;
}