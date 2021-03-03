#include <type_traits>
#include <fstream>
#include <iostream>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string_view>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <string>

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
		case 4:		return TimerA.Counter & 0xFF;
		case 5:		return TimerA.Counter >> 8;
		case 6:		return TimerB.Counter & 0xFF;
		case 7:		return TimerB.Counter >> 8;
		case 13:	return readIrqData();
		case 14:	return TimerA.Control;
		case 15:	return TimerB.Control;
		}
		return 0;
	}

	void writeReg(int reg, uint8_t data)
	{
		switch (reg) {
		case 4:		TimerA.setLo(data); break;
		case 5:		TimerA.setHi(data); break;
		case 6:		TimerB.setLo(data); break;
		case 7:		TimerB.setHi(data); break;
		case 13:	writeIrqMask(data); break;
		case 14:	TimerA.writeControl(data); break;
		case 15:	TimerB.writeControl(data); break;
		}
	}

	void tick()
	{
		bool a_out = TimerA.tick(*this, 1);
		// If timer B in cascade mode, stick the output of timer A
		// into its pipeline.
		if (a_out && (TimerB.Control & INMODEB)) {
			TimerB.Delay |= Count1;
		}
		TimerB.tick(*this, 2);

		// Set interrupt line
		if (IrqDelay & 2) {
			Interrupt = true;
		}

		IrqDelay <<= 1;
	}

	void reset()
	{
		TimerA.reset();
		TimerB.reset();
		IrqData = 0;
		IrqMask = 0x81;
		IrqDelay = 0;
		Interrupt = false;
	}

	[[nodiscard]] uint16_t getTimerA() const;
	[[nodiscard]] uint16_t getTimerB() const;
	[[nodiscard]] uint8_t getTimerADelay() const;
	[[nodiscard]] uint8_t getTimerBDelay() const;

	[[nodiscard]] bool getIRQB() const
	{
		return !Interrupt;
	}

	[[nodiscard]] uint8_t getICR() const;

private:
	uint8_t IrqData = 0;
	uint8_t IrqMask = 0x81;		// As set by the C64 KERNAL
	uint8_t IrqDelay = 0;
	bool Interrupt = false;

	static constexpr inline uint8_t Count0    =   0x01;
	static constexpr inline uint8_t Count1    =   0x02;
	static constexpr inline uint8_t Count2    =   0x04;
	static constexpr inline uint8_t Count3    =   0x08;
	static constexpr inline uint8_t Load0     =   0x10;
	static constexpr inline uint8_t Load1     =   0x20;
	static constexpr inline uint8_t OneShot0  =   0x40;
	static constexpr inline uint8_t DelayMask = ~(Count0 | Load0 | OneShot0);

	struct Timer
	{
		uint16_t Latch = 0xFFFF;
		uint16_t Counter = 0;
		uint8_t Control = 0;
		uint8_t Delay = 0, Feed = 0;

		void reset()
		{
			Latch = 0xFFFF;
			Counter = 0;
			Control = 0;
			Delay = Feed = 0;
		}

		void setLo(uint8_t data)
		{
			Latch = (Latch & 0xFF00) | data;
		}

		void setHi(uint8_t data)
		{
			Latch = (Latch & 0x00FF) | (data << 8);
			// Load counter if timer is stopped
			if (!(Control & START)) {
				Delay |= Load0;
			}
		}

		void writeControl(uint8_t data)
		{
			// Start/stop timer
			if (data & START && !(data & INMODEB)) {
				Delay |= Count1 | Count0;
				Feed |= Count0;
			}
			else {
				Delay &= ~(Count1 | Count0);
				Feed &= ~Count0;
			}

			// Set/clear one shot mode
			if (data & ONESHOT) {
				Feed |= OneShot0;
			}
			else {
				Feed &= ~OneShot0;
			}

			// Set force load
			if (data & LOAD) {
				Delay |= Load0;
			}

			Control = data;
		}

		bool tick(MiniCIA &cia, const uint8_t icrbit)
		{
			// Assume no underflow
			bool out = false;

			if (Delay) {
				// Decrement counter
				if (Delay & Count3) {
					Counter--;
				}

				// Check counter underflow
				if (Counter == 0 && (Delay & Count2)) {
					// Signal underflow
					cia.IrqData |= icrbit;

					// Underflow interrupt in next clock
					if (cia.IrqMask & icrbit) {
						cia.IrqDelay |= 1;
					}

					// Stop timer in one shot mode
					if ((Delay | Feed) & OneShot0) {
						Control &= ~START;
						Delay &= ~(Count2 | Count1 | Count0);
						Feed &= ~Count0;
					}

					// Signal underflow externally
					out = true;

					// Reload counter
					Delay |= Load1;
				}

				// Load counter
				if (Delay & Load1) {
					Counter = Latch;

					// Don't decrement counter in next clock
					Delay &= ~Count2;
				}
			}
			// Advance pipeline for next clock
			Delay = ((Delay << 1) & DelayMask) | Feed;

			return out;
		}
	};

	Timer TimerA, TimerB;

	[[nodiscard]] uint8_t readIrqData()
	{
		uint8_t icr = IrqData | (Interrupt << 7);

		// Clear interrupt and discard any pending
		Interrupt = false;
		IrqDelay = 0;

		IrqData = 0;
		return icr;
	}

	void writeIrqMask(uint8_t mask)
	{
		if (mask & 0x80) {
			IrqMask |= mask & 0x1F;
		}
		else {
			IrqMask &= ~(mask & 0x1F);
		}
		// Raise an interrupt in the next cycle if condition matches
		if ((IrqMask & IrqData) && !Interrupt) {
			IrqDelay |= 1;
		}
	}
};

[[nodiscard]] uint16_t MiniCIA::getTimerA() const
{
	return TimerA.Counter;
}

[[nodiscard]] uint16_t MiniCIA::getTimerB() const
{
	return TimerB.Counter;
}

[[nodiscard]] uint8_t MiniCIA::getTimerADelay() const
{
	return TimerA.Delay;
}

[[nodiscard]] uint8_t MiniCIA::getTimerBDelay() const
{
	return TimerB.Delay;
}

[[nodiscard]] uint8_t MiniCIA::getICR() const
{
	return IrqData | (Interrupt << 7);
}


struct MiniC64Bus {
	constexpr static inline int TRAP = 2;

	constexpr static inline uint16_t FACHO = 0x62;

	constexpr static inline uint16_t FNLEN = 0xB7;
	constexpr static inline uint16_t SA = 0xB9;			// Secondary address for IO
	constexpr static inline uint16_t FNADR = 0xBB;
	constexpr static inline uint16_t PNTR = 0xD3;		// Cursor column on current line

	constexpr static inline uint16_t CBINV = 0x0316;	// BRK interrupt vector

	constexpr static inline uint16_t READY = 0xA474;	// Enter BASIC immediate mode

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

	constexpr static inline uint16_t TIMB = 0xFE66;		// BRK handler
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
	unsigned long long clocks = 0;

	enum class State {
		Running,
		Passed,
		Failed
	} state = State::Running;

	void tick();
	void init(cputype &cpu);
	void startTest(cputype &cpu, uint16_t loadaddr);
	int loadTest(std::string_view testname, bool reloc, uint16_t loadaddr);
	void charOut(uint8_t pet);
	static std::string pet2ascii(uint8_t pet);
	bool breakHandler(cputype &cpu, uint16_t addr);
	bool trap(cputype &cpu, uint16_t addr);
	[[nodiscard]] uint16_t readWord(uint16_t addr) const;
	void writeWord(uint16_t addr, uint16_t data);
	uint8_t readAddr(uint16_t addr);
	void writeAddr(uint16_t addr, uint8_t data);

	[[nodiscard]] uint8_t readNoSideEffects(uint16_t addr) const
	{
		return memory[addr];
	}

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
	++clocks;
}

void MiniC64Bus::init(cputype &cpu)
{
	CIA1.reset();
	CIA2.reset();

	// Set traps for several ROM routines
	for (auto traploc :
		{ TIMB, READY, VEC_RESTOR, VEC_LOAD, VEC_CHROUT,
			FLOATC, FMULT, MOVAF, FADDT, FOUT, STROUT, LINPRNT }) {
		memory[traploc] = TRAP;
		memory[traploc + 1] = 0x60;	// RTS
	}

	memory[VEC_SETNAM] = 0x4C;		// JMP
	writeWord(VEC_SETNAM + 1, SETNAM);

	memory[VEC_IOINIT] = 0x60;		// RTS because not relevant here

	// The secondary address, which indicates whether to relocate
	// the load, is the only thing we care about for SETLFS.
	memory[VEC_SETLFS] = 0x84;		// STY zp
	memory[VEC_SETLFS + 1] = SA;
	memory[VEC_SETLFS + 2] = 0x60;	// RTS

	// Make GETIN always return a space character
	memory[VEC_GETIN] = 0xA9;		// LDA #
	memory[VEC_GETIN + 1] = ' ';
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
	writeWord(cpu.opToVector(m65xx::op::IRQ), PULS);
	writeWord(CBINV, TIMB);

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

	// Set up top of stack so RTS goes to READY
	writeWord(0x1FE, READY - 1);
	cpu.setSP(0x1FD);
	cpu.setP(0);

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
		// amounts to converting from uppercase to lowercase.
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
	if (!DontTrapBreak && addr >= 0xA000) {
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
	case TIMB:
		state = State::Failed;
		return true;

	case READY:
		state = State::Passed;
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

	// Various BASIC routines used to print 16- and 32-bit decimal numbers
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
		snprintf(reinterpret_cast<char *>(&memory[0x100]), 32, "% .f", fp_accum);
		cpu.setY(1);
		cpu.setA(0);
		return true;

	case LINPRNT:
		snprintf(reinterpret_cast<char *>(&memory[0x100]), 32, "%u", (cpu.getA() << 8) | cpu.getX());
		cpu.setY(1);
		cpu.setA(0);
		[[fallthrough]];

	case STROUT:
		std::cout << reinterpret_cast<char *>(&memory[(cpu.getY() << 8) | cpu.getA()]);
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
#if 0
	auto irqb = getIRQB();
	auto nmib = getNMIB();
	auto icr1 = CIA1.getICR();
	auto icr2 = CIA2.getICR();
	printf("Read %04x, IRQB=%d NMIB=%d ICR1=%02x ICR2=%02x", addr, irqb, nmib, icr1, icr2);
	if (!nmib)
		printf("*************************************************");
	printf("\n");
#endif
	if (addr == SCROLY) {
		// Always say we're inside the border, since we're not emulating any VIC-II DMA
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
#if 0
		if (cpu.getSync()) {
			printf("A=%02X X=%02X Y=%02X S=%02X ",
				cpu.getA(), cpu.getX(), cpu.getY(), cpu.getSP());
			print_p(cpu.getP());
			printf("  %s\n", cpu.disasmOp(cpu.getPC() - 1, true).c_str());
		}
#endif
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
		if (1 && cpu.getSync()) {
			printf("A=%02X X=%02X Y=%02X S=%02X ",
				cpu.getA(), cpu.getX(), cpu.getY(), cpu.getSP());
			print_p(cpu.getP());
			printf("  %s\n", cpu.disasmOp(cpu.getPC() - 1, true).c_str());
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

void run_lorenz_tests()
{
	printf("Running Wolfgang Lorenz's test suite\n");
	MiniC64Bus bus;
	MiniC64Bus::cputype cpu(&bus);
	cpu.EmulateNMIBRKBug = true;
	if (auto loadaddr = bus.loadTest("START", false, 0)) {
		bus.startTest(cpu, loadaddr);
		auto start = std::chrono::steady_clock::now();
		while (bus.state == MiniC64Bus::State::Running) {
			cpu.tick();
			bus.tick();
#if 0
			if (cpu.getSync()) {
				if (auto pc = cpu.getPC() - 1;
//					!(pc >= 0xa80 && pc < 0xa8f) // inside savestack (irq.prg)
//					&& !(pc >= 0xa90 && pc < 0xaaf) // inside restorestack (irq.prg)
					!(pc >= 0xa4b && pc < 0xa5a) // inside savestack (nmi.prg)
					&& !(pc >= 0xa5b && pc < 0xa7a) // inside restorestack (nmi.prg)
					) {
					printf("A=%02X X=%02X Y=%02X S=%02X ",
						cpu.getA(), cpu.getX(), cpu.getY(), cpu.getSP());
					print_p(cpu.getP());
					printf(" T=[{%02x}%-5u {%02x}%-5u {%02x}%-5u {%02x}%-5u] %s\n",
						bus.CIA1.getTimerADelay(),
						bus.CIA1.getTimerA(),
						bus.CIA1.getTimerBDelay(),
						bus.CIA1.getTimerB(),
						bus.CIA2.getTimerADelay(),
						bus.CIA2.getTimerA(),
						bus.CIA2.getTimerBDelay(),
						bus.CIA2.getTimerB(),
						cpu.disasmOp(pc, true).c_str());
				}
			}
#endif
		}
		std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
		std::cout << bus.clocks << " cycles in " << diff.count() << " sec ("
			<< bus.clocks / diff.count() / 1000000 << " MHz)\n";
		if (bus.state == MiniC64Bus::State::Failed) {
			std::cout << "  Failed\n";
		}
		else {
			std::cout << "  Ok\n";
		}
		std::cout << "Exit code " << static_cast<int>(bus.memory[0xd7ff]) << '\n';
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