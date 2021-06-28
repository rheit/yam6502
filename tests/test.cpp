#include <cstdio>
#include <cerrno>
#include <cstring>
#include <iostream>

#include "yam6502.h"
#include "test.h"

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

void print_p(unsigned p)
{
	printf("P=%02X [%c%c%c%c%c%c]",
		p & 0xFF,
		p & yam::FLAG_N ? 'N' : '.',
		p & yam::FLAG_V ? 'V' : '.',
		p & yam::FLAG_D ? 'D' : '.',
		p & yam::FLAG_I ? 'I' : '.',
		p & yam::FLAG_Z ? 'Z' : '.',
		p & yam::FLAG_C ? 'C' : '.'
	);
}

void pfileerror(std::filesystem::path path, const char *errmsg)
{
	puts(TESTS_BIN);
	std::cerr << errmsg << ": " << strerror(errno) << ' ' << path << '\n';
}

int main()
{
	run_functional_test();
	run_decimal_test();
	run_interrupt_test();
	run_lorenz_tests();
	return 0;
}
