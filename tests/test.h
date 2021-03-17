#include <filesystem>

void dump_mem(FILE *out, const uint8_t *memory, uint16_t start, uint16_t end);
void print_p(unsigned p);
void pfileerror(std::filesystem::path path, const char *errmsg);

void run_functional_test();
void run_interrupt_test();
void run_decimal_test();

void run_lorenz_tests();
