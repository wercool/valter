#define FASTRUN __attribute__ ((long_call, section (".fastrun")))

extern FASTRUN int flashWrite(unsigned address, const void *data, unsigned size);
