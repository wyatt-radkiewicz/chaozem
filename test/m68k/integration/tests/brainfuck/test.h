#ifndef TEST_H
#define TEST_H

#include <stdint.h>

struct Input {
    uint8_t code[256];
    uint8_t in[256];
    uint8_t end;
};

struct Output {
    uint8_t out[256];
    uint8_t len;
};

#endif
