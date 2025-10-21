#ifndef TEST_H
#define TEST_H

#include <stdint.h>

struct Input {
    uint16_t offset;
    uint16_t count;
    uint8_t a[8];
    uint8_t b[8];
};

struct Output {
    uint16_t result;
};

#endif