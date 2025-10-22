#ifndef TEST_H
#define TEST_H

#include <stdint.h>

#define W 8
#define H 8

struct Input {
    // Row major
    uint8_t cells[H * W];
};

struct Output {
    // Row major
    uint8_t cells[H * W];
};

#endif
