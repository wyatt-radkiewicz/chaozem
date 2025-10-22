#include "test.h"

#define EMPTY ' '
#define ALIVE 'x'

static inline uint8_t* cell(const uint8_t* cells, uint8_t x, uint8_t y)
{
    return (uint8_t*)&cells[y * W + x];
}

void conway(const uint8_t* curr, uint8_t* next)
{
    for (uint8_t x = W - 1; (int8_t)x >= 0; --x) {
        for (uint8_t y = H - 1; (int8_t)y >= 0; --y) {
            // Get neighbor count
            uint8_t neighbors = 0;

            // Left side
            neighbors += x > 0 && y > 0 && *cell(curr, x - 1, y - 1) == ALIVE;
            neighbors += x > 0 && *cell(curr, x - 1, y) == ALIVE;
            neighbors += x > 0 && y < H - 1 && *cell(curr, x - 1, y + 1) == ALIVE;

            // Right side
            neighbors += x < W - 1 && y > 0 && *cell(curr, x + 1, y - 1) == ALIVE;
            neighbors += x < W - 1 && *cell(curr, x + 1, y) == ALIVE;
            neighbors += x < W - 1 && y < H - 1 && *cell(curr, x + 1, y + 1) == ALIVE;

            // Top and bottom
            neighbors += y > 0 && *cell(curr, x, y - 1) == ALIVE;
            neighbors += y < H - 1 && *cell(curr, x, y + 1) == ALIVE;

            // Update this cell
            if (*cell(curr, x, y) == ALIVE) {
                if (neighbors >= 2 && neighbors <= 3) {
                    *cell(next, x, y) = *cell(curr, x, y);
                } else {
                    *cell(next, x, y) = EMPTY;
                }
            } else if (neighbors == 3) {
                *cell(next, x, y) = ALIVE;
            } else {
                *cell(next, x, y) = EMPTY;
            }
        }
    }
}

void run(const struct Input* input, struct Output* output)
{
    conway(input->cells, output->cells);
}
