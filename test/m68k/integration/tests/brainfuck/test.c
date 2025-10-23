#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 202311L
#include <stdbool.h>
#endif

#include "test.h"

void* memcpy(void* dest, const void* src, int n)
{
    uint8_t* d = dest;
    const uint8_t* s = src;
    while (--n >= 0) {
        d[n] = s[n];
    }
    return dest;
}
void* memset(void* dest, int value, int n)
{
    uint8_t* d = dest;
    while (--n >= 0) {
        d[n] = value;
    }
    return dest;
}

struct State {
    // Memory
    uint8_t cells[0x100];
    uint8_t cell;

    // Instruction pointer
    uint8_t pc;

    // Input pointer
    int8_t in;
};

// Run one step of the machine, returns false when program is complete
static bool state_step(struct State* self, const struct Input* input, struct Output* output)
{
    switch (input->code[self->pc++]) {
    case '\0':
        return false;
    case '>':
        self->cell += 1;
        return true;
    case '<':
        self->cell -= 1;
        return true;
    case '+':
        self->cells[self->cell] += 1;
        return true;
    case '-':
        self->cells[self->cell] -= 1;
        return true;
    case '.':
        if (output->len < sizeof(output->out) - 1) {
            output->out[output->len++] = self->cells[self->cell];
            output->out[output->len] = '\0';
        }
        return true;
    case ',':
        if (self->in <= input->end) {
            self->cells[self->cell] = input->in[self->in++];
        }
        return true;
    case '[':
        if (self->cells[self->cell] == 0) {
            uint8_t depth = 1;
            while (true) {
                switch (input->code[self->pc++]) {
                case '\0':
                    return false;
                case ']':
                    if (--depth == 0) {
                        return true;
                    } else {
                        break;
                    }
                case '[':
                    ++depth;
                    break;
                default:
                    break;
                }
            }
        }
        return true;
    case ']':
        if (self->cells[self->cell] != 0) {
            uint8_t depth = 0;
            while (true) {
                switch (input->code[--self->pc]) {
                case '\0':
                    return false;
                case '[':
                    if (--depth == 0) {
                        ++self->pc;
                        return true;
                    } else {
                        break;
                    }
                case ']':
                    ++depth;
                    break;
                default:
                    break;
                }
            }
        }
        return true;
    default:
        return true;
    }
}

// Save state variable somewhere in ram
struct State state;

void run(const struct Input* input, struct Output* output)
{
    // Initialize the state
    state = (struct State){
        .cells = { 0 },
        .cell = 0,
        .pc = 0,
        .in = 0,
    };

    // Initialize the output state
    *output = (struct Output) {
        .len = 0,
    };

    while (state_step(&state, input, output)) { }
}
