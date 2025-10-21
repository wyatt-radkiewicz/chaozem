#include "test.h"

void run(const struct Input* input, struct Output* output)
{
    // Initialize the output
    output->result = 0;

    // Try add/sub
    output->result = input->a + input->b;
    output->result += input->a;
    output->result -= input->a;

    // Try mul/div
    output->result *= input->a;
    output->result /= input->b / 100;

    // Try bitwise operations
    output->result += input->a & input->b;
    output->result -= input->a ^ input->b;
    output->result *= ~input->a;
    output->result /= (input->a | input->b) >> 5;
}
