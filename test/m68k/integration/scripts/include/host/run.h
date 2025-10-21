#ifndef RUN_H
#define RUN_H

#ifndef TEST_H
#error Expected test header to be included
#else
void run(const struct Input* input, struct Output* output);
#endif

#endif
