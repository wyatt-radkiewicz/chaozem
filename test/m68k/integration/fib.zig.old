const std = @import("std");

pub const Input = extern struct {
    to: u32,
};

pub const Output = extern struct {
    result: u32,
};

fn fib(n: u32) u32 {
    var a: u32 = 0;
    var b: u32 = 1;
    for (0..n) |_| {
        const next = a +% b;
        a = b;
        b = next;
    }
    return a;
}

pub export fn run(input: *const Input, output: *Output) void {
    output.result = fib(input.to);
}
