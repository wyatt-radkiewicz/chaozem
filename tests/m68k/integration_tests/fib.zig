const std = @import("std");

pub const Input = extern struct {
    to: u32,
};

pub const Output = extern struct {
    result: u32,
};

pub export fn main(input: *const Input, output: *Output) void {
    var a: u32 = 0;
    var b: u32 = 1;
    for (0..input.to) |_| {
        const next = a +% b;
        a = b;
        b = next;
    }
    output.result = a;
}
