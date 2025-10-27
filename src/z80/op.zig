const std = @import("std");

const int = @import("int");

const Cpu = @import("Cpu.zig");
const exec = @import("exec.zig");
const Bus = exec.Bus;

/// No operation
pub fn nop(_: *Cpu) void {}

/// Load operation
pub fn ld(comptime Type: type) fn (*Cpu, Type) Type {
    return struct {
        pub fn inner(_: *Cpu, src: Type) Type {
            return src;
        }
    }.inner;
}
