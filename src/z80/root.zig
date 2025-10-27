const std = @import("std");

pub const Cpu = @import("Cpu.zig");
const exec = @import("exec.zig");
pub const bus_width = exec.bus_width;
const Bus = exec.Bus;

/// Step the cpu state by one, returns the number of clocks simulated
pub fn step(cpu: *Cpu, mem: *Bus, io: *Bus) usize {
    _ = cpu; // autofix
    _ = mem; // autofix
    _ = io; // autofix

}

/// Decode which instruction to use for either disassembling or running
fn decode(cpu: *Cpu, mem: *Bus) exec.Instr {
    _ = cpu; // autofix
    _ = mem; // autofix

}
