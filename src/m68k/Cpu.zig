//! M68k cpu state
const std = @import("std");

const bus_interface = @import("bus");

/// Data registers
d: [8]u32 = [1]u32{0} ** 8,
/// Address registers
a: [7]u32 = [1]u32{0} ** 7,
/// Stack pointers (user, then supervisor)
sp: [2]u32 = [1]u32{0} ** 2,
/// Program counter
pc: u32 = 0,
/// Status register
sr: Status = .{},
/// Whether the processor is stopped
stop: bool = false,

/// M68K processor status flags
pub const Status = packed struct(u16) {
    /// Carry
    c: bool = false,
    /// Overflow
    v: bool = false,
    /// Zero
    z: bool = false,
    /// Negative
    n: bool = false,
    /// Extend
    x: bool = false,
    /// Reserved bits
    reserved5: u3 = 0,
    /// Interrupt priority level
    ipl: u3 = 0,
    /// Reserved bits
    reserved8: u1 = 0,
    /// Master
    m: bool = false,
    /// Supervisor
    s: bool = true,
    /// Trace level
    t: u2 = 0,

    /// Pretty formatting
    pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
        try writer.print("Status Register\n\t", .{});
        try writer.print("Raw:        {X:0>4}\n\t", .{@as(u16, @bitCast(this))});
        try writer.print("Carry:      {}\n\t", .{this.c});
        try writer.print("Overflow:   {}\n\t", .{this.v});
        try writer.print("Zero:       {}\n\t", .{this.z});
        try writer.print("Negative:   {}\n\t", .{this.n});
        try writer.print("Extend:     {}\n\t", .{this.x});
        try writer.print("IPL:        {}\n\t", .{this.ipl});
        try writer.print("Master:     {}\n\t", .{this.m});
        try writer.print("Supervisor: {}\n\t", .{this.s});
        try writer.print("Trace:      {}\n", .{this.t});
    }
};

/// General purpose register type
pub const Reg = enum(u1) { d, a };

/// Get address register n
pub inline fn r(this: *@This(), comptime reg: Reg, n: u3) *u32 {
    return switch (reg) {
        .d => &this.d[n],
        .a => switch (n) {
            7 => &this.sp[@intFromBool(this.sr.s)],
            else => |x| &this.a[x],
        },
    };
}

/// Pretty formatting
pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
    // Data registers
    try writer.print("Data Registers\n\t", .{});
    for (0..8) |i| {
        try writer.print("d{}          ", .{i});
    }
    try writer.print("\n\t", .{});
    for (this.d) |d| {
        try writer.print("0x{X:0>8}  ", .{d});
    }
    try writer.print("\n", .{});

    // Address registers
    try writer.print("Addr Registers\n\t", .{});
    for (0..8) |i| {
        try writer.print("a{}          ", .{i});
    }
    try writer.print("\n\t", .{});
    for (0..8) |i| {
        try writer.print("0x{X:0>8}  ", .{switch (i) {
            7 => this.sp[@intFromBool(this.sr.s)],
            else => this.a[i],
        }});
    }
    try writer.print("\n", .{});

    // Stack pointers
    try writer.print("Stack Pointers\n\t", .{});
    try writer.print("(user)      (supervisor)\n\t", .{});
    for (0..2) |i| {
        try writer.print("0x{X:0>8}  ", .{this.sp[i]});
    }
    try writer.print("\n", .{});

    // Program counter
    try writer.print("Program Counter\n\t", .{});
    try writer.print("0x{X:0>8}\n", .{this.pc});

    // Status register
    try writer.print("{f}", .{this.sr});

    // Stop flag
    try writer.print("Stop: {}\n", .{this.stop});
}
