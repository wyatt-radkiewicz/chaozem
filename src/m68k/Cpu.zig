//! M68k cpu state
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
