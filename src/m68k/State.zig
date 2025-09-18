//! M68k cpu state

/// Data registers
d: [8]u32 = [1]u32{0} ** 8,
/// Address registers
a: [8]u32 = [1]u32{0} ** 8,
/// Program counter
pc: u32 = 0,
/// Status register
sr: Status = .{},

/// M68K processor status flags
pub const Status = packed struct {
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
    _pad0: u3 = 0,
    /// Interrupt priority level
    ipl: u3 = 0,
    _pad1: u1 = 0,
    /// Master
    m: bool = false,
    /// Supervisor
    s: bool = true,
    /// Trace level
    t: u2 = 0,
};

/// Processor vectors
pub const Vector = enum(u5) {
    reset_pc = 1,
    illegal = 4,
    _,

    /// Get the address of the vector in the memory map
    fn addr(this: @This()) u32 {
        return @as(u32, @intFromEnum(this)) * @sizeOf(u32);
    }
};
