//! M68k cpu state
const bus_interface = @import("bus");

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
