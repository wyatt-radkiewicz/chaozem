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
};

/// General purpose register type
pub const Reg = enum(u1) {
    d,
    a,

    /// Format the register for output
    pub fn fmt(this: @This(), n: u3) Fmt {
        return .{ .m = this, .n = n };
    }

    /// Formatting for a register
    pub const Fmt = struct {
        m: Reg,
        n: u3,

        /// Print out the register with pretty formatting
        pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
            switch (this.m) {
                .a => switch (this.n) {
                    7 => try writer.print("sp", .{}),
                    else => try writer.print("a{}", .{this.n}),
                },
                .d => try writer.print("d{}", .{this.n}),
            }
        }
    };
};

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
