const std = @import("std");

/// Each 8 bit register bank
banks: [2]Bank = [2]Bank{.{}, .{}},

/// Which bank is being currently used
set: u1 = 0,

/// Index register 'X'
ix: u16 = 0,

/// Index register 'Y'
iy: u16 = 0,

/// Stack pointer
sp: u16 = 0,

/// Program counter
pc: u16 = 0,

/// 8 bit register bank
pub const Bank = struct {
    /// Accumulator
    a: u8 = 0,
    
    /// Flags register
    f: Flags = .{},
    
    /// General purpose registers
    gp: [6]u8 = [1]u8{0} ** 6,

    /// Gives access to 8 bit registers
    pub const R8 = enum {
        b,
        c,
        d,
        e,
        h,
        l,

        /// Get one of the registers
        pub fn get(this: @This(), bank: Bank) u8 {
            return bank.regs[@intFromEnum(this)];
        }

        /// Set one of the registers
        pub fn set(this: @This(), bank: *Bank, value: u8) void {
            bank.regs[@intFromEnum(this)] = value;
        }
    };

    /// Gives access to 16 bit registers
    pub const R16 = enum {
        bc,
        de,
        hl,

        /// Get one of the registers
        pub fn get(this: @This(), bank: Bank) u16 {
            return std.mem.readInt(u16, bank.regs[@intFromEnum(this) * 2][0..2], .big);
        }

        /// Set one of the registers
        pub fn set(this: @This(), bank: *Bank, value: u16) void {
            std.mem.writeInt(u16, bank.regs[@intFromEnum(this) * 2][0..2], value, .big);
        }
    };
};

/// Flags for the z80
pub const Flags = packed struct {
    /// Carry flag
    c: bool = false,
    
    /// Add/Sub flag
    n: bool = false,
    
    /// Parity/Overflow flag
    pv: bool = false,
    
    /// Unused bit
    y: bool = false,
    
    /// Half carry flag
    h: bool = false,
    
    /// Zero flag
    z: bool = false,
    
    /// Sign flag
    s: bool = false,
};
