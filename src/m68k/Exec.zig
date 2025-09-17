/// M68K execution context
const std = @import("std");
const Cpu = @import("Cpu.zig");
const Bus = @import("Bus.zig");

cpu: *Cpu,
bus: *Bus,
clk: usize = 0,

/// M68K operation sizes
const Size = enum {
    /// Byte
    b,
    /// Word
    w,
    /// Long
    l,

    /// Get the integer type for this operation size
    fn Int(comptime this: @This(), comptime signedness: std.builtin.Signedness) type {
        return std.meta.Int(signedness, this.bitSize());
    }

    /// Get the number of bits for this size
    fn bitSize(comptime this: @This()) comptime_int {
        return switch (this) {
            .b => 8,
            .w => 16,
            .l => 32,
        };
    }

    /// M68K size encodings
    const Enc = union(enum) {
        /// Fixed size at compile time
        fixed: Size,
        /// Size determined from bits at a position in the opcode
        dyn: struct {
            at: comptime_int,
            b: ?comptime_int = null,
            w: ?comptime_int = null,
            l: ?comptime_int = null,
        },

        /// Gets the integer type used to represent the size
        fn Type(comptime this: @This()) type {
            return std.math.IntFittingRange(0, this.biggestEncoding() orelse 0);
        }

        /// Decodes a size, and returns null if there was no encoding for the bits given
        fn decode(comptime this: @This(), opcode: u16) ?Size {
            return switch (this) {
                .fixed => |size| size,
                .dyn => |enc| blk: {
                    const map = comptime build: {
                        var map = [1]?Size{null} ** (this.biggestEncoding() orelse 0 + 1);
                        for (std.meta.fieldNames(@TypeOf(enc))) |field| {
                            if (@FieldType(enc, field) == ?comptime_int) {
                                map[@field(enc, field) orelse continue] = @field(Size, field);
                            }
                        }
                        break :build map;
                    };
                    break :blk map[extract(this.Type(), opcode, enc.at)];
                },
            };
        }

        /// Encodes a size and returns the set bits
        fn encode(comptime this: @This(), size: Size) this.Type() {
            return switch (this) {
                .fixed => {},
                .dyn => |enc| switch (size) {
                    inline else => |s| @field(enc, @tagName(s)) orelse 0 << enc.at,
                },
            };
        }

        /// Gets the biggest encoding value
        fn biggestEncoding(comptime this: @This()) ?comptime_int {
            return switch (this) {
                .fixed => null,
                .dyn => |enc| @max(enc.b orelse 0, enc.w orelse 0, enc.l orelse 0),
            };
        }
    };
};

/// Fetch a type from the program counter of the cpu
inline fn fetch(this: *@This(), comptime Data: type) Data {
    const fetch_width = @max(16, @bitSizeOf(Data));
    const data = this.read(std.meta.Int(.unsigned, fetch_width), this.cpu.pc);
    this.cpu.pc += fetch_width / 8;
    return @bitCast(@as(std.meta.Int(.unsigned, @bitSizeOf(Data)), @truncate(data)));
}

/// Read data type X from the bus
inline fn read(this: *@This(), comptime Data: type, addr: u32) Data {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            return @bitCast(this.bus.rdb(this.bus, addr));
        },
        16 => {
            this.clk += 4;
            return @bitCast(this.bus.rdw(this.bus, addr));
        },
        32 => {
            this.clk += 8;
            return @bitCast(this.bus.rdl(this.bus, addr));
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to read data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Write data type X to the bus
inline fn write(this: *@This(), comptime Data: type, addr: u32, data: Data) void {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            this.bus.wrb(this.bus, addr, @bitCast(data));
        },
        16 => {
            this.clk += 4;
            this.bus.wrw(this.bus, addr, @bitCast(data));
        },
        32 => {
            this.clk += 8;
            this.bus.wrl(this.bus, addr, @bitCast(data));
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to write data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Push data onto the stack
inline fn push(this: *@This(), comptime Data: type, data: Data) void {
    const Push = std.meta.Int(.unsigned, @max(16, @bitSizeOf(Data)));
    this.cpu.a[7] -%= @sizeOf(Push);
    this.write(Push, this.cpu.a[7], @as(std.meta.Int(.unsigned, @bitSizeOf(Data)), data));
}

/// Pop data off the stack
inline fn pop(this: *@This(), comptime Data: type) Data {
    const Push = std.meta.Int(.unsigned, @max(16, @bitSizeOf(Data)));
    const data = this.read(Push, this.cpu.a[7]);
    this.cpu.a[7] +%= @sizeOf(Push);
    return @bitCast(@as(std.meta.Int(.unsigned, @bitSizeOf(Data)), @truncate(data)));
}

/// Extract a type from an integer at a position
inline fn extract(comptime Type: type, from: anytype, at: std.math.Log2Int(@TypeOf(from))) Type {
    const TypeInt = std.meta.Int(.unsigned, @bitSizeOf(Type));
    const FromInt = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(from)));
    return @bitCast(@as(TypeInt, @truncate(@as(FromInt, @bitCast(from)) >> at)));
}

/// Sign extend a specified integer to the specified size
inline fn extend(comptime To: type, from: anytype) To {
    const ToSigned = std.meta.Int(.signed, @bitSizeOf(To));
    const FromSigned = std.meta.Int(.signed, @bitSizeOf(@TypeOf(from)));
    return @bitCast(@as(ToSigned, @as(FromSigned, @bitCast(from))));
}

/// Overwrite the lower N bits with another integer
inline fn overwrite(int: anytype, with: anytype) @TypeOf(int) {
    const Int = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(int)));
    const mask: Int = (1 << @bitSizeOf(@TypeOf(with))) - 1;
    return int & ~mask | with;
}

/// Returns true if the int has the highest bit set
inline fn negative(int: anytype) bool {
    return @as(std.meta.Int(.signed, @bitSizeOf(@TypeOf(int))), @bitCast(int)) < 0;
}

/// Converts a byte to a binary coded decimal byte
/// It will return a struct encoding the wrapped bcd byte and if an overflow occurred
inline fn tobcd(byte: u8) struct { u8, bool } {
    const carry = byte > 99;
    const wrapped = byte % 100;
    return .{ ((wrapped / 10) << 4) + (wrapped % 10), carry };
}

/// Converts bcd to a normal byte
inline fn frombcd(bcd: u8) u8 {
    return ((bcd >> 4) * 10) + (bcd & 0xf);
}
