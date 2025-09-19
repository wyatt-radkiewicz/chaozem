/// M68K execution context
const std = @import("std");

const bus_interface = @import("bus");
const Bus = bus_interface.Bus;
const int = @import("int");

const Cpu = @import("Cpu.zig");

const Ctx = @This();

cpu: *Cpu,
bus: *const Bus(Cpu.width),
clk: usize = 0,

/// M68K operation sizes
const Size = enum {
    /// No size specified
    none,
    /// Byte (8 bits)
    b,
    /// Word (16 bits)
    w,
    /// Long (32 bits)
    l,

    /// Get the integer type for this operation size
    fn Int(comptime this: @This(), comptime signedness: std.builtin.Signedness) type {
        return std.meta.Int(signedness, this.bits());
    }

    /// Get the number of bits for this size
    fn bits(comptime this: @This()) comptime_int {
        return switch (this) {
            .none => 0,
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
                    break :blk map[int.extract(this.Type(), opcode, enc.at)];
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

/// Calculate address mode destinations
const Mode = enum {
    data_reg,
    addr_reg,
    indirect,
    post_inc,
    pre_dec,
    addr_disp,
    addr_idx,
    abs_word,
    abs_long,
    pc_disp,
    pc_idx,
    immediate,

    /// Get from 'm' and 'n' bits
    fn decode(m: u3, n: u3) @This() {
        return switch (m) {
            0b000 => .data_reg,
            0b001 => .addr_reg,
            0b010 => .indirect,
            0b011 => .post_inc,
            0b100 => .pre_dec,
            0b101 => .addr_disp,
            0b110 => .addr_idx,
            0b111 => switch (n) {
                0b000 => .abs_word,
                0b001 => .abs_long,
                0b010 => .pc_disp,
                0b011 => .pc_idx,
                0b100 => .immediate,
                else => @panic("Bad address mode encoding"),
            },
        };
    }

    /// Calculate an address, reg can be left as undefined if its a mode that doesn't require it
    fn calc(this: @This(), ctx: *Ctx, comptime size: Size, reg: u3) u32 {
        return switch (this) {
            .data_reg, .addr_reg => reg,
            .indirect => ctx.cpu.a[reg],
            .post_inc => post_inc: {
                const addr = ctx.cpu.a[reg];
                ctx.cpu.a[reg] += size.bits() / 8;
                break :post_inc addr;
            },
            .pre_dec => pre_dec: {
                ctx.cpu.a[reg] -= size.bits() / 8;
                ctx.clk += 2;
                break :pre_dec ctx.cpu.a[reg];
            },
            .addr_disp => ctx.cpu.a[reg] +% int.extend(u32, ctx.fetch(u16)),
            .addr_idx => ctx.cpu.a[reg] +% ctx.fetch(Index).calc(ctx),
            .abs_word => int.extend(u32, ctx.fetch(u16)),
            .abs_long => ctx.fetch(u32),
            .pc_disp => ctx.cpu.pc +% int.extend(u32, ctx.fetch(u16)),
            .pc_idx => ctx.cpu.pc +% ctx.fetch(Index).calc(ctx),
            .immediate => 0,
        };
    }

    /// Index extension word used in address calculation
    const Index = packed struct {
        disp: i8,
        _pad0: u3,
        size: u1,
        n: u3,
        m: u1,

        /// Gets the total displacement of the index word (casted to unsigned int)
        fn calc(this: @This(), ctx: *Ctx) u32 {
            ctx.clk += 2;
            const reg = switch (this.m) {
                0 => ctx.cpu.d[this.n],
                1 => ctx.cpu.a[this.n],
            };
            return switch (this.size) {
                0 => int.extend(u32, @as(u16, @truncate(reg))),
                1 => reg,
            } +% int.extend(u32, this.disp);
        }
    };

    /// Disassemble an addressing mode
    fn Disasm(comptime size: Size) type {
        return struct {
            reader: *std.io.Reader,
            mode: Mode,
            reg: u3,

            pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                switch (this.mode) {
                    .data_reg => try writer.print("d{}", .{this.reg}),
                    .addr_reg => try writer.print("a{}", .{this.reg}),
                    .indirect => try writer.print("(a{})", .{this.reg}),
                    .post_inc => try writer.print("(a{})+", .{this.reg}),
                    .pre_dec => try writer.print("-(a{})", .{this.reg}),
                    .addr_disp => try writer.print("({},a{})", .{
                        this.reader.takeInt(i16, .big) catch return error.WriteFailed,
                        this.reg,
                    }),
                    .addr_idx, .pc_idx => |mode| {
                        const idx: Index = @bitCast(this.reader.takeInt(u16, .big) catch
                            return error.WriteFailed);
                        try writer.print("({},", .{idx.disp});
                        switch (mode) {
                            .addr_idx => try writer.print("a{}", .{this.reg}),
                            .pc_idx => try writer.print("pc", .{}),
                            else => unreachable,
                        }
                        try writer.print(",{c}{}.{c})", .{ switch (idx.m) {
                            0 => @as(u8, 'd'),
                            1 => @as(u8, 'a'),
                        }, idx.n, switch (idx.size) {
                            0 => @as(u8, 'w'),
                            1 => @as(u8, 'l'),
                        } });
                    },
                    .abs_word => try writer.print(
                        "(${x:0>4}).w",
                        .{this.reader.takeInt(u16, .big) catch return error.WriteFailed},
                    ),
                    .abs_long => try writer.print(
                        "(${x:0>8}).l",
                        .{this.reader.takeInt(u32, .big) catch return error.WriteFailed},
                    ),
                    .pc_disp => try writer.print("({}, pc)", .{this.reader.takeInt(i16, .big) catch
                        return error.WriteFailed}),
                    .immediate => switch (size) {
                        .b => try writer.print("#${x:0>2}", .{@as(u8, @truncate(
                            this.reader.takeInt(u16, .big) catch return error.WriteFailed,
                        ))}),
                        .w => try writer.print("#${x:0>4}", .{this.reader.takeInt(u16, .big) catch
                            return error.WriteFailed}),
                        .l => try writer.print("#${x:0>8}", .{this.reader.takeInt(u32, .big) catch
                            return error.WriteFailed}),
                    },
                }
            }
        };
    }
};

/// Fetch a type from the program counter of the cpu
inline fn fetch(this: *Ctx, comptime Data: type) Data {
    const fetch_width = @max(16, @bitSizeOf(Data));
    const data = this.read(std.meta.Int(.unsigned, fetch_width), this.cpu.pc);
    this.cpu.pc += fetch_width / 8;
    return @bitCast(@as(std.meta.Int(.unsigned, @bitSizeOf(Data)), @truncate(data)));
}

/// Read data type X from the bus
inline fn read(this: *Ctx, comptime Data: type, addr: u32) Data {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            const byte: u1 = @truncate(addr & 1);
            const word = this.bus.read(@truncate(addr >> 1), @as(u2, 1) << byte) orelse 0;
            return @bitCast(@as(u8, @truncate(word >> @as(u4, byte) * 8)));
        },
        16 => {
            this.clk += 4;
            return @bitCast(this.bus.read(@truncate(addr >> 1), 0xFFFF) orelse 0);
        },
        32 => {
            this.clk += 8;
            return @bitCast(@as(u32, this.bus.read(@truncate(addr >> 1), 0xFFFF) orelse 0) << 16 |
                @as(u32, this.bus.read(@truncate((addr >> 1) + 1), 0xFFFF) orelse 0));
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to read data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Write data type X to the bus
inline fn write(this: *Ctx, comptime Data: type, addr: u32, data: Data) void {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            const byte: u1 = @truncate(addr);
            const word = @as(u16, @as(u8, @bitCast(data)));
            this.bus.write(@truncate(addr >> 1), word << @as(u4, byte) * 8, 1 << byte);
        },
        16 => {
            this.clk += 4;
            this.bus.write(@truncate(addr >> 1), @bitCast(data), 0xFFFF);
        },
        32 => {
            this.clk += 8;
            const long = @as(u32, @bitCast(data));
            this.bus.write(@truncate(addr >> 1), @truncate(long >> 16), 0xFFFF);
            this.bus.write(@truncate((addr >> 1) + 1), @truncate(long), 0xFFFF);
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to write data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Push data onto the stack
inline fn push(this: *Ctx, comptime Data: type, data: Data) void {
    const Push = std.meta.Int(.unsigned, @max(16, @bitSizeOf(Data)));
    this.cpu.a[7] -%= @sizeOf(Push);
    this.write(Push, this.cpu.a[7], @as(std.meta.Int(.unsigned, @bitSizeOf(Data)), data));
}

/// Pop data off the stack
inline fn pop(this: *Ctx, comptime Data: type) Data {
    const Push = std.meta.Int(.unsigned, @max(16, @bitSizeOf(Data)));
    const data = this.read(Push, this.cpu.a[7]);
    this.cpu.a[7] +%= @sizeOf(Push);
    return @bitCast(@as(std.meta.Int(.unsigned, @bitSizeOf(Data)), @truncate(data)));
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
