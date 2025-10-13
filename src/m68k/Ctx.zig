/// M68K execution context
const std = @import("std");

const bus_interface = @import("bus");
const Bus = bus_interface.Bus;
const int = @import("int");

const Cpu = @import("Cpu.zig");

const Ctx = @This();

cpu: *Cpu,
bus: *const Bus(bus_width),
clk: usize = 0,

/// M68K processesor bus width
pub const bus_width = bus_interface.Width{ .addr = 23, .data = 16 };

/// M68K operation sizes
pub const Size = enum {
    /// No size specified
    none,
    /// Byte (8 bits)
    b,
    /// Word (16 bits)
    w,
    /// Long (32 bits)
    l,

    /// Get the unsigned integer type for this operation size
    pub fn Int(comptime this: @This()) type {
        return std.meta.Int(.unsigned, this.bits());
    }

    /// Get the number of bits for this size
    pub fn bits(comptime this: @This()) comptime_int {
        return switch (this) {
            .none => 0,
            .b => 8,
            .w => 16,
            .l => 32,
        };
    }

    /// M68K size encodings
    pub const Enc = union(enum) {
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
        pub fn Type(comptime this: @This()) type {
            return std.math.IntFittingRange(0, this.biggestEncoding() orelse return void);
        }

        /// Decodes a size, and returns null if there was no encoding for the bits given
        pub fn decode(comptime this: @This(), opcode: u16) ?Size {
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
        pub fn encode(comptime this: @This(), size: Size) this.Type() {
            return switch (this) {
                .fixed => {},
                .dyn => |enc| switch (size) {
                    inline else => |s| @field(enc, @tagName(s)) orelse 0 << enc.at,
                },
            };
        }

        /// Gets the biggest encoding value
        pub fn biggestEncoding(comptime this: @This()) ?comptime_int {
            return switch (this) {
                .fixed => null,
                .dyn => |enc| @max(enc.b orelse 0, enc.w orelse 0, enc.l orelse 0),
            };
        }
    };
};

/// Calculate address mode destinations
pub const Mode = enum {
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
    pub fn decode(m: u3, n: u3) @This() {
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
    pub fn calc(this: @This(), ctx: *Ctx, comptime size: Size, reg: u3) u32 {
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
    pub fn Disasm(comptime size: Size) type {
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
                        try writer.print("(#{},", .{idx.disp});
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
                        .none => unreachable,
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

/// Processor vectors
pub const Vector = enum(u5) {
    reset_sp = 0,
    reset_pc = 1,
    illegal = 4,
    divzero = 5,
    chk = 6,
    _,

    /// Get the address of the vector in the memory map
    pub fn addr(this: @This()) u32 {
        return @as(u32, @intFromEnum(this)) * @sizeOf(u32);
    }

    /// Handle exception
    pub fn handle(this: @This(), ctx: *Ctx) void {
        switch (this) {
            .illegal => Group.@"1".handle(this, ctx, 10),
            .divzero => Group.@"2".handle(this, ctx, 8),
            .chk => Group.@"2".handle(this, ctx, 12),
            else => {},
        }
    }

    /// Exception group
    const Group = enum {
        @"1",
        @"2",

        /// Handle an exception in that exception group
        pub fn handle(this: @This(), vector: Vector, ctx: *Ctx, extra_clks: usize) void {
            ctx.clk += extra_clks;
            switch (this) {
                .@"1", .@"2" => {
                    ctx.push(u32, ctx.cpu.pc);
                    ctx.push(Cpu.Status, ctx.cpu.sr);
                    ctx.cpu.pc = ctx.read(u32, vector.addr());
                },
            }
        }
    };
};

/// Conditionals
pub const Cond = enum(u4) {
    t,
    f,
    hi,
    ls,
    cc,
    cs,
    ne,
    eq,
    vc,
    vs,
    pl,
    mi,
    ge,
    lt,
    gt,
    le,

    /// Returns whether or not the conditional is true
    pub fn value(this: @This(), s: Cpu.Status) bool {
        return switch (this) {
            .t => true,
            .f => false,
            .hi => !s.c and !s.z,
            .ls => s.c or s.z,
            .cc => !s.c,
            .cs => s.c,
            .ne => !s.z,
            .eq => s.z,
            .vc => !s.v,
            .vs => s.v,
            .pl => !s.n,
            .mi => s.n,
            .ge => s.n and s.v or !s.n and !s.v,
            .lt => s.n and !s.v or !s.n and s.v,
            .gt => s.n and s.v and !s.z or !s.n and !s.v and !s.z,
            .le => s.z or s.n and !s.v or !s.n and s.v,
        };
    }
};

/// Register masks
pub const RegMask = packed struct {
    d: std.bit_set.IntegerBitSet(8),
    a: std.bit_set.IntegerBitSet(8),

    /// Order to store registers in
    pub const Order = enum {
        forward,
        reverse,

        /// Gets the register type order
        fn types(this: @This()) []const []const u8 {
            return switch (this) {
                .forward => &.{ "d", "a" },
                .reverse => &.{ "a", "d" },
            };
        }

        /// Gets mask iterator options
        fn iteratorOptions(this: @This()) std.bit_set.IteratorOptions {
            return .{ .direction = @field(std.bit_set.IteratorOptions.Direction, switch (this) {
                inline else => |x| @tagName(x),
            }) };
        }
    };

    /// Get the number of registers set in this mask
    pub fn count(this: @This()) usize {
        return this.d.count() + this.a.count();
    }

    /// Store a register mask to memory
    pub fn store(
        this: @This(),
        comptime size: Size,
        ctx: *Ctx,
        addr: u32,
        comptime order: Order,
    ) void {
        var offs: u32 = 0;
        inline for (comptime order.types()) |reg_type| {
            var iter = @field(this, reg_type).iterator(order.iteratorOptions());
            while (iter.next()) |idx| : (offs += size.bits() / 8) {
                const reg = @field(ctx.cpu, reg_type)[idx];
                ctx.write(size.Int(), addr +% offs, @truncate(reg));
            }
        }
    }

    /// Load registers from memory using a mask
    pub fn load(
        this: @This(),
        comptime size: Size,
        ctx: *Ctx,
        addr: u32,
        comptime order: Order,
    ) void {
        var offs: u32 = 0;
        inline for (comptime order.types()) |reg_type| {
            var iter = @field(this, reg_type).iterator(order.iteratorOptions());
            while (iter.next()) |idx| : (offs += size.bits() / 8) {
                const data = ctx.read(size.Int(), addr +% offs);
                const reg = &@field(ctx.cpu, reg_type)[idx];
                reg.* = int.overwrite(reg.*, data);
            }
        }
    }

    /// Disassemble the register mask
    pub const Disasm = struct {
        reader: *std.io.Reader,
        mask: RegMask,

        pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
            inline for (&.{ "d", "a" }) |reg_type| {
                var bits = @field(this.mask, reg_type).mask;
                var idx: u4 = 0;
                while (bits != 0) {
                    const ones = @ctz(~bits);
                    if (ones > 0) {
                        if (ones == 1) {
                            try writer.print("{s}{}", .{ reg_type, idx });
                        } else {
                            try writer.print(
                                "{s}{}-{s}{}",
                                .{ reg_type, idx, reg_type, idx + ones - 1 },
                            );
                        }

                        if (ones != @bitSizeOf(@TypeOf(bits))) {
                            bits >>= @intCast(ones);
                            idx += ones;

                            // Print the seperator if needed
                            try writer.print("{s}", .{if (bits != 0) "/" else ""});
                        }
                    } else {
                        bits >>= 1;
                        idx += 1;
                    }
                }

                // Print a seperator between data and address registers
                try writer.print("{s}", .{if (this.mask.d.count() > 0 and
                    this.mask.a.count() > 0 and
                    comptime std.mem.eql(u8, reg_type, "d")) "/" else ""});
            }
        }
    };
};

/// Fetch a type from the program counter of the cpu
pub inline fn fetch(this: *Ctx, comptime Data: type) Data {
    const fetch_width = @max(16, @bitSizeOf(Data));
    const data = this.read(std.meta.Int(.unsigned, fetch_width), this.cpu.pc);
    this.cpu.pc += fetch_width / 8;
    return @bitCast(@as(std.meta.Int(.unsigned, @bitSizeOf(Data)), @truncate(data)));
}

/// Read data type X from the bus
pub inline fn read(this: *Ctx, comptime Data: type, addr: u32) Data {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            const byte: u1 = @truncate(~addr);
            const word = this.bus.read(@truncate(addr >> 1), @as(u2, 1) << byte) orelse 0;
            return @bitCast(@as(u8, @truncate(word >> @as(u4, byte) * 8)));
        },
        16 => {
            this.clk += 4;
            return @bitCast(this.bus.read(@truncate(addr >> 1), 0b11) orelse 0);
        },
        32 => {
            this.clk += 8;
            return @bitCast(@as(u32, this.bus.read(@truncate(addr >> 1), 0b11) orelse 0) << 16 |
                @as(u32, this.bus.read(@truncate((addr >> 1) + 1), 0b11) orelse 0));
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to read data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Write data type X to the bus
pub inline fn write(this: *Ctx, comptime Data: type, addr: u32, data: Data) void {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            const byte: u1 = @truncate(~addr);
            const word = @as(u16, @as(u8, @bitCast(data)));
            this.bus.write(@truncate(addr >> 1), @as(u2, 1) << byte, word << @as(u4, byte) * 8);
        },
        16 => {
            this.clk += 4;
            this.bus.write(@truncate(addr >> 1), 0b11, @bitCast(data));
        },
        32 => {
            this.clk += 8;
            const long = @as(u32, @bitCast(data));
            this.bus.write(@truncate(addr >> 1), 0b11, @truncate(long >> 16));
            this.bus.write(@truncate((addr >> 1) + 1), 0b11, @truncate(long));
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to write data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Push data onto the stack
pub inline fn push(this: *Ctx, comptime Data: type, data: Data) void {
    const bits = @bitSizeOf(Data);
    const Push = std.meta.Int(.unsigned, @max(16, bits));
    this.cpu.a[7] -%= @sizeOf(Push);
    this.write(Push, this.cpu.a[7], @as(std.meta.Int(.unsigned, bits), @bitCast(data)));
}

/// Pop data off the stack
pub inline fn pop(this: *Ctx, comptime Data: type) Data {
    const bits = @bitSizeOf(Data);
    const Push = std.meta.Int(.unsigned, @max(16, bits));
    const data = this.read(Push, this.cpu.a[7]);
    this.cpu.a[7] +%= @sizeOf(Push);
    return @bitCast(@as(std.meta.Int(.unsigned, bits), @truncate(data)));
}

/// Converts a byte to a binary coded decimal byte
/// It will return a struct encoding the wrapped bcd byte and if an overflow occurred
pub inline fn tobcd(byte: u8) struct { u8, bool } {
    const carry = byte > 99;
    const wrapped = byte % 100;
    return .{ ((wrapped / 10) << 4) + (wrapped % 10), carry };
}

/// Converts bcd to a normal byte
pub inline fn frombcd(bcd: u8) u8 {
    return ((bcd >> 4) * 10) + (bcd & 0xf);
}
