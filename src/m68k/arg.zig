const std = @import("std");

const int = @import("int");

const Ctx = @import("Ctx.zig");
const Size = Ctx.Size;

/// Data register source
pub fn DataReg(n: u4) type {
    return struct {
        n: u3,

        pub fn decode(_: *Ctx, comptime _: Size, opcode: u16) @This() {
            return .{ .n = int.extract(u3, opcode, n) };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime size: Size, _: u16) size.Int() {
            return @truncate(ctx.cpu.d[this.n]);
        }

        pub fn store(this: @This(), ctx: *Ctx, comptime _: Size, _: u16, data: anytype) void {
            const Int = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(data)));
            ctx.cpu.d[this.n] = int.overwrite(ctx.cpu.d[this.n], @as(Int, @bitCast(data)));
        }

        pub fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("d{}", .{int.extract(u3, this.opcode, n)});
                }
            };
        }
    };
}

/// Address register source
pub fn AddrReg(n: u4) type {
    return struct {
        n: u3,

        pub fn decode(_: *Ctx, comptime _: Size, opcode: u16) @This() {
            return .{ .n = int.extract(u3, opcode, n) };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime _: Size, _: u16) u32 {
            return ctx.cpu.an(this.n).*;
        }

        pub fn store(this: @This(), ctx: *Ctx, comptime _: Size, _: u16, data: u32) void {
            ctx.cpu.an(this.n).* = data;
        }

        pub fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("a{}", .{int.extract(u3, this.opcode, n)});
                }
            };
        }
    };
}

/// Address register indirect post increment
pub fn PostInc(n: u4) type {
    return struct {
        n: u3,

        pub fn decode(_: *Ctx, comptime _: Size, opcode: u16) @This() {
            return .{ .n = int.extract(u3, opcode, n) };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime size: Size, _: u16) size.Int() {
            const addr = ctx.cpu.an(this.n).*;
            ctx.cpu.an(this.n).* +%= size.bits() / 8;
            return ctx.read(size.Int(), addr);
        }

        pub fn store(this: @This(), ctx: *Ctx, comptime size: Size, _: u16, data: size.Int()) void {
            ctx.write(size.Int(), ctx.cpu.an(this.n).*, data);
        }

        pub fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("(a{})+", .{int.extract(u3, this.opcode, n)});
                }
            };
        }
    };
}

/// Status register source or destination target
pub const Status = struct {
    pub fn decode(_: *Ctx, comptime size: Size, _: u16) @This() {
        if (size == .l) {
            @compileError("Status register is only 16 bits!");
        }
        return .{};
    }

    pub fn load(_: @This(), ctx: *Ctx, comptime size: Size, _: u16) size.Int() {
        const bits: u16 = @bitCast(ctx.cpu.sr);
        return @truncate(bits);
    }

    pub fn store(_: @This(), ctx: *Ctx, comptime size: Size, _: u16, data: size.Int()) void {
        const bits: u16 = @bitCast(ctx.cpu.sr);
        ctx.cpu.sr = @bitCast(int.overwrite(bits, data));
    }

    pub fn Disasm(comptime size: Size) type {
        return struct {
            reader: *std.io.Reader,
            opcode: u16,

            pub fn format(_: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                try writer.print("{s}", .{switch (size) {
                    .b => "ccr",
                    .w => "sr",
                    else => @compileError("Expected byte or word with status register disasm!"),
                }});
            }
        };
    }
};

/// Access to the user stack pointer
pub const Usp = struct {
    pub fn decode(_: *Ctx, comptime _: Size, _: u16) @This() {
        return .{};
    }

    pub fn load(_: @This(), ctx: *Ctx, comptime _: Size, _: u16) u32 {
        return ctx.cpu.sp[0];
    }

    pub fn store(_: @This(), ctx: *Ctx, comptime _: Size, _: u16, data: u32) void {
        ctx.cpu.sp[0] = data;
    }

    pub fn Disasm(comptime _: Size) type {
        return struct {
            reader: *std.io.Reader,
            opcode: u16,

            pub fn format(_: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                try writer.print("usp", .{});
            }
        };
    }
};

/// Immediate source data target
pub fn Imm(comptime fmt: FormatOptions) type {
    return struct {
        pub fn decode(_: *Ctx, comptime _: Size, _: u16) @This() {
            return .{};
        }

        pub fn load(_: @This(), ctx: *Ctx, comptime size: Size, _: u16) size.Int() {
            return ctx.fetch(size.Int());
        }

        pub fn store(_: @This(), _: *Ctx, comptime _: Size, _: u16, _: void) void {}

        pub fn Disasm(comptime size: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const Data = std.meta.Int(if (fmt.hex) .unsigned else .signed, size.bits());
                    const val: size.Int() = @truncate(switch (size) {
                        else => unreachable,
                        .b, .w => this.reader.takeInt(u16, .big) catch return error.WriteFailed,
                        .l => this.reader.takeInt(u32, .big) catch return error.WriteFailed,
                    });
                    try writer.print("{f}", .{Formatter(Data){
                        .val = @bitCast(val),
                        .options = fmt,
                    }});
                }
            };
        }
    };
}

/// Bit index
pub const BitIdx = struct {
    pub fn decode(_: *Ctx, comptime _: Size, _: u16) @This() {
        return .{};
    }

    pub fn load(_: @This(), ctx: *Ctx, comptime size: Size, _: u16) size.Int() {
        return ctx.fetch(std.math.Log2Int(size.Int()));
    }

    pub fn store(_: @This(), _: *Ctx, comptime _: Size, _: u16, _: void) void {}

    pub fn Disasm(comptime size: Size) type {
        return struct {
            reader: *std.io.Reader,
            opcode: u16,

            pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                try writer.print("#{}", .{@as(
                    std.math.Log2Int(size.Int()),
                    @truncate(this.reader.takeInt(u16, .big) catch return error.WriteFailed),
                )});
            }
        };
    }
};

/// Represents a register index
pub fn RegIdx(comptime reg_type: enum { data, addr }, at: u4) type {
    return struct {
        n: u3,

        pub fn decode(_: *Ctx, comptime _: Size, opcode: u16) @This() {
            return .{ .n = int.extract(u3, opcode, at) };
        }

        pub fn load(this: @This(), _: *Ctx, comptime _: Size, _: u16) u3 {
            return this.n;
        }

        pub fn store(this: @This(), ctx: *Ctx, comptime size: Size, _: u16, data: size.Int()) void {
            switch (reg_type) {
                .data => ctx.cpu.d[this.n] = int.overwrite(ctx.cpu.d[this.n], data),
                .addr => ctx.cpu.an(this.n).* = int.overwrite(ctx.cpu.an(this.n).*, data),
            }
        }

        pub fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("{c}{}", .{ switch (reg_type) {
                        .data => @as(u8, 'd'),
                        .addr => @as(u8, 'a'),
                    }, int.extract(u3, this.opcode, at) });
                }
            };
        }
    };
}

/// Source data target that comes from the opcode
pub fn Opcode(Data: type, at: u4) type {
    return struct {
        pub fn decode(_: *Ctx, comptime _: Size, _: u16) @This() {
            return .{};
        }

        pub fn load(_: @This(), _: *Ctx, comptime _: Size, opcode: u16) Data {
            return int.extract(Data, opcode, at);
        }

        pub fn store(_: @This(), _: *Ctx, comptime _: Size, _: u16, _: void) void {}

        pub fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("{f}", .{Formatter(Data){
                        .val = int.extract(Data, this.opcode, at),
                        .options = .{ .hex = false },
                    }});
                }
            };
        }
    };
}

/// Source data target that is just a constant value
pub fn Const(Type: type, val: Type) type {
    return struct {
        pub fn decode(_: *Ctx, comptime _: Size, _: u16) @This() {
            return .{};
        }

        pub fn load(_: @This(), _: *Ctx, comptime _: Size, _: u16) Type {
            return val;
        }

        pub fn store(_: @This(), _: *Ctx, comptime _: Size, _: u16, _: void) void {}

        pub fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(_: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("{f}", .{Formatter(Type){
                        .val = val,
                        .options = .{ .hex = false },
                    }});
                }
            };
        }
    };
}

/// Delays created by evaluating reg-reg operands
pub const RegRegDelay = struct {
    b: [2]usize = [2]usize{ 0, 0 },
    w: [2]usize = [2]usize{ 0, 0 },
    l: [2]usize = [2]usize{ 0, 0 },

    /// Gets the delay for the mode and size combination
    fn delay(comptime this: @This(), comptime size: Size, mode: u1) usize {
        return @field(this, @tagName(size))[mode];
    }
};

/// Register to Register specific addressing modes
pub const RegRegMode = enum(u1) {
    reg, // Data registers
    mem, // Memory to memory
};

/// Specialized register to register source and destination target
pub fn RegReg(comptime m: u4, comptime n: u4, comptime delay: RegRegDelay) type {
    return struct {
        mode: RegRegMode,
        reg: u3,

        pub fn decode(ctx: *Ctx, comptime size: Size, opcode: u16) @This() {
            const mode = int.extract(RegRegMode, opcode, m);
            const reg = int.extract(u3, opcode, n);
            ctx.clk += delay.delay(size, @intFromEnum(mode));
            if (mode == .mem) {
                ctx.cpu.an(reg).* -= size.bits() / 8;
            }
            return .{ .mode = mode, .reg = reg };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime size: Size, _: u16) size.Int() {
            return switch (this.mode) {
                .reg => @truncate(ctx.cpu.d[this.reg]),
                .mem => ctx.read(size.Int(), ctx.cpu.an(this.reg).*),
            };
        }

        pub fn store(this: @This(), ctx: *Ctx, comptime size: Size, _: u16, data: size.Int()) void {
            switch (this.mode) {
                .reg => ctx.cpu.d[this.reg] = int.overwrite(ctx.cpu.d[this.reg], data),
                .mem => ctx.write(size.Int(), ctx.cpu.an(this.reg).*, data),
            }
        }

        pub fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const reg = int.extract(u3, this.opcode, n);
                    switch (int.extract(RegRegMode, this.opcode, m)) {
                        .reg => try writer.print("d{}", .{reg}),
                        .mem => try writer.print("-(a{})", .{reg}),
                    }
                }
            };
        }
    };
}

/// Effective address clock delays
pub const EaDelay = struct {
    none: std.EnumArray(Ctx.Mode, usize) = .initFill(0),
    b: std.EnumArray(Ctx.Mode, usize) = .initFill(0),
    w: std.EnumArray(Ctx.Mode, usize) = .initFill(0),
    l: std.EnumArray(Ctx.Mode, usize) = .initFill(0),

    /// Gets the delay for the mode and size combination
    fn delay(comptime this: @This(), comptime size: Size, mode: Ctx.Mode) usize {
        return @field(this, @tagName(size)).get(mode);
    }
};

/// Effective address source or destination target
pub fn Ea(comptime m: u4, comptime n: u4, comptime delay: EaDelay) type {
    return struct {
        mode: Ctx.Mode,
        addr: u32,

        pub fn decode(ctx: *Ctx, comptime size: Size, opcode: u16) @This() {
            const reg = int.extract(u3, opcode, n);
            const mode = Ctx.Mode.decode(int.extract(u3, opcode, m), reg);
            ctx.clk += delay.delay(size, mode);
            return .{ .mode = mode, .addr = mode.calc(ctx, size, reg) };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime size: Size, _: u16) size.Int() {
            return switch (this.mode) {
                .data_reg => @truncate(ctx.cpu.d[this.addr]),
                .addr_reg => @truncate(ctx.cpu.an(@truncate(this.addr)).*),
                .immediate => ctx.fetch(size.Int()),
                else => ctx.read(size.Int(), this.addr),
            };
        }

        pub fn store(this: @This(), ctx: *Ctx, comptime size: Size, _: u16, data: size.Int()) void {
            switch (this.mode) {
                .data_reg => ctx.cpu.d[this.addr] = int.overwrite(ctx.cpu.d[this.addr], data),
                .addr_reg => ctx.cpu.an(@truncate(this.addr)).* =
                    int.overwrite(ctx.cpu.an(@truncate(this.addr)).*, data),
                .immediate => {},
                else => ctx.write(size.Int(), this.addr, data),
            }
        }

        pub fn Disasm(comptime size: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const reg = int.extract(u3, this.opcode, n);
                    try writer.print("{f}", .{Ctx.Mode.Disasm(size){
                        .reader = this.reader,
                        .mode = Ctx.Mode.decode(int.extract(u3, this.opcode, m), reg),
                        .reg = reg,
                    }});
                }
            };
        }
    };
}

/// This will get the address of an effective addressing mode
pub fn Addr(comptime m: u4, comptime n: u4, comptime delay: EaDelay) type {
    return struct {
        pub fn decode(_: *Ctx, comptime _: Size, _: u16) @This() {
            return .{};
        }

        pub fn load(_: @This(), ctx: *Ctx, comptime size: Size, opcode: u16) u32 {
            const reg = int.extract(u3, opcode, n);
            const mode = Ctx.Mode.decode(int.extract(u3, opcode, m), reg);
            ctx.clk += delay.delay(size, mode);
            return mode.calc(ctx, .none, reg);
        }

        pub fn store(_: @This(), _: *Ctx, comptime _: Size, _: u16, _: void) void {}

        pub fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const reg = int.extract(u3, this.opcode, n);
                    try writer.print("{f}", .{Ctx.Mode.Disasm(.none){
                        .reader = this.reader,
                        .mode = Ctx.Mode.decode(int.extract(u3, this.opcode, m), reg),
                        .reg = reg,
                    }});
                }
            };
        }
    };
}

pub fn Multiple(
    comptime op: enum { store, load, addr },
    comptime addr_m: u4,
    comptime addr_n: u4,
) type {
    return struct {
        pub fn decode(ctx: *Ctx, comptime size: Size, opcode: u16) @This() {
            if (op == .addr) {
                return .{};
            }

            const reg = int.extract(u3, opcode, addr_n);
            const mode = Ctx.Mode.decode(int.extract(u3, opcode, addr_m), reg);
            const mask = @as(Ctx.RegMask, @bitCast(switch (mode) {
                .pre_dec => @bitReverse(ctx.fetch(u16)),
                else => ctx.fetch(u16),
            }));
            switch (op) {
                .load => {
                    switch (mode) {
                        .post_inc => {
                            mask.load(size, ctx, ctx.cpu.an(reg).*, .forward);
                            ctx.cpu.an(reg).* +%= @intCast(mask.count() * size.bits() / 8);
                        },
                        .pre_dec => {}, // Pre-dec is only used when storing
                        else => mask.load(size, ctx, mode.calc(ctx, size, reg), .forward),
                    }
                },
                .store => {
                    switch (mode) {
                        .post_inc => {}, // Post-inc is only used when loading
                        .pre_dec => {
                            ctx.cpu.an(reg).* -%= @intCast(mask.count() * size.bits() / 8);
                            mask.store(size, ctx, ctx.cpu.an(reg).*, .forward);
                        },
                        else => mask.store(size, ctx, mode.calc(ctx, size, reg), .forward),
                    }
                },
                .addr => {},
            }
            return .{};
        }

        pub fn load(_: @This(), _: *Ctx, comptime _: Size, _: u16) void {}

        pub fn store(_: @This(), _: *Ctx, comptime _: Size, _: u16, _: u32) void {}

        pub fn Disasm(comptime size: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const reg = int.extract(u3, this.opcode, addr_n);
                    const mode = Ctx.Mode.decode(int.extract(u3, this.opcode, addr_m), reg);
                    switch (op) {
                        .load, .store => {
                            const word = this.reader.takeInt(u16, .big) catch
                                return error.WriteFailed;
                            try writer.print("{f}", .{Ctx.RegMask.Disasm{
                                .reader = this.reader,
                                .mask = @bitCast(switch (mode) {
                                    .pre_dec => @bitReverse(word),
                                    else => word,
                                }),
                            }});
                        },
                        else => {
                            try writer.print("{f}", .{Ctx.Mode.Disasm(size){
                                .reader = this.reader,
                                .mode = mode,
                                .reg = reg,
                            }});
                        },
                    }
                }
            };
        }
    };
}

/// Move to and from peripherals
pub fn Peripheral(comptime addr_n: u4) type {
    return struct {
        disp: u32,
        reg: u3,

        pub fn decode(ctx: *Ctx, comptime _: Size, opcode: u16) @This() {
            return .{
                .disp = int.extend(u32, ctx.fetch(u16)),
                .reg = int.extract(u3, opcode, addr_n),
            };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime size: Size, _: u16) size.Int() {
            var addr = ctx.cpu.an(this.reg).* +% this.disp;
            var val: size.Int() = 0;
            inline for (0..size.bits() / 8) |byte| {
                val |= @as(size.Int(), ctx.read(u8, addr)) << size.bits() - 8 - byte * 8;
                addr += 2;
            }
            return val;
        }

        pub fn store(this: @This(), ctx: *Ctx, comptime size: Size, _: u16, data: size.Int()) void {
            var addr = ctx.cpu.an(this.reg).* +% this.disp;
            inline for (0..size.bits() / 8) |byte| {
                ctx.write(u8, addr, @truncate(data >> size.bits() - 8 - byte * 8));
                addr += 2;
            }
        }

        pub fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const reg = int.extract(u3, this.opcode, addr_n);
                    const disp = this.reader.takeInt(i16, .big) catch return error.WriteFailed;
                    try writer.print("(#{},a{})", .{ disp, reg });
                }
            };
        }
    };
}

/// Formatter options
pub const FormatOptions = struct {
    hex: bool = true,
};

/// Print out a type that can be converted to bits
fn Formatter(comptime Type: type) type {
    return struct {
        val: Type,
        options: FormatOptions,

        pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
            switch (@typeInfo(Type)) {
                .@"enum" => try writer.print("{t}", .{this.val}),
                .int => if (this.options.hex) {
                    try writer.print(std.fmt.comptimePrint(
                        "#${{x:0>{}}}",
                        .{@bitSizeOf(Type) / 4},
                    ), .{this.val});
                } else {
                    try writer.print("#{}", .{this.val});
                },
                else => try writer.print("{any}", .{this.val}),
            }
        }
    };
}
