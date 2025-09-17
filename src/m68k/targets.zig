const std = @import("std");

const Exec = @import("Exec.zig");

/// Data register source or
fn DataTarget(n: u4) type {
    return struct {
        n: u3,

        fn Data(comptime size: Exec.Size) type {
            return size.Int(.unsigned);
        }

        fn init(_: *Exec, comptime _: Size, opcode: u16) @This() {
            return .{ .n = extract(u3, opcode, n) };
        }

        fn load(this: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
            return @truncate(exec.cpu.d[this.n]);
        }

        fn store(this: @This(), exec: *Exec, comptime size: Size, _: u16, data: Data(size)) void {
            exec.cpu.d[this.n] = overwrite(exec.cpu.d[this.n], data);
        }

        fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("d{}", .{extract(u3, this.opcode, n)});
                }
            };
        }
    };
}

/// Address register source
fn AddrTarget(n: u4) type {
    return struct {
        n: u3,

        fn Data(comptime _: Size) type {
            return u32;
        }

        fn init(_: *Exec, comptime _: Size, opcode: u16) @This() {
            return .{ .n = extract(u3, opcode, n) };
        }

        fn load(this: @This(), exec: *Exec, comptime _: Size, _: u16) u32 {
            return exec.cpu.a[this.n];
        }

        fn store(this: @This(), exec: *Exec, comptime _: Size, _: u16, data: u32) void {
            exec.cpu.a[this.n] = data;
        }

        fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("a{}", .{extract(u3, this.opcode, n)});
                }
            };
        }
    };
}

/// Status register source or destination target
const SrTarget = struct {
    fn Data(comptime size: Size) type {
        return switch (size) {
            .l => @compileError("Status register is only 16 bits!"),
            else => size.Int(.unsigned),
        };
    }

    fn init(_: *Exec, comptime _: Size, _: u16) @This() {
        return .{};
    }

    fn load(_: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
        const bits: u16 = @bitCast(exec.cpu.sr);
        return @truncate(bits);
    }

    fn store(_: @This(), exec: *Exec, comptime size: Size, _: u16, data: Data(size)) void {
        const bits: u16 = @bitCast(exec.cpu.sr);
        exec.cpu.sr = @bitCast(overwrite(bits, data));
    }

    fn Disasm(comptime size: Size) type {
        return struct {
            reader: *std.io.Reader,
            opcode: u16,

            pub fn format(_: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                try writer.print("{s}", .{switch (size) {
                    .b => "ccr",
                    .w => "sr",
                    .l => @compileError("Expected byte or word with status register disasm!"),
                }});
            }
        };
    }
};

/// Immediate source data target
const ImmTarget = struct {
    fn Data(comptime size: Size) type {
        return size.Int(.unsigned);
    }

    fn init(_: *Exec, comptime _: Size, _: u16) @This() {
        return .{};
    }

    fn load(_: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
        return exec.fetch(Data(size));
    }

    fn store(_: @This(), _: *Exec, comptime size: Size, _: u16, _: Data(size)) void {}

    fn Disasm(comptime size: Size) type {
        return struct {
            reader: *std.io.Reader,
            opcode: u16,

            pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                switch (size) {
                    .b => try writer.print("#${x:0>2}", .{@as(u8, @truncate(
                        this.reader.takeInt(u16, .big) catch return error.WriteFailed,
                    ))}),
                    .w => try writer.print("#${x:0>4}", .{this.reader.takeInt(u16, .big) catch
                        return error.WriteFailed}),
                    .l => try writer.print("#${x:0>8}", .{this.reader.takeInt(u32, .big) catch
                        return error.WriteFailed}),
                }
            }
        };
    }
};

/// Source data target that comes from t
fn QuickTarget(Int: type, at: u4) type {
    return struct {
        fn Data(comptime size: Size) type {
            return size.Int(.unsigned);
        }

        fn init(_: *Exec, comptime _: Size, _: u16) @This() {
            return .{};
        }

        fn load(_: @This(), _: *Exec, comptime size: Size, opcode: u16) Data(size) {
            const int = extract(Int, opcode, at);
            return switch (@typeInfo(Int).int.signedness) {
                .signed => extend(Data(size), int),
                .unsigned => int,
            };
        }

        fn store(_: @This(), _: *Exec, comptime size: Size, _: u16, _: Data(size)) void {}

        fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("#{}", .{extract(Int, this.opcode, at)});
                }
            };
        }
    };
}

/// Source data target that comes from the o
fn ConstTarget(Type: type, val: Type) type {
    return struct {
        fn Data(comptime _: Size) type {
            return Type;
        }

        fn init(_: *Exec, comptime _: Size, _: u16) @This() {
            return .{};
        }

        fn load(_: @This(), _: *Exec, comptime _: Size, _: u16) Type {
            return val;
        }

        fn store(_: @This(), _: *Exec, comptime _: Size, _: u16, _: Type) void {}
    };
}

/// Delays created by evaluating reg-reg operands
const RegRegDelay = struct {
    b: [2]usize = [2]usize{ 0, 0 },
    w: [2]usize = [2]usize{ 0, 0 },
    l: [2]usize = [2]usize{ 0, 0 },

    /// Gets the delay for the mode and size combination
    fn delay(comptime this: @This(), comptime size: Size, mode: u1) usize {
        return @field(this, @tagName(size))[mode];
    }
};

/// Specialized register to register source and destination target
    _ = m; // autofix
    _ = n; // autofix
    _ = delay; // autofix
fn RegRegTarget(comptime m: u4, comptime n: u4, comptime delay: RegRegDelay) type {
    return struct {
        mode: u1,
        reg: u3,

        fn Data(comptime size: Size) type {
            return size.Int(.unsigned);
        }

        fn init(exec: *Exec, comptime size: Size, opcode: u16) @This() {
            const mode = extract(u1, opcode, m);
            const reg = extract(u3, opcode, n);
            exec.clk += delay.delay(size, mode);
            if (mode == 1) {
                exec.cpu.a[reg] -= size.bitSize() / 8;
            }
            return .{ .mode = mode, .reg = reg };
        }

        fn load(this: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
            return switch (this.mode) {
                0 => @truncate(exec.cpu.d[this.reg]),
                1 => exec.read(Data(size), exec.cpu.a[this.reg]),
            };
        }

        fn store(this: @This(), exec: *Exec, comptime size: Size, _: u16, data: Data(size)) void {
            switch (this.mode) {
                0 => exec.cpu.d[this.reg] = overwrite(exec.cpu.d[this.reg], data),
                1 => exec.write(Data(size), exec.cpu.a[this.reg], data),
            }
        }

        fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const reg = extract(u3, this.opcode, n);
                    switch (extract(u1, this.opcode, m)) {
                        0 => try writer.print("d{}", .{reg}),
                        1 => try writer.print("-(a{})", .{reg}),
                    }
                }
            };
        }
    };
}

/// Address modes
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
    fn calc(this: @This(), exec: *Exec, comptime size: Size, reg: u3) u32 {
        return switch (this) {
            .data_reg, .addr_reg => reg,
            .indirect => exec.cpu.a[reg],
            .post_inc => post_inc: {
                const addr = exec.cpu.a[reg];
                exec.cpu.a[reg] += size.bitSize() / 8;
                break :post_inc addr;
            },
            .pre_dec => pre_dec: {
                exec.cpu.a[reg] -= size.bitSize() / 8;
                exec.clk += 2;
                break :pre_dec exec.cpu.a[reg];
            },
            .addr_disp => exec.cpu.a[reg] +% extend(u32, exec.fetch(u16)),
            .addr_idx => exec.cpu.a[reg] +% exec.fetch(Index).getDisp(exec),
            .abs_word => extend(u32, exec.fetch(u16)),
            .abs_long => exec.fetch(u32),
            .pc_disp => exec.cpu.pc +% extend(u32, exec.fetch(u16)),
            .pc_idx => exec.cpu.pc +% exec.fetch(Index).getDisp(exec),
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
        fn getDisp(this: @This(), exec: *Exec) u32 {
            exec.clk += 2;
            const reg = switch (this.m) {
                0 => exec.cpu.d[this.n],
                1 => exec.cpu.a[this.n],
            };
            return switch (this.size) {
                0 => extend(u32, @as(u16, @truncate(reg))),
                1 => reg,
            } +% extend(u32, this.disp);
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

/// Effective address clock delays
const EaDelay = struct {
    b: std.EnumArray(Mode, usize) = .initFill(0),
    w: std.EnumArray(Mode, usize) = .initFill(0),
    l: std.EnumArray(Mode, usize) = .initFill(0),

    /// Gets the delay for the mode and size combination
    fn delay(comptime this: @This(), comptime size: Size, mode: Mode) usize {
        return @field(this, @tagName(size)).get(mode);
    }
};

/// Effective address source or destination target
    _ = m; // autofix
    _ = n; // autofix
    _ = delay; // autofix
fn EaTarget(comptime m: u4, comptime n: u4, comptime delay: EaDelay) type {
    return struct {
        mode: Mode,
        addr: u32,

        fn Data(comptime size: Size) type {
            return size.Int(.unsigned);
        }

        fn init(exec: *Exec, comptime size: Size, opcode: u16) @This() {
            const reg = extract(u3, opcode, n);
            const mode = Mode.decode(extract(u3, opcode, m), reg);
            exec.clk += delay.delay(size, mode);
            return .{ .mode = mode, .addr = mode.calc(exec, size, reg) };
        }

        fn load(this: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
            return switch (this.mode) {
                .data_reg => @truncate(exec.cpu.d[this.addr]),
                .addr_reg => @truncate(exec.cpu.a[this.addr]),
                .immediate => exec.fetch(size.Int(.unsigned)),
                else => exec.read(size.Int(.unsigned), this.addr),
            };
        }

        fn store(this: @This(), exec: *Exec, comptime size: Size, _: u16, data: Data(size)) void {
            switch (this.mode) {
                .data_reg => exec.cpu.d[this.addr] = overwrite(exec.cpu.d[this.addr], data),
                .addr_reg => exec.cpu.a[this.addr] = overwrite(exec.cpu.a[this.addr], data),
                .immediate => {},
                else => exec.write(size.Int(.unsigned), this.addr, data),
            }
        }

        fn Disasm(comptime size: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const reg = extract(u3, this.opcode, n);
                    try writer.print("{f}", .{Mode.Disasm(size){
                        .reader = this.reader,
                        .mode = Mode.decode(extract(u3, this.opcode, m), reg),
                        .reg = reg,
                    }});
                }
            };
        }
    };
}
