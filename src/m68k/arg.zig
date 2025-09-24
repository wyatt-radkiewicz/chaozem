const std = @import("std");

const int = @import("int");

const Ctx = @import("Ctx.zig");

/// Data register source or
pub fn DataReg(n: u4) type {
    return struct {
        n: u3,

        pub fn decode(_: *Ctx, comptime _: Ctx.Size, opcode: u16) @This() {
            return .{ .n = int.extract(u3, opcode, n) };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime size: Ctx.Size, _: u16) size.Int(.unsigned) {
            return @truncate(ctx.cpu.d[this.n]);
        }

        pub fn store(
            this: @This(),
            ctx: *Ctx,
            comptime size: Ctx.Size,
            _: u16,
            data: size.Int(.unsigned),
        ) void {
            ctx.cpu.d[this.n] = int.overwrite(ctx.cpu.d[this.n], data);
        }

        pub fn Disasm(comptime _: Ctx.Size) type {
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

        pub fn decode(_: *Ctx, comptime _: Ctx.Size, opcode: u16) @This() {
            return .{ .n = int.extract(u3, opcode, n) };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime _: Ctx.Size, _: u16) u32 {
            return ctx.cpu.a[this.n];
        }

        pub fn store(this: @This(), ctx: *Ctx, comptime _: Ctx.Size, _: u16, data: u32) void {
            ctx.cpu.a[this.n] = data;
        }

        pub fn Disasm(comptime _: Ctx.Size) type {
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

/// Status register source or destination target
pub const Status = struct {
    pub fn decode(_: *Ctx, comptime size: Ctx.Size, _: u16) @This() {
        if (size == .l) {
            @compileError("Status register is only 16 bits!");
        }
        return .{};
    }

    pub fn load(_: @This(), ctx: *Ctx, comptime size: Ctx.Size, _: u16) size.Int(.unsigned) {
        const bits: u16 = @bitCast(ctx.cpu.sr);
        return @truncate(bits);
    }

    pub fn store(
        _: @This(),
        ctx: *Ctx,
        comptime size: Ctx.Size,
        _: u16,
        data: size.Int(.unsigned),
    ) void {
        const bits: u16 = @bitCast(ctx.cpu.sr);
        ctx.cpu.sr = @bitCast(int.overwrite(bits, data));
    }

    pub fn Disasm(comptime size: Ctx.Size) type {
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

/// Immediate source data target
pub const Imm = struct {
    pub fn decode(_: *Ctx, comptime _: Ctx.Size, _: u16) @This() {
        return .{};
    }

    pub fn load(_: @This(), ctx: *Ctx, comptime size: Ctx.Size, _: u16) size.Int(.unsigned) {
        return ctx.fetch(size.Int(.unsigned));
    }

    pub fn Disasm(comptime size: Ctx.Size) type {
        return struct {
            reader: *std.io.Reader,
            opcode: u16,

            pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                switch (size) {
                    .none => unreachable,
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

/// Source data target that comes from the opcode
pub fn Opcode(Data: type, at: u4) type {
    return struct {
        pub fn decode(_: *Ctx, comptime _: Ctx.Size, _: u16) @This() {
            return .{};
        }

        pub fn load(_: @This(), _: *Ctx, comptime _: Ctx.Size, opcode: u16) Data {
            return int.extract(Data, opcode, at);
        }

        pub fn Disasm(comptime _: Ctx.Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("{s}{any}", .{ switch (@typeInfo(Data)) {
                        .int => "#",
                        else => "",
                    }, int.extract(Data, this.opcode, at) });
                }
            };
        }
    };
}

/// Source data target that is just a constant value
pub fn Const(Type: type, val: Type) type {
    return struct {
        pub fn decode(_: *Ctx, comptime _: Ctx.Size, _: u16) @This() {
            return .{};
        }

        pub fn load(_: @This(), _: *Ctx, comptime _: Ctx.Size, _: u16) Type {
            return val;
        }

        pub fn Disasm(comptime _: Ctx.Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(_: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("{s}{any}", .{ switch (@typeInfo(Type)) {
                        .int => "#",
                        else => "",
                    }, val });
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
    fn delay(comptime this: @This(), comptime size: Ctx.Size, mode: u1) usize {
        return @field(this, @tagName(size))[mode];
    }
};

/// Specialized register to register source and destination target
pub fn RegReg(comptime m: u4, comptime n: u4, comptime delay: RegRegDelay) type {
    return struct {
        mode: u1,
        reg: u3,

        pub fn decode(ctx: *Ctx, comptime size: Ctx.Size, opcode: u16) @This() {
            const mode = int.extract(u1, opcode, m);
            const reg = int.extract(u3, opcode, n);
            ctx.clk += delay.delay(size, mode);
            if (mode == 1) {
                ctx.cpu.a[reg] -= size.bits() / 8;
            }
            return .{ .mode = mode, .reg = reg };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime size: Ctx.Size, _: u16) size.Int(.unsigned) {
            return switch (this.mode) {
                0 => @truncate(ctx.cpu.d[this.reg]),
                1 => ctx.read(size.Int(.unsigned), ctx.cpu.a[this.reg]),
            };
        }

        pub fn store(
            this: @This(),
            ctx: *Ctx,
            comptime size: Ctx.Size,
            _: u16,
            data: size.Int(.unsigned),
        ) void {
            switch (this.mode) {
                0 => ctx.cpu.d[this.reg] = int.overwrite(ctx.cpu.d[this.reg], data),
                1 => ctx.write(size.Int(.unsigned), ctx.cpu.a[this.reg], data),
            }
        }

        pub fn Disasm(comptime _: Ctx.Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const reg = int.extract(u3, this.opcode, n);
                    switch (int.extract(u1, this.opcode, m)) {
                        0 => try writer.print("d{}", .{reg}),
                        1 => try writer.print("-(a{})", .{reg}),
                    }
                }
            };
        }
    };
}

/// Effective address clock delays
pub const EaDelay = struct {
    b: std.EnumArray(Ctx.Mode, usize) = .initFill(0),
    w: std.EnumArray(Ctx.Mode, usize) = .initFill(0),
    l: std.EnumArray(Ctx.Mode, usize) = .initFill(0),

    /// Gets the delay for the mode and size combination
    fn delay(comptime this: @This(), comptime size: Ctx.Size, mode: Ctx.Mode) usize {
        return @field(this, @tagName(size)).get(mode);
    }
};

/// Effective address source or destination target
pub fn Ea(comptime m: u4, comptime n: u4, comptime delay: EaDelay) type {
    return struct {
        mode: Ctx.Mode,
        addr: u32,

        pub fn decode(ctx: *Ctx, comptime size: Ctx.Size, opcode: u16) @This() {
            const reg = int.extract(u3, opcode, n);
            const mode = Ctx.Mode.decode(int.extract(u3, opcode, m), reg);
            ctx.clk += delay.delay(size, mode);
            return .{ .mode = mode, .addr = mode.calc(ctx, size, reg) };
        }

        pub fn load(this: @This(), ctx: *Ctx, comptime size: Ctx.Size, _: u16) size.Int(.unsigned) {
            return switch (this.mode) {
                .data_reg => @truncate(ctx.cpu.d[this.addr]),
                .addr_reg => @truncate(ctx.cpu.a[this.addr]),
                .immediate => ctx.fetch(size.Int(.unsigned)),
                else => ctx.read(size.Int(.unsigned), this.addr),
            };
        }

        pub fn store(
            this: @This(),
            ctx: *Ctx,
            comptime size: Ctx.Size,
            _: u16,
            data: size.Int(.unsigned),
        ) void {
            switch (this.mode) {
                .data_reg => ctx.cpu.d[this.addr] = int.overwrite(ctx.cpu.d[this.addr], data),
                .addr_reg => ctx.cpu.a[this.addr] = int.overwrite(ctx.cpu.a[this.addr], data),
                .immediate => {},
                else => ctx.write(size.Int(.unsigned), this.addr, data),
            }
        }

        pub fn Disasm(comptime size: Ctx.Size) type {
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
