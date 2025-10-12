const std = @import("std");

const int = @import("int");

const Ctx = @import("Ctx.zig");
const Size = Ctx.Size;

/// Binary decimal add operation
pub const Abcd = struct {
    pub fn op(ctx: *Ctx, comptime _: Size, src: u8, dst: u8) u8 {
        const result = Ctx.tobcd(Ctx.frombcd(src) + Ctx.frombcd(dst) + @intFromBool(ctx.cpu.sr.x));
        ctx.cpu.sr.x = result[1];
        ctx.cpu.sr.c = result[1];
        ctx.cpu.sr.z = @intFromBool(ctx.cpu.sr.z) & @intFromBool(result[0] == 0) == 1;
        return result[0];
    }
};

/// Normal addition operation
pub const Add = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
        const result = Arith(size).add(.{ dst, src });
        ctx.cpu.sr.x = result.carry;
        ctx.cpu.sr.n = int.negative(result.val);
        ctx.cpu.sr.z = result.val == 0;
        ctx.cpu.sr.v = result.overflow;
        ctx.cpu.sr.c = result.carry;
        return result.val;
    }
};

/// Address addition operation
pub const Adda = struct {
    pub fn op(_: *Ctx, comptime size: Size, src: size.Int(), dst: u32) u32 {
        return int.extend(u32, src) +% dst;
    }
};

/// Extended addition operation
pub const Addx = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
        const result = Arith(size).add(.{ dst, src, @intFromBool(ctx.cpu.sr.x) });
        ctx.cpu.sr.x = result.carry;
        ctx.cpu.sr.n = int.negative(result.val);
        ctx.cpu.sr.z = ctx.cpu.sr.z and result.val == 0;
        ctx.cpu.sr.v = result.overflow;
        ctx.cpu.sr.c = result.carry;
        return result.val;
    }
};

/// Bitwise logical operation
pub fn Logic(comptime mode: enum { @"and", @"or", eor }) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
            const result = switch (mode) {
                .@"and" => dst & src,
                .@"or" => dst | src,
                .eor => dst ^ src,
            };
            ctx.cpu.sr.n = int.negative(result);
            ctx.cpu.sr.z = result == 0;
            ctx.cpu.sr.v = false;
            ctx.cpu.sr.c = false;
            return result;
        }
    };
}

/// Arithmatic and logical shifts
pub fn Shift(comptime add_cycles: bool, comptime dir: enum { ar, al, lr, ll }) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
            // Calculate the base clock cycles of the operation
            if (add_cycles) {
                ctx.clk += if (size.bits() > 16) @as(usize, 4) else @as(usize, 2);
            }

            // Initialize the overflow flag
            ctx.cpu.sr.v = false;

            // Create the shfit results
            var last: ?bool = null;
            var result = int.castsign(switch (dir) {
                .al, .ar => .signed,
                .ll, .lr => .unsigned,
            }, dst);

            // Shift out each bit
            for (0..src & 64 - 1) |_| {
                last = switch (dir) {
                    .al, .ll => l: {
                        const msb = int.negative(result);
                        result <<= 1;
                        if (dir == .al and msb != last orelse msb) {
                            ctx.cpu.sr.v = true;
                        }
                        break :l msb;
                    },
                    .ar, .lr => r: {
                        const lsb = result & 1 == 1;
                        result >>= 1;
                        break :r lsb;
                    },
                };
                if (add_cycles) {
                    ctx.clk += 2;
                }
            }

            // Update the flags and return
            if (dir == .al and int.negative(result) != last orelse int.negative(result)) {
                ctx.cpu.sr.v = true;
            }
            ctx.cpu.sr.x = last orelse ctx.cpu.sr.x;
            ctx.cpu.sr.n = int.negative(result);
            ctx.cpu.sr.z = result == 0;
            ctx.cpu.sr.c = last orelse false;
            return @bitCast(result);
        }
    };
}

/// Conditional branching operations
pub const Branch = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, src: Ctx.Cond, dst: size.Int()) void {
        const base = ctx.cpu.pc -% (size.bits() / 16 * 2);
        if (src == .f) {
            ctx.clk += 2;
            ctx.push(u32, ctx.cpu.pc);
            ctx.cpu.pc = base +% int.extend(u32, dst);
        } else if (!src.value(ctx.cpu.sr)) {
            ctx.clk += 4;
        } else {
            ctx.clk += 2 + if (size.bits() == 8) 4 else 0;
            ctx.cpu.pc = base +% int.extend(u32, dst);
        }
    }
};

/// Do something with a bit
pub fn Bit(comptime mode: enum { set, chg, clr, tst }) type {
    return struct {
        pub fn op(
            ctx: *Ctx,
            comptime size: Size,
            src: anytype,
            dst: size.Int(),
        ) if (mode != .tst) size.Int() else void {
            const bit = @as(size.Int(), 1) << @truncate(src);
            ctx.clk += if (mode != .tst and @bitSizeOf(@TypeOf(src)) > 4 and src >= 16) 2 else 0;
            ctx.cpu.sr.z = dst & bit == 0;
            return switch (mode) {
                .set => dst | bit,
                .chg => dst ^ bit,
                .clr => dst & ~bit,
                .tst => {},
            };
        }
    };
}

/// Check a variable against some bounds
pub const Chk = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) void {
        if (int.castsign(.signed, src) < 0) {
            ctx.clk += 4;
            ctx.cpu.sr.n = true;
            Ctx.Vector.chk.handle(ctx);
        } else if (int.castsign(.signed, src) > int.castsign(.signed, dst)) {
            ctx.cpu.sr.n = false;
            Ctx.Vector.chk.handle(ctx);
        } else {
            ctx.clk += 6;
        }
    }
};

/// Clear some data
pub const Clr = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, _: size.Int()) size.Int() {
        ctx.cpu.sr.n = false;
        ctx.cpu.sr.z = true;
        ctx.cpu.sr.v = false;
        ctx.cpu.sr.c = false;
        return 0;
    }
};

/// Compare two operands
pub const Cmp = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) void {
        const result = Arith(size).sub(.{ dst, src });
        ctx.cpu.sr.n = int.negative(result.val);
        ctx.cpu.sr.z = result.val == 0;
        ctx.cpu.sr.v = result.overflow;
        ctx.cpu.sr.c = result.carry;
    }
};

/// Compare two operands
pub const Cmpa = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: u32) void {
        const result = Arith(.l).sub(.{ dst, int.extend(u32, src) });
        ctx.cpu.sr.n = int.negative(result.val);
        ctx.cpu.sr.z = result.val == 0;
        ctx.cpu.sr.v = result.overflow;
        ctx.cpu.sr.c = result.carry;
    }
};

/// Test condition, decrement, and branch
pub const Dbcc = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, cnd: Ctx.Cond, src: u16, dst: size.Int()) size.Int() {
        // Check the condition
        if (cnd.value(ctx.cpu.sr)) {
            ctx.clk += 4;
            return dst;
        }

        // Check for the counter expiring or condition
        const result = dst -% 1;
        if (result == std.math.maxInt(size.Int())) {
            ctx.clk += 6;
            return result;
        }

        // Branch
        ctx.cpu.pc = ctx.cpu.pc -% 2 +% int.extend(u32, src);
        ctx.clk += 2;
        return result;
    }
};

/// Signed and unsigned divide operation
pub fn Div(comptime sign: std.builtin.Signedness) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime _: Size, src: u16, dst: u3) void {
            // Get the numerator and denominator
            const res_num: std.meta.Int(sign, 32) = @bitCast(ctx.cpu.d[dst]);
            const res_den: std.meta.Int(sign, 32) = int.castsign(sign, src);
            if (res_den == 0) {
                ctx.clk += 4;
                Ctx.Vector.divzero.handle(ctx);
                return;
            }
            ctx.clk += switch (sign) {
                .signed => 12 + @as(usize, @intFromBool(res_den < 0)) * 2,
                .unsigned => 2,
            };

            // Compute the division and check for overflow
            ctx.cpu.sr.c = false;
            ctx.cpu.sr.v = false;
            const res_quo = std.math.cast(i16, @divTrunc(res_num, res_den)) orelse {
                ctx.cpu.sr.v = true;
                return;
            };
            const res_rem = std.math.cast(i16, @rem(res_num, res_den)) orelse {
                ctx.cpu.sr.v = true;
                return;
            };
            ctx.cpu.sr.n = int.negative(res_quo);
            ctx.cpu.sr.z = res_quo == 0;

            // Calculate the cycle count
            switch (sign) {
                .signed => {
                    ctx.clk += 12;
                    if (res_num < 0) {
                        ctx.clk += 4;
                    } else {
                        ctx.clk += if (res_den < 0) @as(usize, 6) else @as(usize, 2);
                    }

                    var quo: u16 = @bitCast(res_quo);
                    for (0..15) |_| {
                        quo <<= 1;
                        ctx.clk += 6 + @as(usize, ~@as(u1, @truncate(quo >> 15))) * 2;
                    }
                },
                .unsigned => {
                    var num: u32 = @as(u32, @bitCast(res_num));
                    var msb: u2 = @as(u1, @truncate(num >> 31));
                    ctx.clk += 6;
                    for (0..15) |_| {
                        // Calculate clock cycles
                        if (msb & 1 != 0) {
                            ctx.clk += 4;
                        } else {
                            ctx.clk += 6 + if (msb & 2 != 0) @as(usize, 2) else @as(usize, 0);
                        }

                        // Shift numbers
                        msb = msb << 1 | @as(u1, @truncate(num >> 31));
                        num <<= 1;
                        if (num >> 16 > res_den) {
                            num -= res_den << 16;
                        }
                    }
                },
            }

            // Return the packed results
            ctx.cpu.d[dst] = @as(u32, @as(u16, @bitCast(res_rem))) << 16 |
                @as(u16, @bitCast(res_quo));
        }
    };
}

/// Exchange two registers
pub fn Exg(comptime src_reg: enum { data, addr }, comptime dst_reg: enum { data, addr }) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime _: Size, src: u3, dst: u3) void {
            ctx.clk += 2;
            switch (@as(u2, @intFromEnum(src_reg)) << 1 | @intFromEnum(dst_reg)) {
                0b00 => std.mem.swap(u32, &ctx.cpu.d[src], &ctx.cpu.d[dst]),
                0b10 => std.mem.swap(u32, &ctx.cpu.a[src], &ctx.cpu.d[dst]),
                0b01 => std.mem.swap(u32, &ctx.cpu.d[src], &ctx.cpu.a[dst]),
                0b11 => std.mem.swap(u32, &ctx.cpu.a[src], &ctx.cpu.a[dst]),
            }
        }
    };
}

/// Extend data register
pub const Ext = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, dst: u3) void {
        ctx.cpu.sr.v = false;
        ctx.cpu.sr.c = false;
        switch (size) {
            .b, .none => unreachable,
            .w => {
                const res = int.extend(u16, @as(u8, @truncate(ctx.cpu.d[dst])));
                ctx.cpu.d[dst] = int.overwrite(ctx.cpu.d[dst], res);
                ctx.cpu.sr.n = int.negative(res);
                ctx.cpu.sr.z = res == 0;
            },
            .l => {
                const res = int.extend(u32, @as(u16, @truncate(ctx.cpu.d[dst])));
                ctx.cpu.d[dst] = res;
                ctx.cpu.sr.n = int.negative(res);
                ctx.cpu.sr.z = res == 0;
            },
        }
    }
};

/// Run illegal instruction handler
pub const Illegal = struct {
    pub fn op(ctx: *Ctx, comptime _: Size) void {
        Ctx.Vector.illegal.handle(ctx);
    }
};

/// Jump to address
pub const Jmp = struct {
    pub fn op(ctx: *Ctx, comptime _: Size, dst: u32) void {
        ctx.cpu.pc = dst;
    }
};

/// Jump to subroutine
pub const Jsr = struct {
    pub fn op(ctx: *Ctx, comptime _: Size, dst: u32) void {
        ctx.push(u32, ctx.cpu.pc);
        ctx.cpu.pc = dst;
    }
};

/// Link and allocate
pub const Link = struct {
    pub fn op(ctx: *Ctx, comptime _: Size, src: u16, dst: u32) u32 {
        ctx.push(u32, dst);
        const stack = ctx.cpu.a[7];
        ctx.cpu.a[7] +%= int.extend(u32, src);
        return stack;
    }
};

/// Move src to dst
pub fn Move(comptime update_flags: bool) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime size: Size, src: anytype, _: size.Int()) size.Int() {
            if (update_flags) {
                ctx.cpu.sr.n = int.negative(src);
                ctx.cpu.sr.z = src == 0;
                ctx.cpu.sr.v = false;
                ctx.cpu.sr.c = false;
            }
            return int.extend(size.Int(), src);
        }
    };
}

/// Move sign extended source to dst
pub fn MoveExtend(comptime Dst: type) type {
    return struct {
        pub fn op(_: *Ctx, comptime size: Size, src: size.Int(), _: Dst) Dst {
            return int.extend(Dst, src);
        }
    };
}

/// Signed multiply
pub const Muls = struct {
    pub fn op(ctx: *Ctx, comptime _: Size, src: u16, dst: u16) u32 {
        ctx.clk += 34;
        var shifter = @as(u17, src) << 1;
        inline for (0..16) |_| {
            ctx.clk += switch (@as(u2, @truncate(shifter))) {
                0b01, 0b10 => 2,
                else => 0,
            };
            shifter >>= 1;
        }

        const res = @as(i32, int.castsign(.signed, src)) * @as(i32, int.castsign(.signed, dst));
        ctx.cpu.sr.n = int.negative(res);
        ctx.cpu.sr.z = res == 0;
        ctx.cpu.sr.v = false;
        ctx.cpu.sr.c = false;
        return @bitCast(res);
    }
};

/// Unsigned multiply
pub const Mulu = struct {
    pub fn op(ctx: *Ctx, comptime _: Size, src: u16, dst: u16) u32 {
        const res = @as(u32, src) * @as(u32, dst);
        ctx.clk += 34 + 2 * @as(usize, @popCount(src));
        ctx.cpu.sr.n = int.negative(res);
        ctx.cpu.sr.z = res == 0;
        ctx.cpu.sr.v = false;
        ctx.cpu.sr.c = false;
        return res;
    }
};

/// Subtract binary decimal with extend
pub const Sbcd = struct {
    pub fn op(ctx: *Ctx, comptime _: Size, src: u8, dst: u8) u8 {
        var result = Ctx.frombcd(src);
        ctx.cpu.sr.c = false;

        result -%= Ctx.frombcd(dst);
        if (result > 99) {
            result -= 156;
            ctx.cpu.sr.c = true;
        }

        result -%= @intFromBool(ctx.cpu.sr.x);
        if (result > 99) {
            result -= 156;
            ctx.cpu.sr.c = true;
        }

        result = Ctx.tobcd(result)[0];
        ctx.cpu.sr.x = ctx.cpu.sr.c;
        ctx.cpu.sr.z = @intFromBool(ctx.cpu.sr.z) & @intFromBool(result == 0) == 1;
        return result;
    }
};

/// Normal subtraction operation
pub fn Sub(comptime minuend: enum { dst, src }) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
            const result = Arith(size).sub(switch (minuend) {
                .dst => .{ dst, src },
                .src => .{ src, dst },
            });
            ctx.cpu.sr.x = result.carry;
            ctx.cpu.sr.n = int.negative(result.val);
            ctx.cpu.sr.z = result.val == 0;
            ctx.cpu.sr.v = result.overflow;
            ctx.cpu.sr.c = result.carry;
            return result.val;
        }
    };
}

/// Extended subtraction operation
pub fn Subx(comptime minuend: enum { dst, src }) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
            const result = Arith(size).sub(switch (minuend) {
                .dst => .{ dst, src },
                .src => .{ src, dst },
            } ++ .{@intFromBool(ctx.cpu.sr.x)});
            ctx.cpu.sr.x = result.carry;
            ctx.cpu.sr.n = int.negative(result.val);
            ctx.cpu.sr.z = ctx.cpu.sr.z and result.val == 0;
            ctx.cpu.sr.v = result.overflow;
            ctx.cpu.sr.c = result.carry;
            return result.val;
        }
    };
}

/// Push value onto the stack
pub const Push = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, dst: size.Int()) void {
        ctx.push(size.Int(), dst);
    }
};

/// Rotate left and rigth
pub fn Rotate(comptime add_cycles: bool, comptime dir: enum { r, l, rx, lx }) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
            // Calculate the base clock cycles of the operation
            const shift = src & 64 - 1;
            ctx.cpu.sr.v = false;
            if (add_cycles) {
                ctx.clk += shift * 2 + if (size.bits() > 16) @as(usize, 4) else @as(usize, 2);
            }

            // The rotate result
            const Data = switch (dir) {
                .rx, .lx => std.meta.Int(.unsigned, size.bits() + 1),
                .r, .l => size.Int(),
            };
            var result = switch (dir) {
                .rx => @as(Data, dst) + (@as(Data, @intFromBool(ctx.cpu.sr.x)) << size.bits()),
                .lx => (@as(Data, dst) << 1) + @intFromBool(ctx.cpu.sr.x),
                .r, .l => @as(Data, dst),
            };
            if (shift > 1) {
                result = switch (dir) {
                    .r, .rx => std.math.rotr(Data, result, shift - 1),
                    .l, .lx => std.math.rotl(Data, result, shift - 1),
                };
            }

            // Get the carry flag
            if (shift > 0) {
                ctx.cpu.sr.c = int.negative(result);
                result = switch (dir) {
                    .r, .rx => std.math.rotr(Data, result, 1),
                    .l, .lx => std.math.rotl(Data, result, 1),
                };
            } else {
                ctx.cpu.sr.c = switch (dir) {
                    .rx, .lx => ctx.cpu.sr.x,
                    .r, .l => false,
                };
            }

            // Get the extend flag
            switch (dir) {
                .rx => {
                    ctx.cpu.sr.x = result & @as(Data, 1) << size.bits() == 1;
                    result &= std.math.maxInt(size.Int());
                },
                .lx => {
                    ctx.cpu.sr.x = result & 1 == 1;
                    result &= ~@as(Data, 1);
                },
                .l, .r => {},
            }

            // Get the negative and zero flags
            ctx.cpu.sr.n = int.negative(result);
            ctx.cpu.sr.z = result == 0;
            return switch (dir) {
                .rx => @truncate(result),
                .lx => @truncate(result >> 1),
                .r, .l => result,
            };
        }
    };
}

/// Do an arithmatic operation and get the results
fn Arith(comptime size: Size) type {
    return struct {
        val: size.Int(),
        overflow: bool,
        carry: bool,

        /// Adds integers (taken in as a tuple)
        inline fn add(ints: anytype) @This() {
            var this = @This(){ .val = ints[0], .overflow = false, .carry = false };
            inline for (ints, 0..) |val, idx| {
                if (comptime idx == 0) {
                    continue;
                }
                this.overflow = @intFromBool(this.overflow) | @addWithOverflow(
                    int.castsign(.signed, this.val),
                    int.castsign(.signed, val),
                )[1] == 1;
                this.carry = @intFromBool(this.carry) | @addWithOverflow(
                    int.castsign(.unsigned, this.val),
                    int.castsign(.unsigned, val),
                )[1] == 1;
                this.val +%= int.castsign(.unsigned, val);
            }
            return this;
        }

        /// Subtracts integers (taken in as a tuple)
        inline fn sub(ints: anytype) @This() {
            var this = @This(){ .val = ints[0], .overflow = false, .carry = false };
            inline for (ints, 0..) |val, idx| {
                if (comptime idx == 0) {
                    continue;
                }
                this.overflow = @intFromBool(this.overflow) | @subWithOverflow(
                    int.castsign(.signed, this.val),
                    int.castsign(.signed, val),
                )[1] == 1;
                this.carry = @intFromBool(this.carry) | @subWithOverflow(
                    int.castsign(.unsigned, this.val),
                    int.castsign(.unsigned, val),
                )[1] == 1;
                this.val -%= int.castsign(.unsigned, val);
            }
            return this;
        }
    };
}

test "add" {
    try std.testing.expectEqual(
        Arith(.b){ .val = 16 + 16 + 8, .overflow = false, .carry = false },
        Arith(.b).add(.{ @as(u8, 8), @as(u8, 16), @as(u8, 16) }),
    );
    try std.testing.expectEqual(
        Arith(.w){ .val = @as(u16, 0xFFFF) +% @as(u16, 1), .overflow = false, .carry = true },
        Arith(.w).add(.{ @as(u16, 0xFFFF), @as(u16, 1) }),
    );
}

test "sub" {
    try std.testing.expectEqual(
        Arith(.b){ .val = @bitCast(@as(i8, 16) - @as(i8, 32)), .overflow = false, .carry = true },
        Arith(.b).sub(.{ @as(u8, 16), @as(u8, 32) }),
    );
    try std.testing.expectEqual(
        Arith(.w){ .val = @as(u16, 0x8000) -% @as(u16, 100), .overflow = true, .carry = false },
        Arith(.w).sub(.{ @as(u16, 0x8000), @as(u16, 100) }),
    );
}
