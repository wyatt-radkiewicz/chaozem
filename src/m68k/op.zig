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
        ctx.cpu.sr.z = @intFromBool(ctx.cpu.sr.z) & @intFromBool(result[1]) == 1;
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
pub fn Logic(comptime mode: enum { @"and", eor }) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
            const result = switch (mode) {
                .@"and" => dst & src,
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

/// Arithmatic shift
pub fn Asd(comptime add_cycles: bool, comptime dir: enum { r, l }) type {
    return struct {
        pub fn op(ctx: *Ctx, comptime size: Size, src: anytype, dst: size.Int()) size.Int() {
            const Full = std.meta.Int(.unsigned, size.bits() + 64);
            const shift: u6 = if (@bitSizeOf(@TypeOf(src)) <= 6) src else @truncate(src);
            const full: Full = switch (dir) {
                .l => @as(Full, dst) << shift,
                .r => @bitCast(int.castsign(.signed, @as(Full, dst) << 64) >> shift),
            };
            const result: size.Int() = @truncate(full >> if (dir == .r) 64 else 0);
            ctx.clk += (@as(usize, shift) * 2 + (size.bits() / 16) * 2) * @intFromBool(add_cycles);

            const last_bit = if (dir == .r) 63 else size.bits();
            ctx.cpu.sr.x = if (shift == 0) ctx.cpu.sr.x else int.extract(bool, full, last_bit);
            ctx.cpu.sr.n = int.negative(result);
            ctx.cpu.sr.z = result == 0;
            ctx.cpu.sr.v = switch (dir) {
                .r => false,
                .l => v: {
                    const shifted = full >> size.bits() - 1;
                    break :v shifted != 0 and shifted != (@as(Full, 2) << shift) - 1;
                },
            };
            ctx.cpu.sr.c = int.extract(bool, full, last_bit);
            return result;
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
        pub fn op(ctx: *Ctx, comptime _: Size, src: u16, dst: u32) u32 {
            // Get the numerator and denominator
            const result_num: std.meta.Int(sign, 32) = @bitCast(dst);
            const result_den: std.meta.Int(sign, 32) = int.castsign(sign, src);
            if (result_den == 0) {
                ctx.clk += 4;
                Ctx.Vector.divzero.handle(ctx);
                return dst;
            }
            ctx.clk += switch (sign) {
                .signed => 12 + @as(usize, @intFromBool(result_den < 0)) * 2,
                .unsigned => 2,
            };

            // Compute the division and check for overflow
            ctx.cpu.sr.c = false;
            ctx.cpu.sr.v = false;
            const result_quo = std.math.cast(i16, @divTrunc(result_num, result_den)) orelse {
                ctx.cpu.sr.v = true;
                return dst;
            };
            const result_rem = std.math.cast(i16, @rem(result_num, result_den)) orelse {
                ctx.cpu.sr.v = true;
                return dst;
            };
            ctx.cpu.sr.n = int.negative(result_quo);
            ctx.cpu.sr.z = result_quo == 0;

            // Calculate the cycle count
            switch (sign) {
                .signed => {
                    ctx.clk += 12;
                    if (result_num < 0) {
                        ctx.clk += 4;
                    } else {
                        ctx.clk += if (result_den < 0) @as(usize, 6) else @as(usize, 2);
                    }

                    var quo: u16 = @bitCast(result_quo);
                    for (0..15) |_| {
                        quo <<= 1;
                        ctx.clk += 6 + @as(usize, ~@as(u1, @truncate(quo >> 15))) * 2;
                    }
                },
                .unsigned => {
                    var num: u32 = @as(u32, @bitCast(result_num));
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
                        if (num >> 16 > result_den) {
                            num -= result_den << 16;
                        }
                    }
                },
            }

            // Return the packed results
            return @as(u32, @as(u16, @bitCast(result_rem))) << 16 | @as(u16, @bitCast(result_quo));
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
