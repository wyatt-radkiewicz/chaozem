const std = @import("std");

const int = @import("int");

const Ctx = @import("Ctx.zig");
const Size = Ctx.Size;

/// Binary decimal add operation
pub const Abcd = struct {
    pub fn op(ctx: *Ctx, comptime _: Size, src: u8, dst: u8) u8 {
        const result = Ctx.tobcd(Ctx.frombcd(src) + Ctx.frombcd(dst) +
            @intFromBool(ctx.cpu.sr.x));
        ctx.cpu.sr.x = result[1];
        ctx.cpu.sr.c = result[1];
        ctx.cpu.sr.z = @intFromBool(ctx.cpu.sr.z) & @intFromBool(result[1]) == 1;
        return result[0];
    }
};

/// Normal addition operation
pub const Add = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
        const sum = src +% dst;
        const overflow = @addWithOverflow(
            int.castsign(.signed, src),
            int.castsign(.signed, dst),
        )[1] == 1;
        const carry = @addWithOverflow(src, dst)[1] == 1;

        ctx.cpu.sr.x = carry;
        ctx.cpu.sr.n = int.negative(sum);
        ctx.cpu.sr.z = sum == 0;
        ctx.cpu.sr.v = overflow;
        ctx.cpu.sr.c = carry;
        return sum;
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
        const sum = src +% dst +% @intFromBool(ctx.cpu.sr.x);
        const overflow = @addWithOverflow(
            int.castsign(.signed, src),
            int.castsign(.signed, dst +% 1),
        )[1] | @addWithOverflow(int.castsign(.signed, dst), 1)[1] == 1;
        const carry = @addWithOverflow(src, dst)[1] | @addWithOverflow(dst, 1)[1] == 1;

        ctx.cpu.sr.x = carry;
        ctx.cpu.sr.n = int.negative(sum);
        ctx.cpu.sr.z = @intFromBool(ctx.cpu.sr.z) & @intFromBool(sum == 0) == 1;
        ctx.cpu.sr.v = overflow;
        ctx.cpu.sr.c = carry;
        return sum;
    }
};

/// Bitwise and
pub const And = struct {
    pub fn op(ctx: *Ctx, comptime size: Size, src: size.Int(), dst: size.Int()) size.Int() {
        const result = src & dst;
        ctx.cpu.sr.n = int.negative(result);
        ctx.cpu.sr.z = result == 0;
        return result;
    }
};

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
