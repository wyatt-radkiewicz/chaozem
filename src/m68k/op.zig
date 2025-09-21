const std = @import("std");

const int = @import("int");

const Ctx = @import("Ctx.zig");

/// Binary decimal add operation
pub const Abcd = struct {
    pub fn op(ctx: *Ctx, comptime _: Ctx.Size, src: u8, dst: u8) u8 {
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
    pub fn op(
        ctx: *Ctx,
        comptime size: Ctx.Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const sum = src +% dst;
        const overflow = @addWithOverflow(
            @as(size.Int(.signed), @bitCast(src)),
            @as(size.Int(.signed), @bitCast(dst)),
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
    pub fn op(_: *Ctx, comptime size: Ctx.Size, src: size.Int(.unsigned), dst: u32) u32 {
        return int.extend(u32, src) +% dst;
    }
};

/// Extended addition operation
pub const Addx = struct {
    pub fn op(
        ctx: *Ctx,
        comptime size: Ctx.Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const Signed = size.Int(.signed);

        const sum = src +% dst +% @intFromBool(ctx.cpu.sr.x);
        const overflow = @addWithOverflow(
            @as(Signed, @bitCast(src)),
            @as(Signed, @bitCast(dst +% 1)),
        )[1] | @addWithOverflow(@as(Signed, @bitCast(dst)), @as(Signed, 1))[1] == 1;
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
    pub fn op(
        ctx: *Ctx,
        comptime size: Ctx.Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const result = src & dst;
        ctx.cpu.sr.n = int.negative(result);
        ctx.cpu.sr.z = result == 0;
        return result;
    }
};

/// Arithmatic shift left
pub fn Asl(comptime add_cycles: bool) type {
    return struct {
        pub fn op(
            ctx: *Ctx,
            comptime size: Ctx.Size,
            src: anytype,
            dst: size.Int(.unsigned),
        ) size.Int(.unsigned) {
            const Full = std.meta.Int(.unsigned, size.bits() + 64);
            const shift: u6 = if (@bitSizeOf(@TypeOf(src)) <= 6) src else @truncate(src);
            const full = @as(Full, dst) << shift;
            const result: size.Int(.unsigned) = @truncate(full);
            ctx.clk += (@as(usize, shift) * 2 + (size.bits() / 16) * 2) * @intFromBool(add_cycles);

            ctx.cpu.sr.x = if (shift == 0) ctx.cpu.sr.x else int.extract(bool, full, size.bits());
            ctx.cpu.sr.n = int.negative(result);
            ctx.cpu.sr.z = result == 0;
            ctx.cpu.sr.v = v: {
                const shifted = full >> size.bits() - 1;
                break :v shifted != 0 and shifted != (@as(Full, 2) << shift) - 1;
            };
            ctx.cpu.sr.c = int.extract(bool, full, size.bits());
            return result;
        }
    };
}
