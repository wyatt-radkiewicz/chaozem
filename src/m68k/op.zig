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
        const sum = src +% dst +% @intFromBool(ctx.cpu.sr.x);
        const overflow = @addWithOverflow(
            @as(size.Int(.signed), @bitCast(src)),
            @as(size.Int(.signed), @bitCast(dst +% 1)),
        )[1] | @addWithOverflow(
            @as(size.Int(.signed), @bitCast(dst)),
            @as(size.Int(.signed), 1),
        )[1] == 1;
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
pub const Asl = struct {
    pub fn op(
        ctx: *Ctx,
        comptime size: Ctx.Size,
        shift_amt: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const shift: std.math.Log2Int(@TypeOf(dst)) = @truncate(@min(shift_amt, size.bits()));
        const result = dst << shift;
        const last_bit = @as(u1, @truncate(result >> size.bits() - 1 -| shift)) == 1;
        const ovf_mask = (@as(size.Int(.unsigned), 1) << shift) - 1;
        const chopped_bits = result >> @truncate(size.bits() - @as(u32, shift));
        ctx.clk += (shift_amt -| 1) * 2;
        ctx.cpu.sr.x = if (shift == 0) ctx.cpu.sr.x else last_bit;
        ctx.cpu.sr.n = int.negative(result);
        ctx.cpu.sr.z = result == 0;
        ctx.cpu.sr.v = chopped_bits == ovf_mask or chopped_bits == 0;
        ctx.cpu.sr.c = last_bit;
        return result;
    }
};

/// Normal or operation
pub const Or = struct {
    pub fn op(
        ctx: *Ctx,
        comptime size: Ctx.Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const result = src | dst;
        ctx.cpu.sr.n = int.negative(result);
        ctx.cpu.sr.z = result == 0;
        ctx.cpu.sr.v = false;
        ctx.cpu.sr.c = false;
        return result;
    }
};
