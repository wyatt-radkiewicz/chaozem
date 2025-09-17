const std = @import("std");

const Exec = @import("Exec.zig");

/// Binary decimal add operation
const Abcd = struct {
    fn op(exec: *Exec, comptime _: Exec.Size, src: u8, dst: u8) u8 {
        const result = Exec.tobcd(Exec.frombcd(src) + Exec.frombcd(dst) +
            @intFromBool(exec.cpu.sr.x));
        exec.cpu.sr.x = result[1];
        exec.cpu.sr.c = result[1];
        exec.cpu.sr.z = @intFromBool(exec.cpu.sr.z) & @intFromBool(result[1]) == 1;
        return result[0];
    }
};

/// Normal addition operation
const Add = struct {
    fn op(
        exec: *Exec,
        comptime size: Exec.Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const sum = src +% dst;
        const overflow = @addWithOverflow(
            @as(size.Int(.signed), @bitCast(src)),
            @as(size.Int(.signed), @bitCast(dst)),
        )[1] == 1;
        const carry = @addWithOverflow(src, dst)[1] == 1;
        exec.cpu.sr.x = carry;
        exec.cpu.sr.n = Exec.negative(sum);
        exec.cpu.sr.z = sum == 0;
        exec.cpu.sr.v = overflow;
        exec.cpu.sr.c = carry;
        return sum;
    }
};

/// Address addition operation
const Adda = struct {
    fn op(_: *Exec, comptime size: Exec.Size, src: size.Int(.unsigned), dst: u32) u32 {
        return Exec.extend(u32, src) +% dst;
    }
};

/// Extended addition operation
const Addx = struct {
    fn op(
        exec: *Exec,
        comptime size: Exec.Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const sum = src +% dst +% @intFromBool(exec.cpu.sr.x);
        const overflow = @addWithOverflow(
            @as(size.Int(.signed), @bitCast(src)),
            @as(size.Int(.signed), @bitCast(dst +% 1)),
        )[1] | @addWithOverflow(
            @as(size.Int(.signed), @bitCast(dst)),
            @as(size.Int(.signed), 1),
        )[1] == 1;
        const carry = @addWithOverflow(src, dst)[1] | @addWithOverflow(dst, 1)[1] == 1;
        exec.cpu.sr.x = carry;
        exec.cpu.sr.n = Exec.negative(sum);
        exec.cpu.sr.z = @intFromBool(exec.cpu.sr.z) & @intFromBool(sum == 0) == 1;
        exec.cpu.sr.v = overflow;
        exec.cpu.sr.c = carry;
        return sum;
    }
};

/// Bitwise and
const And = struct {
    fn op(
        exec: *Exec,
        comptime size: Exec.Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const result = src & dst;
        exec.cpu.sr.n = Exec.negative(result);
        exec.cpu.sr.z = result == 0;
        return result;
    }
};

/// Arithmatic shift left
const Asl = struct {
    fn op(
        exec: *Exec,
        comptime size: Exec.Size,
        shift_amt: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const shift: std.math.Log2Int(@TypeOf(dst)) = @truncate(@min(shift_amt, size.bitSize()));
        const result = dst << shift;
        const last_bit = @as(u1, @truncate(result >> size.bitSize() - 1 -| shift)) == 1;
        const ovf_mask = (@as(size.Int(.unsigned), 1) << shift) - 1;
        const chopped_bits = result >> @truncate(size.bitSize() - @as(u32, shift));
        exec.clk += (shift_amt -| 1) * 2;
        exec.cpu.sr.x = if (shift == 0) exec.cpu.sr.x else last_bit;
        exec.cpu.sr.n = Exec.negative(result);
        exec.cpu.sr.z = result == 0;
        exec.cpu.sr.v = chopped_bits == ovf_mask or chopped_bits == 0;
        exec.cpu.sr.c = last_bit;
        return result;
    }
};

/// Normal or operation
const Or = struct {
    fn op(
        exec: *Exec,
        comptime size: Exec.Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const result = src | dst;
        exec.cpu.sr.n = Exec.negative(result);
        exec.cpu.sr.z = result == 0;
        exec.cpu.sr.v = false;
        exec.cpu.sr.c = false;
        return result;
    }
};
