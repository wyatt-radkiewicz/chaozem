//! Integer helper functions
const std = @import("std");

/// Extract a type from an integer at a position
pub inline fn extract(
    comptime Type: type,
    from: anytype,
    at: std.math.Log2Int(@TypeOf(from)),
) Type {
    const TypeInt = std.meta.Int(.unsigned, @bitSizeOf(Type));
    const FromInt = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(from)));
    return @bitCast(@as(TypeInt, @truncate(@as(FromInt, @bitCast(from)) >> at)));
}

/// Sign extend a specified integer to the specified size
pub inline fn extend(comptime To: type, from: anytype) To {
    const ToSigned = std.meta.Int(.signed, @bitSizeOf(To));
    const FromSigned = std.meta.Int(.signed, @bitSizeOf(@TypeOf(from)));
    return @bitCast(@as(ToSigned, @as(FromSigned, @bitCast(from))));
}

/// Overwrite the lower N bits with another integer
pub inline fn overwrite(int: anytype, with: anytype) @TypeOf(int) {
    const Int = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(int)));
    const mask: Int = (1 << @bitSizeOf(@TypeOf(with))) - 1;
    return int & ~mask | with;
}

/// Returns true if the int has the highest bit set
pub inline fn negative(int: anytype) bool {
    return @as(std.meta.Int(.signed, @bitSizeOf(@TypeOf(int))), @bitCast(int)) < 0;
}
