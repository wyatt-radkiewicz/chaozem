const std = @import("std");

const int = @import("root.zig");

test "extract" {
    try std.testing.expectEqual(@as(u4, 0x3), int.extract(u4, @as(u16, 0x3210), 12));
}

test "extend" {
    try std.testing.expectEqual(@as(u8, 0xf8), int.extend(u8, @as(u4, 0x8)));
    try std.testing.expectEqual(@as(u8, 0x07), int.extend(u8, @as(u4, 0x7)));
    try std.testing.expectEqual(@as(u8, 0xf8), int.extend(u8, @as(i4, -8)));
    try std.testing.expectEqual(@as(i8, 7), int.extend(i8, @as(i4, 7)));
}

test "overwrite" {
    try std.testing.expectEqual(@as(u8, 0xf0), int.overwrite(@as(u8, 0xff), @as(u4, 0x0)));
    try std.testing.expectEqual(@as(u8, 0x00), int.overwrite(@as(u8, 0xff), @as(u8, 0x00)));
}

test "negative" {
    try std.testing.expectEqual(true, int.negative(@as(u4, 0x8)));
    try std.testing.expectEqual(false, int.negative(@as(u4, 0x4)));
}
