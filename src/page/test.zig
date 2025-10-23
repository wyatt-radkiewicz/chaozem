const std = @import("std");
const Signedness = std.builtin.Signedness;

const page = @import("page");

test "test small table with only one page entry and positive entry values" {
    const opts = page.Options{
        .Index = u4,
        .Entry = u8,
        .max_pages = 256,
        .page_size = 4,
    };
    const uncompressed_table = [1 << @bitSizeOf(opts.Index)]opts.Entry{
        0, 0, 0, 0,
        1, 1, 1, 1,
        2, 2, 2, 2,
        3, 3, 3, 3,
    };
    const ctx = struct {
        pub fn get(_: @This(), index: opts.Index) opts.Entry {
            return uncompressed_table[index];
        }
    }{};
    const info = page.info(opts, ctx);
    try std.testing.expectEqual(Signedness.unsigned, info.entry_signedness);
    try std.testing.expectEqual(2, info.entry_bits);
    try std.testing.expectEqual(16, info.table_len);
    for (0..uncompressed_table.len) |i| {
        try std.testing.expectEqual(uncompressed_table[i], page.table(opts, ctx)(@intCast(i)));
    }
}

test "test small table with signed entry values" {
    const opts = page.Options{
        .Index = u4,
        .Entry = i4,
        .max_pages = 256,
        .page_size = 2,
    };
    const uncompressed_table = [1 << @bitSizeOf(opts.Index)]opts.Entry{
        -6, -6, -6, -6,
        -4, -4, -4, -4,
        2,  2,  2,  2,
        -4, -4, -4, -4,
    };
    const ctx = struct {
        pub fn get(_: @This(), index: opts.Index) opts.Entry {
            return uncompressed_table[index];
        }
    }{};
    const info = page.info(opts, ctx);
    try std.testing.expectEqual(Signedness.signed, info.entry_signedness);
    try std.testing.expectEqual(4, info.entry_bits);
    try std.testing.expectEqual(16, info.table_len);
    for (0..uncompressed_table.len) |i| {
        try std.testing.expectEqual(uncompressed_table[i], page.table(opts, ctx)(@intCast(i)));
    }
}

test "test small table with optional entry values" {
    const opts = page.Options{
        .Index = u4,
        .Entry = ?u4,
        .max_pages = 256,
        .page_size = 2,
    };
    const uncompressed_table = [1 << @bitSizeOf(opts.Index)]opts.Entry{
        null, null, null, null,
        null, null, null, null,
        2,    2,    2,    2,
        null, null, null, null,
    };
    const ctx = struct {
        pub fn get(_: @This(), index: opts.Index) opts.Entry {
            return uncompressed_table[index];
        }
    }{};
    const info = page.info(opts, ctx);
    try std.testing.expectEqual(Signedness.unsigned, info.entry_signedness);
    try std.testing.expectEqual(2, info.entry_bits);
    try std.testing.expectEqual(12, info.table_len);
    for (0..uncompressed_table.len) |i| {
        try std.testing.expectEqual(uncompressed_table[i], page.table(opts, ctx)(@intCast(i)));
    }
}

test "test small table with unaligned index page size" {
    const opts = page.Options{
        .Index = u5,
        .Entry = u8,
        .max_pages = 256,
        .page_size = 2,
    };
    const uncompressed_table = [1 << @bitSizeOf(opts.Index)]opts.Entry{
        0, 0, 0, 0,
        1, 1, 1, 1,
        2, 2, 2, 2,
        3, 3, 3, 3,
        0, 0, 0, 0,
        1, 1, 1, 1,
        2, 2, 2, 2,
        3, 3, 3, 3,
    };
    const ctx = struct {
        pub fn get(_: @This(), index: opts.Index) opts.Entry {
            return uncompressed_table[index];
        }
    }{};
    const info = page.info(opts, ctx);
    try std.testing.expectEqual(Signedness.unsigned, info.entry_signedness);
    try std.testing.expectEqual(3, info.entry_bits);
    try std.testing.expectEqual(24, info.table_len);
    for (0..uncompressed_table.len) |i| {
        try std.testing.expectEqual(uncompressed_table[i], page.table(opts, ctx)(@intCast(i)));
    }
}
