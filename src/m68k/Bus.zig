/// M68K bus interface
const std = @import("std");

/// Read byte/word/long
rdb: *const fn (bus: *@This(), addr: u32) u8,
rdw: *const fn (bus: *@This(), addr: u32) u16,
rdl: *const fn (bus: *@This(), addr: u32) u32,

/// Write byte/word/long
wrb: *const fn (bus: *@This(), addr: u32, data: u8) void,
wrw: *const fn (bus: *@This(), addr: u32, data: u16) void,
wrl: *const fn (bus: *@This(), addr: u32, data: u32) void,

/// Create a reader at a certain position
pub fn reader(this: *@This(), addr: u32, buffer: []u8) Reader {
    return .{
        .bus = this,
        .addr = addr,
        .interface = .{
            .vtable = &.{ .stream = Reader.stream },
            .buffer = buffer,
            .seek = 0,
            .end = 0,
        },
    };
}

/// Interface to read
pub const Reader = struct {
    bus: *Bus,
    addr: u32,
    interface: std.io.Reader,

    /// Standard reader interface function
    pub fn stream(
        io_reader: *std.Io.Reader,
        w: *std.Io.Writer,
        limit: std.Io.Limit,
    ) std.Io.Reader.StreamError!usize {
        const this: *@This() = @alignCast(@fieldParentPtr("interface", io_reader));
        const dest = limit.slice(try w.writableSliceGreedy(1));
        for (dest, this.addr..) |*dest_byte, addr| {
            dest_byte.* = this.bus.rdb(this.bus, @truncate(addr));
        }
        this.addr += 1;
        return dest.len;
    }
};
