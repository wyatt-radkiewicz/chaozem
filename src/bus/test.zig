const std = @import("std");

const bus = @import("root.zig");

test "16 bit address and 8 bit data bus" {
    const width = bus.Width{
        .addr = 16,
        .data = 8,
    };
    var device_a = bus.Device(width){
        .read = struct {
            pub fn read(_: *bus.Device(width), addr: u16, _: u8) ?u8 {
                return @truncate(addr);
            }
        }.read,
        .write = null,
    };
    const mapping_a = bus.Mapping(width){
        .start = 0x80,
        .size = 0x80,
        .end = 0x017F,
    };
    var device_b = bus.Device(width){
        .read = struct {
            pub fn read(_: *bus.Device(width), _: u16, mask: u8) ?u8 {
                return mask;
            }
        }.read,
        .write = null,
    };
    const mapping_b = bus.Mapping(width){
        .start = 0x200,
        .size = 0x10,
    };
    const network = bus.Bus(width).init(&.{ mapping_a, mapping_b }, &.{ &device_a, &device_b });
    try std.testing.expectEqual(null, network.read(0x0000, 0x00));
    try std.testing.expectEqual(0x00, network.read(0x0080, 0x00));
    try std.testing.expectEqual(0x01, network.read(0x0081, 0x00));
    try std.testing.expectEqual(0x00, network.read(0x0100, 0x00));
    try std.testing.expectEqual(0x01, network.read(0x0101, 0x00));
    try std.testing.expectEqual(null, network.read(0x0180, 0x00));
    try std.testing.expectEqual(0xAA, network.read(0x0200, 0xAA));
    try std.testing.expectEqual(0xAA, network.read(0x020F, 0xAA));
    try std.testing.expectEqual(null, network.read(0x0210, 0xAA));
}

test "bus reader" {
    const width = bus.Width{
        .addr = 4,
        .data = 16,
    };
    var device_a = bus.Device(width){
        .read = struct {
            pub fn read(_: *bus.Device(width), addr: u4, _: u16) ?u16 {
                const bytes = "hello_world_____";
                return std.mem.readInt(u16, bytes[addr << 1 ..][0..2], .little);
            }
        }.read,
        .write = null,
    };
    const mapping_a = bus.Mapping(width){
        .start = 0,
        .size = 8,
        .end = 15,
    };
    const network = bus.Bus(width).init(&.{mapping_a}, &.{&device_a});
    var buffer = [1]u8{0} ** 12;
    var reader = network.reader(0, &buffer, .little, 1);

    for (0..2) |_| {
        try std.testing.expectEqualSlices(u8, "hello_world", try reader.interface.take(11));
        try std.testing.expectEqual(5, try reader.interface.discard(.limited(5)));
    }
}
