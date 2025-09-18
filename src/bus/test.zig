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
