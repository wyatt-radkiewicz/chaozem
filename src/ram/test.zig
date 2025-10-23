const std = @import("std");

const bus_interface = @import("bus");
const ram_device = @import("ram");

test "ram" {
    const width = bus_interface.Width{ .addr = 8, .data = 16 };
    var buffer = [1]width.Data(){0x0000} ** 128;
    var ram = ram_device.Ram(width).init(&buffer);
    const read = ram.device.read orelse return error.TestFailed;
    const write = ram.device.write orelse return error.TestFailed;

    // Read and write some bytes
    try std.testing.expectEqual(0x0000, read(&ram.device, 0x00, 0b11));
    try std.testing.expectEqual(null, read(&ram.device, 0x80, 0b11));
    write(&ram.device, 0x00, 0b11, 0xfeed);
    try std.testing.expectEqual(0xfeed, read(&ram.device, 0x00, 0b11));
}
