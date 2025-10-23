const std = @import("std");

const bus_interface = @import("bus");
const rxm = @import("rxm");

test "rom" {
    const width = bus_interface.Width{ .addr = 8, .data = 16 };
    const buffer = [_]width.Data(){ 0x0001, 0x0002, 0x0003, 0x0004 };
    var rom = rxm.Rxm(.ro, width).init(&buffer);
    const read = rom.device.read orelse return error.TestFailed;
    try std.testing.expectEqual(null, rom.device.write);

    // Read some data
    try std.testing.expectEqual(0x0001, read(&rom.device, 0x00, 0b11));
    try std.testing.expectEqual(0x0002, read(&rom.device, 0x01, 0b11));
    try std.testing.expectEqual(null, read(&rom.device, 0x04, 0b11));
}

test "ram" {
    const width = bus_interface.Width{ .addr = 8, .data = 16 };
    var buffer = [1]width.Data(){0x0000} ** 128;
    var ram = rxm.Rxm(.rw, width).init(&buffer);
    const read = ram.device.read orelse return error.TestFailed;
    const write = ram.device.write orelse return error.TestFailed;

    // Read and write some bytes
    try std.testing.expectEqual(0x0000, read(&ram.device, 0x00, 0b11));
    try std.testing.expectEqual(null, read(&ram.device, 0x80, 0b11));
    write(&ram.device, 0x00, 0b11, 0xfeed);
    try std.testing.expectEqual(0xfeed, read(&ram.device, 0x00, 0b11));
}
