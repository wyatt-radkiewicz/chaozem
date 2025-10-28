const std = @import("std");

const bus_interface = @import("bus");
const rxm = @import("rxm");

test "rom" {
    const width = bus_interface.Width{ .addr = 8, .data = 16 };
    const Device = bus_interface.Device(width);
    const buffer = [_]width.Data(){ 0x0001, 0x0002, 0x0003, 0x0004 };
    var rom = rxm.Rxm(.ro, width).init(&buffer);
    try std.testing.expect(rom.device.read != Device.default_read);
    try std.testing.expect(rom.device.write == Device.default_write);

    // Read some data
    try std.testing.expectEqual(0x0001, rom.device.read(&rom.device, 0x00, 0b11));
    try std.testing.expectEqual(0x0002, rom.device.read(&rom.device, 0x01, 0b11));
    try std.testing.expectEqual(0x0000, rom.device.read(&rom.device, 0x04, 0b11));
}

test "ram" {
    const width = bus_interface.Width{ .addr = 8, .data = 16 };
    const Device = bus_interface.Device(width);
    var buffer = [1]width.Data(){0x0000} ** 128;
    var ram = rxm.Rxm(.rw, width).init(&buffer);
    try std.testing.expect(ram.device.read != Device.default_read);
    try std.testing.expect(ram.device.write != Device.default_write);

    // Read and write some bytes
    try std.testing.expectEqual(0x0000, ram.device.read(&ram.device, 0x00, 0b11));
    try std.testing.expectEqual(0x0000, ram.device.read(&ram.device, 0xFF, 0b11));
    ram.device.write(&ram.device, 0x00, 0b11, 0xfeed);
    try std.testing.expectEqual(0xfeed, ram.device.read(&ram.device, 0x00, 0b11));
}
