const std = @import("std");

const bus_interface = @import("bus");
const m68k = @import("m68k");
const test_host = @import("host");
const test_input: []const test_host.Input = @import("input");

const test_rom = @embedFile("target_rom");
const test_ram = @embedFile("target_ram");

const Bus = bus_interface.Bus(m68k.bus_width);
const Device = bus_interface.Device(m68k.bus_width);
const Mapping = bus_interface.Mapping(m68k.bus_width);

/// Ram device (allows reading and writing)
const Ram = struct {
    bytes: [0x10000]u8 = test_ram.* ++ ([1]u8{0} ** (0x10000 - test_ram.len)),
    device: Device = .{
        .read = struct {
            pub fn read(device: *Device, addr: u23, _: u2) ?u16 {
                const ram: *Ram = @fieldParentPtr("device", device);
                return std.mem.readInt(u16, ram.bytes[addr * 2 ..][0..2], .big);
            }
        }.read,
        .write = struct {
            pub fn write(device: *Device, addr: u23, mask: u2, data: u16) void {
                const ram: *Ram = @fieldParentPtr("device", device);
                switch (mask) {
                    0b00 => {},
                    0b10 => ram.bytes[addr * 2 + 0] = @truncate(data >> 8),
                    0b01 => ram.bytes[addr * 2 + 1] = @truncate(data),
                    0b11 => std.mem.writeInt(u16, ram.bytes[addr * 2 ..][0..2], data, .big),
                }
            }
        }.write,
    },
};

test "m68k integration test" {
    // Run every test input
    for (test_input, 0..) |input, test_case| {
        // Run native version first to get the expected output of the test
        const expected = expected: {
            var output: test_host.Output = undefined;
            test_host.run(&input, &output);
            break :expected output;
        };

        // Create the rom and ram interface and the bus interface
        var rom = Device{
            .read = struct {
                pub fn read(_: *Device, addr: u23, _: u2) ?u16 {
                    if (addr < test_rom.len / 2) {
                        return std.mem.readInt(u16, test_rom[addr * 2 ..][0..2], .big);
                    } else {
                        return 0;
                    }
                }
            }.read,
        };
        var ram = Ram{};
        var bus = Bus.init(&.{
            Mapping{
                .start = 0,
                .size = 0x10000 >> 1,
            },
            Mapping{
                .start = 0xff0000 >> 1,
                .size = 0x10000 >> 1,
            },
        }, &.{ &rom, &ram.device });
        var cpu = m68k.Cpu{};

        // Start writing the things we need to ram
        const writer_base: u32 = 0xffff0000 + @as(u32, @intCast(test_ram.len));
        var ram_writer = std.io.Writer.fixed(ram.bytes[test_ram.len..]);

        // Write to ram the input data
        const input_addr = writer_base + @as(u32, @intCast(ram_writer.end));
        try ram_writer.writeStruct(input, .big);

        // Write to ram the output data
        const output_addr = writer_base + @as(u32, @intCast(ram_writer.end));
        _ = try ram_writer.writeSplat(
            &.{&.{0xA5}},
            @divFloor(@sizeOf(test_host.Output) + 1, 2) * 2,
        );

        // Write a STOP instruction to ram
        const stop_addr = writer_base + @as(u32, @intCast(ram_writer.end));
        try ram_writer.writeInt(u16, 0x4E72, .big);

        // Push paramters onto stack in reverse order
        try ram_writer.flush();
        ram_writer = std.io.Writer.fixed(ram.bytes[ram.bytes.len - 4 - 4 - 4 ..]);

        // Push return addres, then input, then output and set stack pointer
        try ram_writer.writeInt(u32, stop_addr, .big);
        try ram_writer.writeInt(u32, input_addr, .big);
        try ram_writer.writeInt(u32, output_addr, .big);
        try ram_writer.flush();

        // Call the run function and run until we've stopped
        cpu.r(.a, 7).* = std.mem.readInt(u32, test_rom[0..4], .big) -% (4 * 3);
        cpu.pc = std.mem.readInt(u32, test_rom[4..8], .big);
        var timeout: usize = 0;
        while (!cpu.stop and timeout < 10000 * 20) {
            timeout += m68k.step(&cpu, &bus);
        }

        // Check the outputs
        var ram_reader = std.io.Reader.fixed(ram.bytes[output_addr - 0xffff0000 ..]);
        const actual = ram_reader.takeStruct(test_host.Output, .big);
        std.testing.expectEqual(expected, actual) catch {
            std.log.err("Test case {} failed.", .{test_case});
            return error.TestFailed;
        };
    }
}
