const std = @import("std");

const bus_interface = @import("bus");
const config = @import("config");
const m68k = @import("m68k");
const Test = @import("Test");

const Bus = bus_interface.Bus(m68k.bus_width);
const Device = bus_interface.Device(m68k.bus_width);
const Mapping = bus_interface.Mapping(m68k.bus_width);

/// Rom device (only allows reading)
const Rom = struct {
    bytes: []const u8,
    device: Device = .{ .read = read },

    /// Read from the device
    pub fn read(device: *Device, addr: u23, _: u2) ?u16 {
        const rom: *Rom = @fieldParentPtr("device", device);
        if (addr < rom.bytes.len / 2) {
            return std.mem.readInt(u16, rom.bytes[addr * 2 ..][0..2], .big);
        } else {
            return 0;
        }
    }
};

/// Ram device (allows reading and writing)
const Ram = struct {
    bytes: [0x10000]u8 = [1]u8{0} ** 0x10000,
    device: Device = .{ .read = read, .write = write },

    /// Initialize ram with test info
    fn init(tests: Test, case: Test.Case) @This() {
        var this = @This(){};
        @memcpy(this.bytes[0..tests.ram.len], tests.ram);
        @memcpy(this.bytes[this.bytes.len - tests.stack.len ..], tests.stack);
        @memcpy(this.bytes[tests.setup_base..][0..case.setup.len], case.setup);
        return this;
    }

    /// Read data
    pub fn read(device: *Device, addr: u23, _: u2) ?u16 {
        const ram: *Ram = @fieldParentPtr("device", device);
        return std.mem.readInt(u16, ram.bytes[addr * 2 ..][0..2], .big);
    }

    /// Write data
    pub fn write(device: *Device, addr: u23, mask: u2, data: u16) void {
        const ram: *Ram = @fieldParentPtr("device", device);
        switch (mask) {
            0b00 => {},
            0b10 => ram.bytes[addr * 2 + 0] = @truncate(data >> 8),
            0b01 => ram.bytes[addr * 2 + 1] = @truncate(data),
            0b11 => std.mem.writeInt(u16, ram.bytes[addr * 2 ..][0..2], data, .big),
        }
    }

    /// Print debug info
    const Debug = struct {
        ram: *const Ram,
        base: usize,
        to: usize,

        pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
            var byte: u32 = 0;
            while (byte < this.to) {
                try writer.print("0x{X:0>8}: ", .{byte + this.base});
                for (0..8) |i| {
                    try writer.print(
                        " {X:0>4}",
                        .{std.mem.readInt(u16, this.ram.bytes[byte + i * 2 ..][0..2], .big)},
                    );
                    byte += 2;
                }
                try writer.print("\n", .{});
            }
        }
    };
};

// Prints a instruction at the address
fn instr(bus: Bus, addr: u32) !void {
    var buffer: [8]u8 = undefined;
    var reader = bus.reader(@truncate(addr >> 1), &buffer, .big, 1);
    std.debug.print("0x{X:0>8}: {f}\n", .{ addr, m68k.Disasm{
        .reader = &reader.interface,
    } });
}

test "m68k integration test" {
    // Check environment for 'M68K_INTEGRATION_DISASM' to print out disassembly
    const do_disasm = std.process.parseEnvVarInt("M68K_INTEGRATION_DISASM", i8, 10) catch 0 != 0;

    // Check arguments for 'M68K_INTEGRATION_DUMP' to dump cpu info on every instruction
    const do_dump = std.process.parseEnvVarInt("M68K_INTEGRATION_DUMP", i8, 10) catch 0 != 0;

    // Find and run every test in the current directory
    var dir = try std.fs.cwd().openDir(config.tests_path, .{ .iterate = true });
    defer dir.close();
    var iter = dir.iterate();
    while (try iter.next()) |entry| {
        // Make sure we are testing a test file
        if (!std.mem.eql(u8, ".zon", std.fs.path.extension(entry.name))) {
            continue;
        }

        // Read in the file
        const max_len = 4 * 1024 * 1024;
        const allocator = std.testing.allocator;
        const bytes = try dir.readFileAllocOptions(allocator, entry.name, max_len, null, .@"1", 0);
        defer allocator.free(bytes);

        // Parse the file
        const tests = std.zon.parse.fromSlice(Test, allocator, bytes, null, .{}) catch {
            std.log.err("Failed to parse \"{s}\" m68k integration test file\n", .{entry.name});
            return error.ParsingFailed;
        };
        defer std.zon.parse.free(allocator, tests);

        // Run each test case in the file
        for (tests.cases, 0..) |case, i| {
            // Create the rom and ram interface and the bus interface
            var rom = Rom{ .bytes = tests.rom };
            var ram = Ram.init(tests, case);
            var bus = Bus.init(&.{
                Mapping{
                    .start = 0,
                    .size = 0x10000 >> 1,
                },
                Mapping{
                    .start = 0xff0000 >> 1,
                    .size = 0x10000 >> 1,
                },
            }, &.{ &rom.device, &ram.device });
            var cpu = m68k.Cpu{};

            // Call the run function and run until we've stopped
            cpu.r(.a, 7).* = std.mem.readInt(u32, tests.rom[0..4], .big) -% (4 * 3);
            cpu.pc = std.mem.readInt(u32, tests.rom[4..8], .big);
            var timeout: usize = 0;
            while (!cpu.stop) {
                if (do_dump) {
                    std.debug.print(
                        "{f}\n{f}\n",
                        .{ cpu, Ram.Debug{ .ram = &ram, .base = 0xffff0000, .to = tests.ram_end } },
                    );
                }
                if (do_disasm) {
                    try instr(bus, cpu.pc);
                }

                // Check if we timedout
                timeout += m68k.step(&cpu, &bus);
                if (timeout >= 10000 * 20) {
                    std.log.err("Test case {} timed out in {s}.", .{ i, entry.name });
                    return error.TestFailed;
                }
            }

            // Dump processor status one last time and also post a dump of the ram
            if (do_dump) {
                std.debug.print(
                    "{f}\n{f}\n",
                    .{ cpu, Ram.Debug{ .ram = &ram, .base = 0xffff0000, .to = tests.ram_end } },
                );
            }

            // Check the outputs
            std.testing.expectEqualSlices(
                u8,
                case.expect,
                ram.bytes[tests.expect_base..][0..case.expect.len],
            ) catch {
                std.log.err("Test case {} failed in {s}.", .{ i, entry.name });
                return error.TestFailed;
            };
        }
    }
}
