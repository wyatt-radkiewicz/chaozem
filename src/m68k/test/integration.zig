const std = @import("std");

const bus_interface = @import("bus");
const test_data: []const Test = @import("test_data");
const test_host = @import("test_host");

const Cpu = @import("Cpu.zig");
const m68k = @import("root.zig");

const test_target = @embedFile("test_target");
const Bus = bus_interface.Bus(m68k.bus_width);
const Device = bus_interface.Device(m68k.bus_width);
const Mapping = bus_interface.Mapping(m68k.bus_width);

const Test = struct {
    input: test_host.Input,
    output: test_host.Output,
};

test "m68k integration tester" {
    // Rom device
    const rom = Device{
        .read = struct {
            pub fn read(_: *Device, addr: u23, _: u2) u16 {
                return if (addr < test_target.len / 2)
                    std.mem.readInt(u16, test_target[addr << 1 ..][0..2], .big)
                else
                    0;
            }
        }.read,
    };
    _ = rom; // autofix

    //
}
