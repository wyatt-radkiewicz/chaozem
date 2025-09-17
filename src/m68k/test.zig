const std = @import("std");

pub const Bus = @import("Bus.zig");
pub const Cpu = @import("Cpu.zig");
const m68k = @import("m68k.zig");

// A simple test runner for instructions
const Runner = struct {
    cpu: Cpu = .{},
    mem: [1024]u8 = [1]u8{0} ** 1024,
    interface: Bus = .{
        .rdb = struct {
            fn rdb(bus: *Bus, addr: u32) u8 {
                return rd(bus, addr, u8);
            }
        }.rdb,
        .rdw = struct {
            fn rdw(bus: *Bus, addr: u32) u16 {
                return rd(bus, addr, u16);
            }
        }.rdw,
        .rdl = struct {
            fn rdl(bus: *Bus, addr: u32) u32 {
                return rd(bus, addr, u32);
            }
        }.rdl,
        .wrb = struct {
            fn wrb(bus: *Bus, addr: u32, data: u8) void {
                return wr(bus, addr, u8, data);
            }
        }.wrb,
        .wrw = struct {
            fn wrw(bus: *Bus, addr: u32, data: u16) void {
                return wr(bus, addr, u16, data);
            }
        }.wrw,
        .wrl = struct {
            fn wrl(bus: *Bus, addr: u32, data: u32) void {
                return wr(bus, addr, u32, data);
            }
        }.wrl,
    },

    /// Run one instruction
    fn run(this: *@This()) usize {
        return step(&this.cpu, &this.interface);
    }

    /// Disassemble one instruction
    fn disasm(this: *@This(), allocator: std.mem.Allocator) ![]const u8 {
        var reader_buffer = [1]u8{0} ** 8;
        var reader = this.interface.reader(0, &reader_buffer);
        var writer_buffer = std.ArrayList(u8){};
        var writer = writer_buffer.writer(allocator);
        try writer.print("{f}", .{Disasm{ .reader = &reader.interface }});
        return try writer_buffer.toOwnedSlice(allocator);
    }

    /// Reading/writing instructions
    fn rd(bus: *Bus, addr: u32, comptime Data: type) Data {
        const this: *@This() = @fieldParentPtr("interface", bus);
        return std.mem.readInt(Data, this.mem[addr..][0..@sizeOf(Data)], .big);
    }
    fn wr(bus: *Bus, addr: u32, comptime Data: type, data: Data) void {
        const this: *@This() = @fieldParentPtr("interface", bus);
        std.mem.writeInt(Data, this.mem[addr..][0..@sizeOf(Data)], data, .big);
    }
};

// Test data
const Test = struct {
    config: Config,
    expect: Expected,

    /// Cpu status flags
    const Status = struct {
        c: bool = false,
        v: bool = false,
        z: bool = false,
        n: bool = false,
        x: bool = false,
    };

    /// Initial config of the test
    const Config = struct {
        code: []const u16,
        ram: []const u8 = &.{},
        data: []const u32 = &.{},
        addr: []const u32 = &.{},
        status: Status = .{},

        /// Initialize the state of the runner
        fn setup(this: @This(), runner: *Runner) void {
            for (0..this.code.len) |i| {
                std.mem.writeInt(u16, runner.mem[i * 2 ..][0..2], this.code[i], .big);
            }
            @memcpy(runner.mem[0x80..][0..this.ram.len], this.ram);
            @memcpy(runner.cpu.d[0..this.data.len], this.data);
            @memcpy(runner.cpu.a[0..this.addr.len], this.addr);
            runner.cpu.sr.c = this.status.c;
            runner.cpu.sr.v = this.status.v;
            runner.cpu.sr.z = this.status.z;
            runner.cpu.sr.n = this.status.n;
            runner.cpu.sr.x = this.status.x;
        }
    };

    /// Expected state
    const Expected = struct {
        disasm: ?[]const u8 = null,
        clk: ?usize = null,
        ram: ?[]const u8 = null,
        data: ?[]const u32 = null,
        addr: ?[]const u32 = null,
        status: ?Status = null,

        /// Check the state of the runner
        fn check(
            this: @This(),
            runner: *Runner,
            cycles: usize,
        ) !void {
            if (this.clk) |clk| {
                try std.testing.expectEqual(clk, cycles);
            }
            if (this.ram) |ram| {
                try std.testing.expectEqualSlices(u8, ram, runner.mem[0x80..][0..ram.len]);
            }
            if (this.data) |data| {
                try std.testing.expectEqualSlices(u32, data, runner.cpu.d[0..data.len]);
            }
            if (this.addr) |addr| {
                try std.testing.expectEqualSlices(u32, addr, runner.cpu.a[0..addr.len]);
            }
            if (this.status) |status| {
                try std.testing.expectEqual(status.c, runner.cpu.sr.c);
                try std.testing.expectEqual(status.v, runner.cpu.sr.v);
                try std.testing.expectEqual(status.z, runner.cpu.sr.z);
                try std.testing.expectEqual(status.n, runner.cpu.sr.n);
                try std.testing.expectEqual(status.x, runner.cpu.sr.x);
            }
        }
    };
};

test "m68k tests" {
    // Run each test
    var dir = try std.fs.cwd().openDir("tests/m68k", .{ .iterate = true });
    defer dir.close();
    var iter = dir.iterate();
    while (try iter.next()) |entry| {
        if (!std.mem.eql(u8, ".zon", std.fs.path.extension(entry.name))) {
            continue;
        }

        const max_len = 1 * 1024 * 1024;
        const allocator = std.testing.allocator;
        const bytes = try dir.readFileAllocOptions(allocator, entry.name, max_len, null, .@"1", 0);
        defer allocator.free(bytes);

        const tests = try std.zon.parse.fromSlice([]Test, allocator, bytes, null, .{});
        defer std.zon.parse.free(allocator, tests);

        for (tests) |current| {
            var runner = Runner{};
            current.config.setup(&runner);

            var cycles: usize = 0;
            var lines = if (current.expect.disasm) |x| std.mem.splitScalar(u8, x, '\n') else null;
            while (runner.cpu.pc < current.config.code.len * 2) {
                if (lines) |*line_iter| {
                    const actual = try runner.disasm(allocator);
                    defer allocator.free(actual);
                    const expected = line_iter.next() orelse {
                        std.debug.print("expected another instruction in \"{s}\" at \"{s}\"", .{
                            entry.name,
                            current.expect.disasm orelse "<no disasm>",
                        });
                        return error.TestFailed;
                    };
                    std.testing.expectEqualSlices(u8, expected, actual) catch {
                        std.debug.print("disasm failed in \"{s}\" at \"{s}\"", .{
                            entry.name,
                            current.expect.disasm orelse "<no disasm>",
                        });
                        return error.TestFailed;
                    };
                }
                cycles += runner.run();
            }

            current.expect.check(&runner, cycles) catch {
                std.debug.print("test failed in \"{s}\" at \"{s}\"", .{
                    entry.name,
                    current.expect.disasm orelse "<no disasm>",
                });
                return error.TestFailed;
            };
        }
    }
}
