const std = @import("std");

const bus_interface = @import("bus");
const config = @import("config");
const m68k = @import("m68k");

const Bus = bus_interface.Bus(m68k.bus_width);
const Device = bus_interface.Device(m68k.bus_width);
const Mapping = bus_interface.Mapping(m68k.bus_width);

/// A list of tests
const Tests = struct {
    vectors: Vectors = .{},
    tests: []const Test,
};

/// Test data
const Test = struct {
    env: Env,
    expect: Expect,

    /// Run the test
    fn run(this: @This(), group: []const u8, vectors: Vectors, allocator: std.mem.Allocator) !void {
        // Setup the environment to test
        const err_ctx = .{ group, this.expect.disasm };
        var rom = Rom{};
        var ram = Ram{};
        var cpu = m68k.Cpu{};
        this.env.setup(vectors, &cpu, &rom, &ram);

        // Create a bus interface to access the environment
        var bus = Bus.init(&.{
            Mapping{
                .start = 0,
                .size = rom.words.len,
            },
            Mapping{
                .start = rom.words.len,
                .size = ram.words.len,
                .end = std.math.maxInt(m68k.bus_width.Addr()),
            },
        }, &.{ &rom.device, &ram.device });

        // Create a reader to generate a disassembly from the environment
        var reader_buffer = [1]u8{0} ** 8;
        var reader = bus.reader(@truncate(vectors.reset_pc >> 1), &reader_buffer, .big, 1);

        // Run though each instruction and test its disassembly
        var cycles: usize = 0;
        var lines = std.mem.splitScalar(u8, this.expect.disasm, '\n');
        while (cpu.pc < vectors.reset_pc + this.env.code.len * 2) {
            // Print the dissassembler output
            var disasm = std.io.Writer.Allocating.init(allocator);
            try disasm.writer.print("{f}", .{m68k.Disasm{ .reader = &reader.interface }});
            const actual = try disasm.toOwnedSlice();
            defer allocator.free(actual);

            // Get the next line from the diassembly and check that they are the same
            const expected = lines.next() orelse {
                std.log.err("expected another instruction in \"{s}\" at \"{s}\"\n", err_ctx);
                return error.TestFailed;
            };
            std.testing.expectEqualSlices(u8, expected, actual) catch {
                std.log.err("disasm failed in \"{s}\" at \"{s}\"\n", err_ctx);
                return error.TestFailed;
            };

            // Run one instruction
            cycles += m68k.step(&cpu, &bus);
        }

        // Check the final environment
        this.expect.check(vectors, ram, cpu, cycles) catch {
            std.log.err("test failed in \"{s}\" at \"{s}\"\n", err_ctx);
            return error.TestFailed;
        };

        // Check that ROM wasn't accidentally written to
        if (rom.write_addr) |addr| {
            std.log.err(
                \\ test failed in \"{s}\" at \"{s}\"
                \\ wrote 0x{x:0>4} to ROM at 0x{x:0>6}
            , err_ctx ++ .{ addr, rom.write_data });
            return error.TestFailed;
        }
    }
};

/// Various interrupt vectors that can be set at the start of a test suite
const Vectors = struct {
    reset_sp: u32 = 0x2000,
    reset_pc: u32 = 0x100,
    illegal: u32 = 0x200,
    divzero: u32 = 0x200,
    chk: u32 = 0x200,
    trap: u32 = 0x200,
    trapv: u32 = 0x200,
};

/// Cpu status flags
const Flags = struct {
    c: ?bool = null,
    v: ?bool = null,
    z: ?bool = null,
    n: ?bool = null,
    x: ?bool = null,

    /// Initialize the state of a cpu
    fn setup(this: @This(), cpu: *m68k.Cpu) void {
        cpu.sr.c = this.c orelse cpu.sr.c;
        cpu.sr.v = this.v orelse cpu.sr.v;
        cpu.sr.z = this.z orelse cpu.sr.z;
        cpu.sr.n = this.n orelse cpu.sr.n;
        cpu.sr.x = this.x orelse cpu.sr.x;
    }

    /// Check the state of the cpu to see if it matches
    fn check(this: @This(), cpu: m68k.Cpu) !void {
        try std.testing.expectEqual(this.c orelse cpu.sr.c, cpu.sr.c);
        try std.testing.expectEqual(this.v orelse cpu.sr.v, cpu.sr.v);
        try std.testing.expectEqual(this.z orelse cpu.sr.z, cpu.sr.z);
        try std.testing.expectEqual(this.n orelse cpu.sr.n, cpu.sr.n);
        try std.testing.expectEqual(this.x orelse cpu.sr.x, cpu.sr.x);
    }
};

/// Initial config of the test
const Env = struct {
    code: []const u16,
    ram: []const u16 = &.{},
    stack: []const u16 = &.{},
    data: []const u32 = &.{},
    addr: []const u32 = &.{},
    usp: ?u32 = null,
    ssp: ?u32 = null,
    flags: Flags = .{},

    /// Initialize the state of the cpu
    fn setup(this: @This(), vectors: Vectors, cpu: *m68k.Cpu, rom: *Rom, ram: *Ram) void {
        // Inject the vectors into the rom
        inline for (comptime std.meta.fieldNames(m68k.Vector)) |vector| {
            if (@hasField(Vectors, vector)) {
                const addr = @field(m68k.Vector, vector).addr();
                const value = @field(vectors, vector);
                rom.words[addr >> 1] = @truncate(value >> 16);
                rom.words[(addr >> 1) + 1] = @truncate(value);
            }
        }

        // Inject the code into the rom
        const code_start = vectors.reset_pc >> 1;
        @memcpy(rom.words[code_start .. code_start + this.code.len], this.code);

        // Inject data and stack into ram
        @memcpy(ram.words[0..this.ram.len], this.ram);
        @memcpy(ram.words[ram.words.len - this.stack.len ..], this.stack);

        // Setup the special purpose registers
        const stack = vectors.reset_sp - @as(u32, @intCast(this.stack.len * 2));
        cpu.pc = vectors.reset_pc;
        cpu.sp[0] = stack;
        cpu.sp[1] = stack;
        this.flags.setup(cpu);

        // Setup the general purpose registers
        @memcpy(cpu.d[0..this.data.len], this.data);
        @memcpy(cpu.a[0..this.addr.len], this.addr);
    }
};

/// Expect state
const Expect = struct {
    disasm: []const u8,
    clk: ?usize = null,
    usp: ?u32 = null,
    ssp: ?u32 = null,
    pc: ?u32 = null,
    ram: ?[]const u16 = null,
    data: ?[]const u32 = null,
    addr: ?[]const u32 = null,
    stack: ?[]const u16 = null,
    flags: ?Flags = null,
    stop: ?bool = null,

    /// Check the state of the runner
    fn check(this: @This(), vectors: Vectors, ram: Ram, cpu: m68k.Cpu, cycles: usize) !void {
        try std.testing.expectEqual(this.clk orelse cycles, cycles);
        if (this.pc) |pc| {
            try std.testing.expectEqual(vectors.reset_pc + pc, cpu.pc);
        }
        if (this.ram) |words| {
            try std.testing.expectEqualSlices(u16, words, ram.words[0..words.len]);
        }
        if (this.stack) |stack| {
            const start = ram.words.len - stack.len;
            try std.testing.expectEqualSlices(u16, stack, ram.words[start..ram.words.len]);
        }
        if (this.data) |data| {
            try std.testing.expectEqualSlices(u32, data, cpu.d[0..data.len]);
        }
        if (this.addr) |addr| {
            try std.testing.expectEqualSlices(u32, addr, cpu.a[0..addr.len]);
        }
        if (this.usp) |usp| {
            try std.testing.expectEqual(usp, cpu.sp[0]);
        }
        if (this.ssp) |ssp| {
            try std.testing.expectEqual(ssp, cpu.sp[1]);
        }
        if (this.stop) |stop| {
            try std.testing.expectEqual(stop, cpu.stop);
        }
        if (this.flags) |flags| {
            try flags.check(cpu);
        }
    }
};

/// ROM chip implementation for m68k
const Rom = struct {
    words: [0x1000 >> 1]u16 = [1]u16{0} ** (0x1000 >> 1),
    device: Device = .{ .read = read },
    write_addr: ?u24 = null,
    write_data: u16 = 0,

    /// Get data from ROM
    pub fn read(dev: *Device, addr: u23, _: u2) ?u16 {
        const this: *@This() = @fieldParentPtr("device", dev);
        return this.words[addr];
    }

    /// Write data to ROM, (this does nothing but log the write)
    pub fn write(dev: *Device, addr: u23, _: u2, data: u16) void {
        const this: *@This() = @fieldParentPtr("device", dev);
        this.write_addr = @as(u24, addr) << 1;
        this.write_data = data;
    }
};

/// RAM chip implementation for m68k
const Ram = struct {
    words: [0x1000 >> 1]u16 = [1]u16{0} ** (0x1000 >> 1),
    device: Device = .{ .read = read, .write = write },

    /// Get data from RAM
    pub fn read(dev: *Device, addr: u23, _: u2) ?u16 {
        const this: *@This() = @fieldParentPtr("device", dev);
        return this.words[addr];
    }

    /// Set data in RAM
    pub fn write(dev: *Device, addr: u23, bytes: u2, data: u16) void {
        const this: *@This() = @fieldParentPtr("device", dev);
        const mask = [4]u16{ 0xFFFF, 0xFF00, 0x00FF, 0x0000 };
        this.words[addr] = this.words[addr] & mask[bytes] | data;
    }
};

test "m68k tests" {
    // Run each test
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
        const tests = std.zon.parse.fromSlice(Tests, allocator, bytes, null, .{}) catch {
            std.log.err("Failed to parse \"{s}\" m68k unit test file\n", .{entry.name});
            return error.ParsingFailed;
        };
        defer std.zon.parse.free(allocator, tests);

        // Run each test in the file
        for (tests.tests) |@"test"| {
            try @"test".run(entry.name, tests.vectors, allocator);
        }
    }
}
