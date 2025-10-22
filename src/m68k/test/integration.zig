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
            return null;
        }
    }
};

/// Ram device (allows reading and writing)
const Ram = struct {
    bytes: [0x10000]u8 = [1]u8{0} ** 0x10000,
    device: Device = .{ .read = read, .write = write },

    /// Initialize ram with test info
    fn init(tests: Test) @This() {
        var this = @This(){};
        @memcpy(this.bytes[0..tests.ram.len], tests.ram);
        return this;
    }

    /// Load the input for case data
    fn load(this: *@This(), tests: Test, case: Test.Case) void {
        @memcpy(this.bytes[this.bytes.len - tests.stack.len ..], tests.stack);
        @memcpy(this.bytes[tests.setup_base..][0..case.setup.len], case.setup);
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
};

/// A piece of the formatting string
const Token = union(enum) {
    literal: []const u8,
    pc: void,
    i: void,
    d: u3,
    a: u3,
    sp: u1,
    sr: ?std.meta.FieldEnum(m68k.Cpu.Status),
    stop: void,
    b: struct { u23, u23 },
    st: ?usize,

    /// How long the disassembly for an instruction can be
    const max_disasm_len = 64;

    /// Format the output for state debugging
    const State = struct {
        tok: Token,
        cpu: *const m68k.Cpu,
        bus: *const Bus,

        /// Format the output
        pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
            switch (this.tok) {
                .literal => |str| try writer.print("{s}", .{str}),
                .pc => try writer.print("0x{x:0>8}", .{this.cpu.pc}),
                .i => {
                    var reader_buf: [8]u8 = undefined;
                    var reader = this.bus.reader(@truncate(this.cpu.pc >> 1), &reader_buf, .big, 1);
                    var disasm_buf: [max_disasm_len]u8 = undefined;
                    const disasm = std.fmt.bufPrint(&disasm_buf, "{f}", .{m68k.Disasm{
                        .reader = &reader.interface,
                    }}) catch @panic("Disassembly formatting out of range");
                    try writer.printAscii(disasm, .{
                        .alignment = .left,
                        .fill = ' ',
                        .width = max_disasm_len,
                    });
                },
                .d => |n| try writer.print("0x{x:0>8}", .{this.cpu.d[n]}),
                .a => |n| try writer.print("0x{x:0>8}", .{switch (n) {
                    7 => this.cpu.sp[@intFromBool(this.cpu.sr.s)],
                    else => this.cpu.a[n],
                }}),
                .sp => |n| try writer.print("0x{x:0>8}", .{this.cpu.sp[n]}),
                .sr => |field| if (field) |f| switch (f) {
                    inline else => |x| try writer.print(
                        "{any}",
                        .{@field(this.cpu.sr, @tagName(x))},
                    ),
                } else try writer.print("0b{b:0>16}", .{@as(u16, @bitCast(this.cpu.sr))}),
                .stop => try writer.print("{}", .{this.cpu.stop}),
                .b => |range| {
                    for (range[0]..range[1]) |addr| {
                        try writer.print(
                            "0x{x:0>4},",
                            .{this.bus.read(@truncate(addr), 0b11) orelse 0},
                        );
                    }
                    try writer.print("0x{x:0>4}", .{this.bus.read(range[1], 0b11) orelse 0});
                    if (std.math.sub(u23, 11, (range[1] - range[0]) * 7)) |n| {
                        _ = try writer.splatByte(' ', n);
                    } else |_| {}
                },
                .st => |l| {
                    const start: u23 = if (l) |limit|
                        @truncate(0x800000 - (std.math.divCeil(usize, limit, 2) catch unreachable))
                    else
                        @as(u23, @truncate(this.cpu.sp[@intFromBool(this.cpu.sr.s)] >> 1));
                    for (start..0x7fffff) |addr| {
                        try writer.print(
                            "0x{x:0>4},",
                            .{this.bus.read(@truncate(addr), 0b11) orelse 0},
                        );
                    }
                    try writer.print("0x{x:0>4}", .{this.bus.read(0x7fffff, 0b11) orelse 0});
                },
            }
        }
    };

    /// Format this token for a header
    pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
        switch (this) {
            .literal => |str| _ = try writer.splatByte(' ', str.len),
            .pc => try writer.print("pc        ", .{}),
            .i => try writer.printAscii("disasm", .{
                .alignment = .left,
                .fill = ' ',
                .width = max_disasm_len,
            }),
            .d => |n| try writer.print("{f}        ", .{m68k.Cpu.Reg.d.fmt(n)}),
            .a => |n| try writer.print("{f}        ", .{m68k.Cpu.Reg.a.fmt(n)}),
            .sp => |n| switch (n) {
                0 => try writer.print("usp       ", .{}),
                1 => try writer.print("ssp       ", .{}),
            },
            .sr => |f| if (f) |flag|
                try writer.printAscii(@tagName(flag), .{
                    .alignment = .left,
                    .fill = ' ',
                    .width = 5,
                })
            else
                try writer.print("  ttsm-ipl---xnzvc", .{}),
            .stop => try writer.print("stop ", .{}),
            .b => |r| {
                const len = (r[1] - r[0]) * 7 + 6;
                try writer.print("0x{x:0>8}", .{@as(usize, r[0]) << 1});
                if (len < 21) {
                    _ = try writer.splatByte(' ', 21 - len);
                } else {
                    _ = try writer.splatByte(' ', len - 20);
                    try writer.print("0x{x:0>8}", .{@as(usize, r[1]) << 1});
                }
            },
            .st => |n| if (n) |limit|
                try writer.printAscii("stack", .{
                    .alignment = .left,
                    .fill = ' ',
                    .width = (std.math.divCeil(usize, limit, 2) catch unreachable) * 7 - 1,
                })
            else
                try writer.print("stack", .{}),
        }
    }

    /// Greedily parses whats in the reader to get the next token
    fn parse(allocator: std.mem.Allocator, reader: *std.io.Reader) error{ParseFailed}!?@This() {
        switch (reader.peekByte() catch |err|
            if (err == error.EndOfStream) return null else return error.ParseFailed) {
            '{' => {
                _ = reader.discard(.limited(1)) catch return error.ParseFailed;
                var spec = std.io.Writer.Allocating.init(allocator);
                defer spec.deinit();
                if ((reader.peekByte() catch return error.ParseFailed) == '{') {
                    _ = reader.discard(.limited(1)) catch return error.ParseFailed;
                    return .{ .literal = "{" };
                }
                while (reader.peekByte()) |byte| {
                    switch (byte) {
                        ':', '}' => break,
                        else => spec.writer.writeByte(reader.takeByte() catch unreachable) catch
                            return error.ParseFailed,
                    }
                } else |_| {
                    return error.ParseFailed;
                }
                switch (std.meta.stringToEnum(
                    std.meta.FieldEnum(@This()),
                    spec.written(),
                ) orelse return error.ParseFailed) {
                    .literal => return error.ParseFailed,
                    inline .pc, .i, .stop => |tok_type| {
                        if ((reader.takeByte() catch return error.ParseFailed) != '}') {
                            return error.ParseFailed;
                        } else {
                            return @unionInit(@This(), @tagName(tok_type), {});
                        }
                    },
                    inline .d, .a, .sp => |tok_type| {
                        if ((reader.takeByte() catch return error.ParseFailed) != ':') {
                            return error.ParseFailed;
                        }
                        var writer = std.io.Writer.Allocating.init(allocator);
                        defer writer.deinit();
                        _ = reader.streamDelimiter(&writer.writer, '}') catch
                            return error.ParseFailed;
                        _ = reader.discard(.limited(1)) catch return error.ParseFailed;
                        return @unionInit(@This(), @tagName(tok_type), std.fmt.parseInt(
                            @FieldType(@This(), @tagName(tok_type)),
                            writer.written(),
                            0,
                        ) catch return error.ParseFailed);
                    },
                    .st => {
                        switch (reader.takeByte() catch return error.ParseFailed) {
                            ':' => {
                                var writer = std.io.Writer.Allocating.init(allocator);
                                defer writer.deinit();
                                _ = reader.streamDelimiter(&writer.writer, '}') catch
                                    return error.ParseFailed;
                                _ = reader.discard(.limited(1)) catch return error.ParseFailed;
                                return .{
                                    .st = std.fmt.parseInt(usize, writer.written(), 0) catch
                                        return error.ParseFailed,
                                };
                            },
                            '}' => return .{ .st = null },
                            else => return error.ParseFailed,
                        }
                    },
                    .sr => {
                        switch (reader.takeByte() catch return error.ParseFailed) {
                            ':' => {
                                var writer = std.io.Writer.Allocating.init(allocator);
                                defer writer.deinit();
                                _ = reader.streamDelimiter(&writer.writer, '}') catch
                                    return error.ParseFailed;
                                _ = reader.discard(.limited(1)) catch return error.ParseFailed;
                                const Flags = std.meta.FieldEnum(m68k.Cpu.Status);
                                return .{ .sr = std.meta.stringToEnum(
                                    Flags,
                                    writer.written(),
                                ) orelse return error.ParseFailed };
                            },
                            '}' => return .{ .sr = null },
                            else => return error.ParseFailed,
                        }
                    },
                    .b => {
                        if ((reader.takeByte() catch return error.ParseFailed) != ':') {
                            return error.ParseFailed;
                        }
                        var writer = std.io.Writer.Allocating.init(allocator);
                        defer writer.deinit();

                        // Start address
                        _ = reader.streamDelimiter(&writer.writer, '-') catch
                            return error.ParseFailed;
                        _ = reader.discard(.limited(1)) catch return error.ParseFailed;
                        const start = @as(u23, @truncate((std.fmt.parseInt(
                            u32,
                            writer.written(),
                            0,
                        ) catch return error.ParseFailed) >> 1));

                        // End address
                        writer.clearRetainingCapacity();
                        _ = reader.streamDelimiter(&writer.writer, '}') catch
                            return error.ParseFailed;
                        _ = reader.discard(.limited(1)) catch return error.ParseFailed;
                        const end = @as(u23, @truncate((std.fmt.parseInt(
                            u32,
                            writer.written(),
                            0,
                        ) catch return error.ParseFailed) >> 1));

                        // Return bytes to print
                        return .{ .b = .{ start, end } };
                    },
                }
            },
            else => {
                var literal = std.io.Writer.Allocating.init(allocator);
                defer literal.deinit();
                while (true) {
                    switch (reader.peekByte() catch |err|
                        if (err == error.EndOfStream) break else return error.ParseFailed) {
                        '{' => break,
                        '}' => {
                            _ = reader.discard(.limited(1)) catch return error.ParseFailed;
                            if ((reader.takeByte() catch return error.ParseFailed) != '}') {
                                return error.ParseFailed;
                            }
                            return .{ .literal = "}" };
                        },
                        else => |byte| {
                            _ = reader.discard(.limited(1)) catch return error.ParseFailed;
                            literal.writer.writeByte(byte) catch return error.ParseFailed;
                        },
                    }
                }
                return .{ .literal = literal.toOwnedSlice() catch return error.ParseFailed };
            },
        }
    }

    /// Releases resources used by this token
    fn deinit(this: @This(), allocator: std.mem.Allocator) void {
        switch (this) {
            .literal => |str| allocator.free(str),
            else => {},
        }
    }
};

test "m68k integration test" {
    // What to print before each instruction and at the end
    const allocator = std.testing.allocator;
    const debug_fmt = std.process.getEnvVarOwned(allocator, "M68K_INTEGRATION_FORMAT") catch
        try allocator.dupe(u8, "");
    defer allocator.free(debug_fmt);
    var tokens = try std.ArrayList(Token).initCapacity(allocator, 8);
    defer {
        for (tokens.items) |token| {
            token.deinit(allocator);
        }
        tokens.deinit(allocator);
    }
    var fmt_reader = std.io.Reader.fixed(debug_fmt);
    while (Token.parse(allocator, &fmt_reader) catch {
        std.log.err("Debugging format wrong at pos: {}", .{fmt_reader.end});
        return error.TestFailed;
    }) |token| {
        try tokens.append(allocator, token);
    }

    // Get what tests we should run
    const run_tests_env = std.process.getEnvVarOwned(allocator, "M68K_INTEGRATION_ENABLE") catch
        try allocator.dupe(u8, "");
    defer allocator.free(run_tests_env);
    var tests_to_run = try std.ArrayList([]const u8).initCapacity(allocator, 8);
    defer tests_to_run.deinit(allocator);
    var run_tests_iter = std.mem.splitScalar(u8, run_tests_env, ',');
    while (run_tests_iter.next()) |@"test"| {
        try tests_to_run.append(allocator, @"test");
    }

    // Find and run every test in the current directory
    var dir = try std.fs.cwd().openDir(config.tests_path, .{ .iterate = true });
    defer dir.close();
    var iter = dir.iterate();
    while (try iter.next()) |entry| {
        // Make sure we are testing a test file
        if (entry.kind != .file or !std.mem.eql(u8, ".zon", std.fs.path.extension(entry.name))) {
            continue;
        }

        // If there are specific tests to run check if this is the test we want to run
        if (run_tests_env.len > 0 and for (tests_to_run.items) |@"test"| {
            if (std.mem.eql(u8, @"test", std.fs.path.stem(entry.name))) {
                break false;
            }
        } else true) {
            continue;
        }

        // Read in the file
        const max_len = 4 * 1024 * 1024;
        const bytes = try dir.readFileAllocOptions(allocator, entry.name, max_len, null, .@"1", 0);
        defer allocator.free(bytes);

        // Parse the file
        const tests = std.zon.parse.fromSlice(Test, allocator, bytes, null, .{}) catch {
            std.log.err("Failed to parse \"{s}\" m68k integration test file\n", .{entry.name});
            return error.ParsingFailed;
        };
        defer std.zon.parse.free(allocator, tests);

        // Create the rom and base ram regions
        var rom = Rom{ .bytes = tests.rom };
        var ram = Ram.init(tests);
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

        // Run each test case in the file
        for (tests.cases, 0..) |case, i| {
            // Print the header for this test
            if (tokens.items.len > 0) {
                std.debug.print(
                    "\n\"{s}\" test case: \"{s}\"\n",
                    .{ std.fs.path.stem(entry.name), case.name },
                );
                for (tokens.items) |token| {
                    std.debug.print("{f}", .{token});
                }
                std.debug.print("\n", .{});
            }

            // Create the rom and ram interface and the bus interface
            ram.load(tests, case);
            var cpu = m68k.Cpu{};

            // Call the run function and run until we've stopped
            cpu.r(.a, 7).* = std.mem.readInt(u32, tests.rom[0..4], .big) -% (4 * 3);
            cpu.pc = std.mem.readInt(u32, tests.rom[4..8], .big);
            var timeout: usize = 0;
            while (!cpu.stop) {
                for (tokens.items) |token| {
                    std.debug.print("{f}", .{Token.State{
                        .tok = token,
                        .bus = &bus,
                        .cpu = &cpu,
                    }});
                }
                if (tokens.items.len > 0) {
                    std.debug.print("\n", .{});
                }

                // Check if we timedout
                timeout += m68k.step(&cpu, &bus);
                if (timeout >= 10000 * 20) {
                    std.log.err("Test case {} timed out in {s}.", .{ i, entry.name });
                    return error.TestFailed;
                }
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
