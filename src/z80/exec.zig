const std = @import("std");

const bus_interface = @import("bus");

const Cpu = @import("Cpu.zig");

/// z80 bus interface, used both for memory and io operations
pub const bus_width = bus_interface.Width{ .addr = 16, .data = 8 };
pub const Bus = bus_interface.Bus(bus_width);

/// Used when creating prefixes for instructions
pub const Prefix = packed struct {
    /// The bytes that make up the instruction
    bytes: Int = 0,

    /// Gets opcode byte n
    pub fn opcode(this: @This(), n: std.math.IntFittingRange(0, max_len - 1)) u8 {
        return @truncate(this.bytes >> @as(std.math.Log2Int(Int), n * 8));
    }

    /// Adds a new byte to the start of the prefix
    pub fn push(this: *@This(), byte: u8) void {
        this.bytes <<= 8;
        this.bytes |= byte;
    }

    /// Maximum length of a instruction prefix
    pub const max_len = 8;
    pub const Int = std.meta.Int(.unsigned, max_len * 8);
};

/// Generate a 256 entry decoder given a list of decoders, solves collisions with priority
pub fn decoder(comptime decoders: []const Pattern) Fetch.Decoder {
    // Sort the decoders by priority (lowest priority first)
    comptime var sorted = decoders.*;
    comptime std.sort.pdq(Pattern, &sorted, void, struct {
        pub fn lessThan(_: void, lhs: Pattern, rhs: Pattern) bool {
            return rhs.priority < lhs.priority;
        }
    }.lessThan);

    // Add them into a look up table
    comptime var lut = [1]Fetch.Decoder{struct {
        pub fn inner(_: *Cpu, _: *Bus) Fetch {
            return .{};
        }
    }.inner} ** 0x100;
    inline for (sorted) |decode| {
        lut[decode.opcode] = decode.fetch;
    }

    // Return a function that looks up the look up table
    const final_lut = lut;
    return struct {
        pub fn inner(cpu: *Cpu, bus: *Bus) Fetch {
            const opcode = fetch(cpu, bus, u8);
            var next = final_lut[opcode](cpu, bus);
            next.prefix.push(opcode);
            return next;
        }
    }.inner;
}

/// Generate multiple instruction permutations from a pattern
pub fn instr(
    comptime pattern: *const [8]u8,
    comptime name: []const u8,
    comptime clk: usize,
    comptime op: anytype,
    comptime dst: ?type,
    comptime src: ?type,
) []const Pattern {
    // Generate pattern matching int's to generate the patterns
    comptime var set: u8 = 0;
    comptime var any: u8 = 0;
    inline for (pattern) |char| {
        set <<= 1;
        any <<= 1;
        switch (char) {
            '1' => set |= 1,
            'x' => any |= 1,
            '0' => {},
            else => @panic("Bad pattern matching character for opcode"),
        }
    }

    // Generate each pattern
    var buffer: [std.mem.count(u8, pattern, "x")]Pattern = undefined;
    var decoders = std.ArrayList(Pattern).initBuffer(&buffer);
    for (0..0x100) |byte| {
        if (byte & ~any ^ set != 0) {
            continue;
        }
        decoders.appendAssumeCapacity(.{
            .priority = any,
            .opcode = byte,
            .fetch = struct {
                pub fn inner(_: *Cpu, _: *Bus) Fetch {
                    return .{ .instr = Instr.init(byte, name, clk, op, dst, src) };
                }
            }.inner,
        });
    }

    // Return the generated patterns
    const final = buffer;
    return &final;
}

/// Represents the data needed to decode an instruction and the function to fetch it
pub const Pattern = struct {
    /// Priority of this pattern in a decoding lut, 0 is highest priority
    priority: u8 = std.math.maxInt(u8),

    /// What opcode this pattern is for
    opcode: u8,

    /// Fetch and get the instruction
    fetch: Fetch.Decoder,
};

/// Result of decoding an instruction
pub const Fetch = struct {
    /// Prefix bytes
    prefix: Prefix = .{},

    /// Instruction decoded
    instr: ?Instr = null,

    /// The actual function that will fetch an instruction
    pub const Decoder = *const fn (cpu: *Cpu, mem: *Bus) @This();
};

/// Result of fetching and decoding an instruction
pub const Instr = struct {
    /// Run the instruction with the cpu, memory bus, and io bus and returns the # of cycles
    run: *const fn (cpu: *Cpu, mem: *Bus, io: *Bus, prefix: Prefix) usize,

    /// Disassembles the instruction using the bytes at reader
    disasm: *const fn (
        writer: *std.io.Writer,
        reader: *std.io.Reader,
        prefix: Prefix,
    ) std.io.Writer.Error!void,

    /// Create a specific instruction
    /// When given the name, and the operands, it will involke the `op` given for the run handler,
    /// and it will generate the disassembly code for the name and the operands.
    /// When generating the run handler, if src is specified, it must be a part of the operands'
    /// arguments. 'dst' may be ommitted but can still be written to. If both arguments are in the
    /// op handler, then both src and dest have to be supplied.
    pub fn init(
        comptime name: []const u8,
        comptime clk: usize,
        comptime op: anytype,
        comptime arg_dst: ?type,
        comptime arg_src: ?type,
    ) Instr {
        return .{
            .run = struct {
                pub fn run(cpu: *Cpu, mem: *Bus, io: *Bus, prefix: Prefix) usize {
                    switch (@as(u2, @intFromBool(arg_dst != null)) << 1 |
                        @as(u2, @intFromBool(arg_src != null))) {
                        0b00 => _ = op(cpu),
                        0b01 => {
                            const src = arg_src orelse unreachable;
                            const src_data = src.load(cpu, mem, io, prefix);
                            _ = op(cpu, src_data);
                        },
                        0b10 => {
                            const dst = arg_dst orelse unreachable;
                            const dst_data = dst.load(cpu, mem, io, prefix);
                            const res = switch (@typeInfo(@TypeOf(op)).@"fn".params.len) {
                                2 => op(cpu, dst_data),
                                1 => op(cpu),
                                else => @compileError("Expected 1 or 2 arguments!"),
                            };
                            if (@hasDecl(dst, "store")) {
                                dst.store(cpu, mem, io, prefix, res);
                            }
                        },
                        0b11 => {
                            const dst = arg_dst orelse unreachable;
                            const src = arg_src orelse unreachable;
                            const dst_data = dst.load(cpu, mem, io, prefix);
                            const src_data = src.load(cpu, mem, io, prefix);
                            const res = switch (@typeInfo(@TypeOf(op)).@"fn".params.len) {
                                3 => op(cpu, dst_data, src_data),
                                2 => op(cpu, src_data),
                                else => @compileError("Expected 2 or 3 arguments!"),
                            };
                            if (@TypeOf(res) != void and @hasDecl(dst, "store")) {
                                dst.store(cpu, mem, io, prefix, res);
                            }
                        },
                    }
                    return clk;
                }
            },
            .disasm = struct {
                pub fn disasm(
                    writer: *std.io.Writer,
                    reader: *std.io.Reader,
                    prefix: Prefix,
                ) std.io.Writer.Error!void {
                    try writer.print("{s}", .{name});
                    const has_dst = @hasDecl(arg_dst orelse struct {}, "disasm");
                    const has_src = @hasDecl(arg_src orelse struct {}, "disasm");
                    if (has_dst) {
                        try writer.print(" ", .{});
                        try (arg_dst orelse unreachable).disasm(writer, reader, prefix);
                    }
                    if (has_src) {
                        try writer.print("{s}", .{if (has_dst) "," else " "});
                        try (arg_src orelse unreachable).disasm(writer, reader, prefix);
                    }
                }
            },
        };
    }
};

/// Helper for reading data from a bus
pub fn read(bus: *Bus, comptime Type: type, addr: u16) Type {
    return switch (@bitSizeOf(Type)) {
        8 => @bitCast(bus.read(addr, 1) orelse 0),
        16 => @bitCast(@as(u16, bus.read(addr, 1) orelse 0) << 8 |
            @as(u16, bus.read(addr, 1) orelse 0)),
        else => @compileError("Expected a type of size 8 or 16 bits for reading z80 bus!"),
    };
}

/// Helper for writing data to a bus
pub fn write(bus: *Bus, comptime Type: type, addr: u16, value: Type) void {
    return switch (@bitSizeOf(Type)) {
        8 => bus.write(addr, 1, @bitCast(value)),
        16 => {
            const word: u16 = @bitCast(value);
            bus.write(addr, 1, @truncate(word >> 8));
            bus.write(addr, 1, @truncate(word));
        },
        else => @compileError("Expected a type of size 8 or 16 bits for writing z80 bus!"),
    };
}

/// Helper function for fetching next word/data
pub fn fetch(cpu: *Cpu, bus: *Bus, comptime Type: type) Type {
    const data = read(bus, Type, cpu.pc);
    cpu.pc += @sizeOf(Type);
    return data;
}
