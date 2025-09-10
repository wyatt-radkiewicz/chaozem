const std = @import("std");

cpu: *Cpu,
bus: *Bus,
clk: usize = 0,

/// M68K execution context
const Exec = @This();

/// M68K cpu state
pub const Cpu = struct {
    /// Data registers
    d: [8]u32 = [1]u32{0} ** 8,
    /// Address registers
    a: [8]u32 = [1]u32{0} ** 8,
    /// Program counter
    pc: u32 = 0,
    /// Status register
    sr: Status = .{},

    /// M68K processor status flags
    pub const Status = packed struct {
        /// Carry
        c: bool = false,
        /// Overflow
        v: bool = false,
        /// Zero
        z: bool = false,
        /// Negative
        n: bool = false,
        /// Extend
        x: bool = false,
        _pad0: u3 = 0,
        /// Interrupt priority level
        ipl: u3 = 0,
        _pad1: u1 = 0,
        /// Master
        m: bool = false,
        /// Supervisor
        s: bool = true,
        /// Trace level
        t: u2 = 0,
    };
};

/// M68K bus interface
pub const Bus = struct {
    /// Read byte/word/long
    rdb: *const fn (bus: *@This(), addr: u32) u8,
    rdw: *const fn (bus: *@This(), addr: u32) u16,
    rdl: *const fn (bus: *@This(), addr: u32) u32,

    /// Write byte/word/long
    wrb: *const fn (bus: *@This(), addr: u32, data: u8) void,
    wrw: *const fn (bus: *@This(), addr: u32, data: u16) void,
    wrl: *const fn (bus: *@This(), addr: u32, data: u32) void,

    /// Create a reader at a certain position
    pub fn reader(this: *@This(), addr: u32, buffer: []u8) Reader {
        return .{
            .bus = this,
            .addr = addr,
            .interface = .{
                .vtable = &.{ .stream = Reader.stream },
                .buffer = buffer,
                .seek = 0,
                .end = 0,
            },
        };
    }

    /// Interface to read
    const Reader = struct {
        bus: *Bus,
        addr: u32,
        interface: std.io.Reader,

        /// Standard reader interface function
        fn stream(
            io_reader: *std.Io.Reader,
            w: *std.Io.Writer,
            limit: std.Io.Limit,
        ) std.Io.Reader.StreamError!usize {
            const this: *@This() = @alignCast(@fieldParentPtr("interface", io_reader));
            const dest = limit.slice(try w.writableSliceGreedy(1));
            for (dest, this.addr..) |*dest_byte, addr| {
                dest_byte.* = this.bus.rdb(this.bus, @truncate(addr));
            }
            this.addr += 1;
            return dest.len;
        }
    };
};

/// Run one instruction
pub fn step(cpu: *Cpu, bus: *Bus) usize {
    var exec = Exec{
        .cpu = cpu,
        .bus = bus,
    };
    const opcode = fetch(&exec, u16);
    if (isa.handler(opcode)) |pfn| {
        pfn(opcode, &exec);
    }
    return exec.clk;
}

/// Disassemble one instruction
const Disasm = struct {
    reader: *std.io.Reader,

    pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
        const opcode = this.reader.takeInt(u16, .big) catch return error.WriteFailed;
        if (isa.disasm(opcode)) |pfn| {
            try pfn(writer, this.reader, opcode);
        } else {
            try writer.print("<invalid opcode>", .{});
        }
    }
};

/// M68k instruction set architecture
const isa = Isa(&.{
    Instr{
        .name = "add",
        .enc = .init("1101xxx0xxxxxxxx"),
        .src = EaTarget(3, 0, .{ .l = .initDefault(2, .{
            .data_reg = 4,
            .addr_reg = 4,
            .immediate = 4,
        }) }),
        .dst = DataTarget(9),
        .op = Add,
        .size = Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    Instr{
        .name = "add",
        .enc = .init("1101xxx1xxxxxxxx"),
        .src = DataTarget(9),
        .dst = EaTarget(3, 0, .{}),
        .op = Add,
        .size = Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    Instr{
        .name = "ori",
        .enc = .init("00000000xx111100"),
        .src = ImmTarget,
        .dst = SrTarget(12),
        .op = Or,
        .size = Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01 } },
    },
    Instr{
        .name = "ori",
        .enc = .init("00000000xxxxxxxx"),
        .src = ImmTarget,
        .dst = EaTarget(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = Or,
        .size = Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    Instr{
        .name = "nop",
        .enc = .init("0100111001110001"),
    },
});

/// A M68K instruction definition
const Instr = struct {
    /// Name of the instruction
    name: []const u8,
    /// Opcode's encoding
    enc: Opcode,
    /// Source of the instruction
    src: ?type = null,
    /// Destination of the instruction
    dst: ?type = null,
    /// What operation to perform
    op: ?type = null,
    /// The size(s) of the instruction
    size: ?Size.Enc = null,

    /// Runtime operand
    fn Operand(comptime Target: type, comptime size: ?Size) type {
        return struct {
            operand: Target,
            data: Data,

            const Data = switch (@typeInfo(@TypeOf(Target.Data)).@"fn".params[0].type orelse
                void) {
                Size => Target.Data(size orelse @compileError("Expected size for operand!")),
                ?Size => Target.Data(size),
                else => @compileError("Expected size parameter"),
            };

            fn init(exec: *Exec, opcode: u16) @This() {
                const InitParam = @typeInfo(@TypeOf(Target.init)).@"fn".params[1].type orelse void;
                var operand = switch (InitParam) {
                    Size => Target.init(exec, size orelse unreachable, opcode),
                    ?Size => Target.init(exec, size, opcode),
                    else => @compileError("Expected size parameter"),
                };
                const LoadParam = @typeInfo(@TypeOf(Target.load)).@"fn".params[2].type orelse void;
                const data = switch (LoadParam) {
                    Size => operand.load(exec, size orelse unreachable, opcode),
                    ?Size => operand.load(exec, size, opcode),
                    else => @compileError("Expected size parameter"),
                };
                return .{
                    .operand = operand,
                    .data = data,
                };
            }

            fn store(this: *@This(), exec: *Exec, opcode: u16, data: Data) void {
                const Param = @typeInfo(@TypeOf(Target.store)).@"fn".params[2].type orelse void;
                switch (Param) {
                    Size => this.operand.store(exec, size orelse unreachable, opcode, data),
                    ?Size => this.operand.store(exec, size, opcode, data),
                    else => @compileError("Expected size parameter"),
                }
            }
        };
    }

    /// Instruction specialization
    const Spec = struct {
        enc: Opcode,
        run: *const fn (u16, *Exec) void,
        disasm: *const fn (*std.io.Writer, *std.io.Reader, u16) std.io.Writer.Error!void,

        fn init(comptime instr: Instr, comptime size: ?Size) @This() {
            const enc = if (instr.size) |enc| switch (enc) {
                .fixed => instr.enc,
                .dyn => |dyn| instr.enc.overwrite(dyn.at, enc.encode(size orelse
                    @compileError("Expected size for instruction"))),
            } else instr.enc;
            return .{
                .enc = enc,
                .run = struct {
                    pub fn run(opcode: u16, exec: *Exec) void {
                        if (instr.src) |Src| {
                            const SrcOp = Operand(Src, size);
                            if (instr.dst) |Dst| {
                                const DstOp = Operand(Dst, size);
                                const src = SrcOp.init(exec, opcode);
                                var dst = DstOp.init(exec, opcode);
                                const result = runop(instr.op, SrcOp.Data, size, exec, .{
                                    src.data,
                                    dst.data,
                                });
                                dst.store(exec, opcode, result);
                            } else {
                                const src = SrcOp.init(exec, opcode);
                                _ = runop(instr.op, SrcOp.Data, size, exec, .{src.data});
                            }
                        } else if (instr.dst) |Dst| {
                            const DstOp = Operand(Dst, size);
                            var dst = DstOp.init(exec, opcode);
                            const result = runop(instr.op, DstOp.Data, size, exec, .{dst.data});
                            dst.store(exec, opcode, result);
                        } else {
                            const Data = if (size) |s| s.Int(.unsigned) else void;
                            _ = runop(instr.op, Data, size, exec, .{});
                        }
                    }
                }.run,
                .disasm = struct {
                    fn disasm(
                        writer: *std.io.Writer,
                        reader: *std.io.Reader,
                        opcode: u16,
                    ) std.io.Writer.Error!void {
                        _ = try writer.write(instr.name);
                        if (size) |s| switch (instr.size orelse .{ .fixed = .l }) {
                            .fixed => {},
                            .dyn => try writer.print(".{s}", .{@tagName(s)}),
                        };
                        if (instr.src) |src| {
                            if (instr.dst) |dst| {
                                try writer.print(" {f},{f}", .{
                                    initdisasm(size, src, reader, opcode),
                                    initdisasm(size, dst, reader, opcode),
                                });
                            } else {
                                try writer.print(" {f}", .{initdisasm(size, src, reader, opcode)});
                            }
                        } else if (instr.dst) |dst| {
                            try writer.print(" {f}", .{initdisasm(size, dst, reader, opcode)});
                        }
                    }
                }.disasm,
            };
        }

        /// Run a operation handler
        fn runop(
            comptime op: ?type,
            comptime Result: type,
            comptime size: ?Size,
            exec: *Exec,
            targets: anytype,
        ) Result {
            const Op = op orelse return if (targets.len > 0) targets[targets.len - 1] else {};
            const Param = @typeInfo(@TypeOf(Op.op)).@"fn".params[1].type orelse
                @panic("Expected size as second parameter");
            return @call(.auto, Op.op, .{
                exec,
                if (Param == Size) size orelse @panic("Expected size for instruction target") else size,
            } ++ targets);
        }

        /// Initialize the disassembly formatter
        fn initdisasm(
            comptime size: ?Size,
            comptime Target: type,
            reader: *std.io.Reader,
            opcode: u16,
        ) Target.Disasm(if (@typeInfo(@TypeOf(Target.Disasm)).@"fn".params[0].type == ?Size)
            size
        else
            size orelse @compileError("Expected concrete size")) {
            return Target.Disasm(if (@typeInfo(@TypeOf(Target.Disasm)).@"fn".params[0].type == ?Size)
                size
            else
                size orelse @compileError("Expected concrete size")){
                .reader = reader,
                .opcode = opcode,
            };
        }
    };

    /// Get all instruction permutations for this
    fn specializations(comptime this: @This()) []const Spec {
        var buffer: [std.meta.fields(Size).len]Spec = undefined;
        var perms = std.ArrayList(Spec).initBuffer(&buffer);
        if (this.size) |enc| {
            switch (enc) {
                .fixed => |size| perms.appendAssumeCapacity(.init(this, size)),
                .dyn => |x| for (std.meta.fieldNames(@TypeOf(x))) |field| {
                    if (@FieldType(@TypeOf(x), field) == ?comptime_int and
                        @field(x, field) != null)
                    {
                        perms.appendAssumeCapacity(.init(this, @field(Size, field)));
                    }
                },
            }
        } else {
            perms.appendAssumeCapacity(.init(this, null));
        }
        const final = buffer;
        return final[0..perms.items.len];
    }
};

/// Updates condition flags
fn Flags(comptime flags: struct {
    c: Flag = .keep,
    v: Flag = .keep,
    z: Flag = .keep,
    n: Flag = .keep,
    x: Flag = .keep,
}) type {
    const FlagName = enum { c, v, z, n, x };
    return struct {
        pub inline fn apply(status: *Cpu.Status, updates: anytype) void {
            var set = comptime @as(u5, @intFromBool(flags.c.mask(.set))) << 0 |
                @as(u5, @intFromBool(flags.v.mask(.set))) << 1 |
                @as(u5, @intFromBool(flags.z.mask(.set))) << 2 |
                @as(u5, @intFromBool(flags.n.mask(.set))) << 3 |
                @as(u5, @intFromBool(flags.x.mask(.set))) << 4;
            var mask = comptime @as(u5, @intFromBool(flags.c.mask(.overwrite))) << 0 |
                @as(u5, @intFromBool(flags.v.mask(.overwrite))) << 1 |
                @as(u5, @intFromBool(flags.z.mask(.overwrite))) << 2 |
                @as(u5, @intFromBool(flags.n.mask(.overwrite))) << 3 |
                @as(u5, @intFromBool(flags.x.mask(.overwrite))) << 4;
            inline for (comptime std.meta.fieldNames(@TypeOf(updates))) |flag| {
                const pos = @intFromEnum(@field(FlagName, flag));
                const bit = @intFromBool(@field(updates, flag));
                switch (@field(flags, flag)) {
                    .set => set |= @as(u5, bit) << pos,
                    .clr => mask |= @as(u5, ~bit) << pos,
                    else => @compileError("Unneeded flag given"),
                }
            }

            status.* = @bitCast(@as(u16, @bitCast(status.*)) & ~@as(u16, mask) | set);

            inline for (comptime std.meta.fieldNames(@TypeOf(flags))) |flag| {
                switch (comptime @field(flags, flag)) {
                    inline .c, .v, .z, .n, .x => |copy| {
                        const bits: u16 = @bitCast(status.*);
                        const src = @intFromEnum(@field(FlagName, @tagName(copy)));
                        const dst = @intFromEnum(@field(FlagName, flag));
                        status.* = @bitCast(bits & ~@as(u16, 1 << dst) |
                            (@as(u16, @as(u1, @truncate(bits >> src))) << dst));
                    },
                    else => {},
                }
            }
        }
    };
}

/// What updates to do for the flags
const Flag = enum {
    /// Do nothing, set to zero, or one always
    keep,
    zero,
    one,

    /// Copy another flag given in the updates list
    c,
    v,
    z,
    n,
    x,

    /// Set it or clear
    set,

    /// Clear it or do nothing
    clr,

    /// Default bit for a mask
    fn mask(this: @This(), usage: enum { set, overwrite }) bool {
        return switch (usage) {
            .set => this == .zero,
            .overwrite => this != .keep,
        };
    }
};

/// M68K operation sizes
const Size = enum {
    /// Byte
    b,
    /// Word
    w,
    /// Long
    l,

    /// Get the integer type for this operation size
    fn Int(comptime this: @This(), comptime signedness: std.builtin.Signedness) type {
        return std.meta.Int(signedness, this.bitSize());
    }

    /// Get the number of bits for this size
    fn bitSize(comptime this: @This()) comptime_int {
        return switch (this) {
            .b => 8,
            .w => 16,
            .l => 32,
        };
    }

    /// M68K size encodings
    const Enc = union(enum) {
        /// Fixed size at compile time
        fixed: Size,
        /// Size determined from bits at a position in the opcode
        dyn: struct {
            at: comptime_int,
            b: ?comptime_int = null,
            w: ?comptime_int = null,
            l: ?comptime_int = null,
        },

        /// Gets the integer type used to represent the size
        fn Type(comptime this: @This()) type {
            return std.math.IntFittingRange(0, this.biggestEncoding() orelse 0);
        }

        /// Decodes a size, and returns null if there was no encoding for the bits given
        fn decode(comptime this: @This(), opcode: u16) ?Size {
            return switch (this) {
                .fixed => |size| size,
                .dyn => |enc| blk: {
                    const map = comptime build: {
                        var map = [1]?Size{null} ** (this.biggestEncoding() orelse 0 + 1);
                        for (std.meta.fieldNames(@TypeOf(enc))) |field| {
                            if (@FieldType(enc, field) == ?comptime_int) {
                                map[@field(enc, field) orelse continue] = @field(Size, field);
                            }
                        }
                        break :build map;
                    };
                    break :blk map[extract(this.Type(), opcode, enc.at)];
                },
            };
        }

        /// Encodes a size and returns the set bits
        fn encode(comptime this: @This(), size: Size) this.Type() {
            return switch (this) {
                .fixed => {},
                .dyn => |enc| switch (size) {
                    inline else => |s| @field(enc, @tagName(s)) orelse 0 << enc.at,
                },
            };
        }

        /// Gets the biggest encoding value
        fn biggestEncoding(comptime this: @This()) ?comptime_int {
            return switch (this) {
                .fixed => null,
                .dyn => |enc| @max(enc.b orelse 0, enc.w orelse 0, enc.l orelse 0),
            };
        }
    };
};

/// Matches against ranges of opcodes
const Opcode = struct {
    set: u16,
    any: u16,

    /// Match '0','1', or 'x'
    fn init(enc: *const [16]u8) @This() {
        var set: u16 = 0;
        var any: u16 = 0;
        for (enc) |char| {
            set <<= 1;
            any <<= 1;
            switch (char) {
                '1' => set |= 1,
                'x' => any |= 1,
                '0' => {},
                else => @panic("Bad opcode matching character"),
            }
        }
        return .{ .set = set, .any = any };
    }

    fn match(this: @This(), opcode: u16) bool {
        return opcode ^ this.set & ~this.any == 0;
    }

    /// Overwrite some bits at a specific position
    fn overwrite(this: @This(), at: u4, with: anytype) @This() {
        const bits = switch (@TypeOf(with)) {
            comptime_int => @as(std.math.IntFittingRange(0, with), with),
            u0 => return this,
            else => with,
        };
        const mask: u16 = ((1 << @bitSizeOf(@TypeOf(bits))) - 1) << at;
        return .{
            .set = this.set & ~mask | @as(u16, bits) << at,
            .any = this.any & ~mask,
        };
    }
};

/// Data register source or destination target
fn DataTarget(n: u4) type {
    return struct {
        n: u3,

        fn Data(comptime size: Size) type {
            return size.Int(.unsigned);
        }

        fn init(_: *Exec, comptime _: Size, opcode: u16) @This() {
            return .{ .n = extract(u3, opcode, n) };
        }

        fn load(this: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
            return @truncate(exec.cpu.d[this.n]);
        }

        fn store(this: @This(), exec: *Exec, comptime size: Size, _: u16, data: Data(size)) void {
            exec.cpu.d[this.n] = overwrite(exec.cpu.d[this.n], data);
        }

        fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("d{}", .{extract(u3, this.opcode, n)});
                }
            };
        }
    };
}

/// Address register source or destination target
fn AddrTarget(n: u4) type {
    return struct {
        n: u3,

        fn Data(comptime size: Size) type {
            return size.Int(.unsigned);
        }

        fn init(_: *Exec, comptime _: Size, opcode: u16) @This() {
            return .{ .n = extract(u3, opcode, n) };
        }

        fn load(this: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
            return @truncate(exec.cpu.a[this.n]);
        }

        fn store(this: @This(), exec: *Exec, comptime size: Size, _: u16, data: Data(size)) void {
            exec.cpu.a[this.n] = extend(u32, data);
        }

        fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("a{}", .{extract(u3, this.opcode, n)});
                }
            };
        }
    };
}

/// Status register source or destination target
fn SrTarget(comptime delay: usize) type {
    return struct {
        fn Data(comptime size: Size) type {
            return switch (size) {
                .l => @compileError("Status register is only 16 bits!"),
                else => size.Int(.unsigned),
            };
        }

        fn init(exec: *Exec, comptime _: Size, _: u16) @This() {
            exec.clk += delay;
            return .{};
        }

        fn load(_: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
            const bits: u16 = @bitCast(exec.cpu.sr);
            return @truncate(bits);
        }

        fn store(_: @This(), exec: *Exec, comptime size: Size, _: u16, data: Data(size)) void {
            const bits: u16 = @bitCast(exec.cpu.sr);
            exec.cpu.sr = @bitCast(overwrite(bits, data));
        }

        fn Disasm(comptime size: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(_: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    try writer.print("{s}", .{switch (size) {
                        .b => "ccr",
                        .w => "sr",
                        .l => @compileError("Expected byte or word with status register disasm!"),
                    }});
                }
            };
        }
    };
}

/// Immediate source data target
const ImmTarget = struct {
    fn Data(comptime size: Size) type {
        return size.Int(.unsigned);
    }

    fn init(_: *Exec, comptime _: Size, _: u16) @This() {
        return .{};
    }

    fn load(_: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
        return exec.fetch(Data(size));
    }

    fn store(_: @This(), _: *Exec, comptime size: Size, _: u16, _: Data(size)) void {}

    fn Disasm(comptime size: Size) type {
        return struct {
            reader: *std.io.Reader,
            opcode: u16,

            pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                switch (size) {
                    .b => try writer.print("#${x:0>2}", .{@as(u8, @truncate(
                        this.reader.takeInt(u16, .big) catch return error.WriteFailed,
                    ))}),
                    .w => try writer.print("#${x:0>4}", .{this.reader.takeInt(u16, .big) catch
                        return error.WriteFailed}),
                    .l => try writer.print("#${x:0>8}", .{this.reader.takeInt(u32, .big) catch
                        return error.WriteFailed}),
                }
            }
        };
    }
};

/// Address modes
const Mode = enum {
    data_reg,
    addr_reg,
    indirect,
    post_inc,
    pre_dec,
    addr_disp,
    addr_idx,
    abs_word,
    abs_long,
    pc_disp,
    pc_idx,
    immediate,

    /// Get from 'm' and 'n' bits
    fn decode(m: u3, n: u3) @This() {
        return switch (m) {
            0b000 => .data_reg,
            0b001 => .addr_reg,
            0b010 => .indirect,
            0b011 => .post_inc,
            0b100 => .pre_dec,
            0b101 => .addr_disp,
            0b110 => .addr_idx,
            0b111 => switch (n) {
                0b000 => .abs_word,
                0b001 => .abs_long,
                0b010 => .pc_disp,
                0b011 => .pc_idx,
                0b100 => .immediate,
                else => @panic("Bad address mode encoding"),
            },
        };
    }

    /// Calculate an address, reg can be left as undefined if its a mode that doesn't require it
    fn calc(this: @This(), exec: *Exec, comptime size: Size, reg: u3) u32 {
        return switch (this) {
            .data_reg, .addr_reg => reg,
            .indirect => exec.cpu.a[reg],
            .post_inc => post_inc: {
                const addr = exec.cpu.a[reg];
                exec.cpu.a[reg] += size.bitSize() / 8;
                break :post_inc addr;
            },
            .pre_dec => pre_dec: {
                exec.cpu.a[reg] -= size.bitSize() / 8;
                exec.clk += 2;
                break :pre_dec exec.cpu.a[reg];
            },
            .addr_disp => exec.cpu.a[reg] +% extend(u32, exec.fetch(u16)),
            .addr_idx => exec.cpu.a[reg] +% exec.fetch(Index).getDisp(exec),
            .abs_word => extend(u32, exec.fetch(u16)),
            .abs_long => exec.fetch(u32),
            .pc_disp => exec.cpu.pc +% extend(u32, exec.fetch(u16)),
            .pc_idx => exec.cpu.pc +% exec.fetch(Index).getDisp(exec),
            .immediate => 0,
        };
    }

    /// Index extension word used in address calculation
    const Index = packed struct {
        disp: i8,
        _pad0: u3,
        size: u1,
        n: u3,
        m: u1,

        /// Gets the total displacement of the index word (casted to unsigned int)
        fn getDisp(this: @This(), exec: *Exec) u32 {
            exec.clk += 2;
            const reg = switch (this.m) {
                0 => exec.cpu.d[this.n],
                1 => exec.cpu.a[this.n],
            };
            return switch (this.size) {
                0 => extend(u32, @as(u16, @truncate(reg))),
                1 => reg,
            } +% extend(u32, this.disp);
        }
    };

    /// Disassemble an addressing mode
    fn Disasm(comptime size: Size) type {
        return struct {
            reader: *std.io.Reader,
            mode: Mode,
            reg: u3,

            pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                switch (this.mode) {
                    .data_reg => try writer.print("d{}", .{this.reg}),
                    .addr_reg => try writer.print("a{}", .{this.reg}),
                    .indirect => try writer.print("(a{})", .{this.reg}),
                    .post_inc => try writer.print("(a{})+", .{this.reg}),
                    .pre_dec => try writer.print("-(a{})", .{this.reg}),
                    .addr_disp => try writer.print("({}, a{})", .{
                        this.reader.takeInt(i16, .big) catch return error.WriteFailed,
                        this.reg,
                    }),
                    .addr_idx, .pc_idx => |mode| {
                        const idx: Index = @bitCast(this.reader.takeInt(u16, .big) catch
                            return error.WriteFailed);
                        try writer.print("({}, ", .{idx.disp});
                        switch (mode) {
                            .addr_idx => try writer.print("a{}", .{this.reg}),
                            .pc_idx => try writer.print("pc", .{}),
                            else => unreachable,
                        }
                        try writer.print(", {c}{}.{c})", .{ switch (idx.m) {
                            0 => @as(u8, 'd'),
                            1 => @as(u8, 'a'),
                        }, idx.n, switch (idx.size) {
                            0 => @as(u8, 'w'),
                            1 => @as(u8, 'l'),
                        } });
                    },
                    .abs_word => try writer.print(
                        "(${x:0>4}).w",
                        .{this.reader.takeInt(u16, .big) catch return error.WriteFailed},
                    ),
                    .abs_long => try writer.print(
                        "(${x:0>8}).l",
                        .{this.reader.takeInt(u32, .big) catch return error.WriteFailed},
                    ),
                    .pc_disp => try writer.print("({}, pc)", .{this.reader.takeInt(i16, .big) catch
                        return error.WriteFailed}),
                    .immediate => switch (size) {
                        .b => try writer.print("#${x:0>2}", .{@as(u8, @truncate(
                            this.reader.takeInt(u16, .big) catch return error.WriteFailed,
                        ))}),
                        .w => try writer.print("#${x:0>4}", .{this.reader.takeInt(u16, .big) catch
                            return error.WriteFailed}),
                        .l => try writer.print("#${x:0>8}", .{this.reader.takeInt(u32, .big) catch
                            return error.WriteFailed}),
                    },
                }
            }
        };
    }
};

/// Effective address clock delays
const EaDelay = struct {
    b: std.EnumArray(Mode, usize) = .initFill(0),
    w: std.EnumArray(Mode, usize) = .initFill(0),
    l: std.EnumArray(Mode, usize) = .initFill(0),

    /// Gets the delay for the mode and size combination
    fn delay(comptime this: @This(), comptime size: Size, mode: Mode) usize {
        return @field(this, @tagName(size)).get(mode);
    }
};

/// Effective address source or destination target
fn EaTarget(comptime m: u4, comptime n: u4, comptime delay: EaDelay) type {
    return struct {
        mode: Mode,
        addr: u32,

        fn Data(comptime size: Size) type {
            return size.Int(.unsigned);
        }

        fn init(exec: *Exec, comptime size: Size, opcode: u16) @This() {
            const reg = extract(u3, opcode, n);
            const mode = Mode.decode(extract(u3, opcode, m), reg);
            exec.clk += delay.delay(size, mode);
            return .{ .mode = mode, .addr = mode.calc(exec, size, reg) };
        }

        fn load(this: @This(), exec: *Exec, comptime size: Size, _: u16) Data(size) {
            return switch (this.mode) {
                .data_reg => @truncate(exec.cpu.d[this.addr]),
                .addr_reg => @truncate(exec.cpu.a[this.addr]),
                .immediate => exec.fetch(size.Int(.unsigned)),
                else => exec.read(size.Int(.unsigned), this.addr),
            };
        }

        fn store(this: @This(), exec: *Exec, comptime size: Size, _: u16, data: Data(size)) void {
            switch (this.mode) {
                .data_reg => exec.cpu.d[this.addr] = overwrite(exec.cpu.d[this.addr], data),
                .addr_reg => exec.cpu.a[this.addr] = overwrite(exec.cpu.a[this.addr], data),
                .immediate => {},
                else => exec.write(size.Int(.unsigned), this.addr, data),
            }
        }

        fn Disasm(comptime size: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
                    const reg = extract(u3, this.opcode, n);
                    try writer.print("{f}", .{Mode.Disasm(size){
                        .reader = this.reader,
                        .mode = Mode.decode(extract(u3, this.opcode, m), reg),
                        .reg = reg,
                    }});
                }
            };
        }
    };
}

/// Normal addition operation
const Add = struct {
    fn op(
        exec: *Exec,
        comptime size: Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const sum = src +% dst;
        const overflow = @addWithOverflow(
            @as(size.Int(.signed), @bitCast(src)),
            @as(size.Int(.signed), @bitCast(dst)),
        )[1] == 1;
        const carry = @addWithOverflow(src, dst)[1] == 1;
        Flags(.{
            .x = .c,
            .n = .set,
            .z = .set,
            .v = .set,
            .c = .set,
        }).apply(&exec.cpu.sr, .{
            .n = negative(sum),
            .z = sum == 0,
            .v = overflow,
            .c = carry,
        });
        return sum;
    }
};

/// Normal or operation
const Or = struct {
    fn op(
        exec: *Exec,
        comptime size: Size,
        src: size.Int(.unsigned),
        dst: size.Int(.unsigned),
    ) size.Int(.unsigned) {
        const result = src | dst;
        Flags(.{
            .n = .set,
            .z = .set,
            .v = .zero,
            .c = .zero,
        }).apply(&exec.cpu.sr, .{
            .n = negative(result),
            .z = result == 0,
        });
        return result;
    }
};

/// Matches opcodes to instruction permutations
const Matcher = struct {
    perms: []const Instr.Spec,

    /// Create the matcher
    fn init(comptime instrs: []const Instr) @This() {
        var num_perms = 0;
        for (instrs) |instr| {
            num_perms += instr.specializations().len;
        }

        var perms: [num_perms]Instr.Spec = undefined;
        var perms_builder = std.ArrayList(Instr.Spec).initBuffer(&perms);
        for (instrs) |instr| {
            perms_builder.appendSliceAssumeCapacity(instr.specializations());
        }

        std.sort.pdq(Instr.Spec, &perms, {}, struct {
            fn lessThanFn(_: void, lhs: Instr.Spec, rhs: Instr.Spec) bool {
                return @popCount(lhs.enc.any) < @popCount(rhs.enc.any);
            }
        }.lessThanFn);
        const final = perms;
        return .{ .perms = &final };
    }

    /// Get the index of the permutation that was matched
    fn match(comptime this: @This(), opcode: u16) ?comptime_int {
        return for (0..this.perms.len) |idx| {
            if (this.perms[idx].enc.match(opcode)) {
                break idx;
            }
        } else null;
    }
};

/// Builds a sparse look up table for decoding, where Index's bit size is a multiple of 4
fn sparselut(
    comptime Index: type,
    comptime context: anytype,
    comptime getter: fn (@TypeOf(context), Index) ?comptime_int,
) fn (Index) SparseLut(Index, context, getter) {
    const level_size = 4;
    const index_size = @bitSizeOf(Index);
    const Node = struct {
        entries: [1 << level_size]?comptime_int = [1]?comptime_int{null} ** (1 << level_size),

        fn visit(
            comptime nodes: *std.ArrayList(@This()),
            comptime null_entry: *comptime_int,
            comptime prefix: anytype,
        ) ?comptime_int {
            const prefix_len = if (@TypeOf(prefix) == void) 0 else @bitSizeOf(@TypeOf(prefix));
            if (prefix_len >= index_size) {
                const entry = getter(context, @bitCast(prefix));
                null_entry.* = @max(null_entry.*, (entry orelse return entry) + 1);
                return entry;
            } else {
                var node = @This(){};
                for (&node.entries, 0..1 << level_size) |*entry, i| {
                    var next_prefix = @as(
                        std.meta.Int(.unsigned, prefix_len + level_size),
                        if (prefix_len == 0) 0 else prefix,
                    );
                    if (prefix_len > 0) {
                        next_prefix <<= level_size;
                    }
                    next_prefix += @as(std.meta.Int(.unsigned, level_size), @intCast(i));
                    entry.* = visit(nodes, null_entry, next_prefix);
                }
                return node.add(nodes);
            }
        }

        fn add(comptime this: @This(), comptime nodes: *std.ArrayList(@This())) usize {
            return for (nodes.items, 0..) |node, idx| {
                if (std.mem.eql(?comptime_int, &this.entries, &node.entries)) {
                    break idx;
                }
            } else add: {
                nodes.appendAssumeCapacity(this);
                break :add nodes.items.len - 1;
            };
        }
    };

    var node_buffer: [1 + (1 << level_size * (index_size / level_size - 1))]Node = undefined;
    var nodes = std.ArrayList(Node).initBuffer(&node_buffer);
    var null_entry = 0;

    const top_level = Node.visit(&nodes, &null_entry, void) orelse unreachable;
    const Entry = std.math.IntFittingRange(0, @max(nodes.items.len - 1, null_entry));

    var compressed: [nodes.items.len << level_size]Entry = undefined;
    for (&compressed, 0..) |*entry, i| {
        entry.* = nodes.items[i >> level_size].entries[i % (1 << level_size)] orelse null_entry;
    }
    const final = compressed;
    const RetType = ?std.math.IntFittingRange(0, null_entry - 1);
    const null_return = null_entry;

    return struct {
        pub fn lut(index: Index) RetType {
            const first_level = index_size - level_size;
            var entry: usize = final[(top_level << level_size) + (index >> first_level)];
            inline for (1..index_size / level_size) |level| {
                const nibble: u4 = @truncate(index >> first_level - level * level_size);
                entry = final[(entry << level_size) + nibble];
            }
            return if (entry == null_return) null else @intCast(entry);
        }
    }.lut;
}

fn SparseLut(
    comptime Index: type,
    comptime context: anytype,
    comptime getter: fn (@TypeOf(context), Index) ?comptime_int,
) type {
    @setEvalBranchQuota((1 << @bitSizeOf(Index)) * 1000);
    var max = 0;
    for (0..1 << @bitSizeOf(Index)) |i| {
        max = @max(max, getter(context, i) orelse 0);
    }
    return ?std.math.IntFittingRange(0, max);
}

/// Instruction set architecture faciltates runtime running of instructions and disassembling
fn Isa(comptime instrs: []const Instr) type {
    const matcher = Matcher.init(instrs);
    const lut = sparselut(u16, matcher, struct {
        fn getter(m: Matcher, index: u16) ?comptime_int {
            return m.match(index);
        }
    }.getter);
    return struct {
        /// Gets the instruction handler for the opcode
        pub fn handler(opcode: u16) ?*const fn (u16, *Exec) void {
            return matcher.perms[lut(opcode) orelse return null].run;
        }

        /// Gets the disassembler for the opcode
        pub fn disasm(
            opcode: u16,
        ) ?*const fn (*std.io.Writer, *std.io.Reader, u16) std.io.Writer.Error!void {
            return matcher.perms[lut(opcode) orelse return null].disasm;
        }
    };
}

/// Fetch a type from the program counter of the cpu
inline fn fetch(this: *Exec, comptime Data: type) Data {
    const fetch_width = @max(16, @bitSizeOf(Data));
    const data = this.read(std.meta.Int(.unsigned, fetch_width), this.cpu.pc);
    this.cpu.pc += fetch_width / 8;
    return @bitCast(@as(std.meta.Int(.unsigned, @bitSizeOf(Data)), @truncate(data)));
}

/// Read data type X from the bus
inline fn read(this: *Exec, comptime Data: type, addr: u32) Data {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            return @bitCast(this.bus.rdb(this.bus, addr));
        },
        16 => {
            this.clk += 4;
            return @bitCast(this.bus.rdw(this.bus, addr));
        },
        32 => {
            this.clk += 8;
            return @bitCast(this.bus.rdl(this.bus, addr));
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to read data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Write data type X to the bus
inline fn write(this: *Exec, comptime Data: type, addr: u32, data: Data) void {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            this.bus.wrb(this.bus, addr, @bitCast(data));
        },
        16 => {
            this.clk += 4;
            this.bus.wrw(this.bus, addr, @bitCast(data));
        },
        32 => {
            this.clk += 8;
            this.bus.wrl(this.bus, addr, @bitCast(data));
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to write data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Extract a type from an integer at a position
inline fn extract(comptime Type: type, from: anytype, at: std.math.Log2Int(@TypeOf(from))) Type {
    const TypeInt = std.meta.Int(.unsigned, @bitSizeOf(Type));
    const FromInt = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(from)));
    return @bitCast(@as(TypeInt, @truncate(@as(FromInt, @bitCast(from)) >> at)));
}

/// Sign extend a specified integer to the specified size
inline fn extend(comptime To: type, from: anytype) To {
    const ToSigned = std.meta.Int(.signed, @bitSizeOf(To));
    const FromSigned = std.meta.Int(.signed, @bitSizeOf(@TypeOf(from)));
    return @bitCast(@as(ToSigned, @as(FromSigned, @bitCast(from))));
}

/// Overwrite the lower N bits with another integer
inline fn overwrite(int: anytype, with: anytype) @TypeOf(int) {
    const Int = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(int)));
    const mask: Int = (1 << @bitSizeOf(@TypeOf(with))) - 1;
    return int & ~mask | with;
}

/// Returns true if the int has the highest bit set
inline fn negative(int: anytype) bool {
    return @as(std.meta.Int(.signed, @bitSizeOf(@TypeOf(int))), @bitCast(int)) < 0;
}

/// A simple test runner for instructions
const Test = struct {
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

    /// Initialize with some opcodes and default cpu state
    fn init(opcodes: []const u16) @This() {
        var this = @This(){};
        for (0..opcodes.len) |i| {
            std.mem.writeInt(u16, this.mem[i * 2 ..][0..2], opcodes[i], .big);
        }
        return this;
    }

    /// Run one instruction
    fn run(this: *@This(), disasm: []const u8) !usize {
        var reader_buffer = [1]u8{0} ** 8;
        var reader = this.interface.reader(0, &reader_buffer);
        var writer_buffer = std.ArrayList(u8){};
        var writer = writer_buffer.writer(std.testing.allocator);
        defer writer_buffer.deinit(std.testing.allocator);
        try writer.print("{f}", .{
            Disasm{ .reader = &reader.interface },
        });
        try std.testing.expectEqualSlices(u8, disasm, writer_buffer.items);
        return step(&this.cpu, &this.interface);
    }

    /// Reading/writing instructions
    fn rd(bus: *Bus, addr: u32, comptime Data: type) Data {
        const this: *Test = @fieldParentPtr("interface", bus);
        return std.mem.readInt(Data, this.mem[addr..][0..@sizeOf(Data)], .big);
    }
    fn wr(bus: *Bus, addr: u32, comptime Data: type, data: Data) void {
        const this: *Test = @fieldParentPtr("interface", bus);
        std.mem.writeInt(Data, this.mem[addr..][0..@sizeOf(Data)], data, .big);
    }
};

test "add ea,dn; add dn,ea" {
    var runner: Test = undefined;

    // 1) Test that the <instruction> is encoded correctly
    runner = Test.init(&.{ 0xD038, 0x0080 });
    Test.wr(&runner.interface, 0x80, u8, 35);
    runner.cpu.d[0] = 10;
    try std.testing.expectEqual(12, try runner.run("add.b ($0080).w,d0"));
    try std.testing.expectEqual(45, runner.cpu.d[0]);

    // 2) Test that the <instruction> is encoded correctly
    runner = Test.init(&.{ 0xD1B8, 0x0080 });
    Test.wr(&runner.interface, 0x80, u32, 350000);
    runner.cpu.d[0] = 10;
    try std.testing.expectEqual(24, try runner.run("add.l d0,($0080).w"));
    try std.testing.expectEqual(350010, Test.rd(&runner.interface, 0x80, u32));

    // 3) Test that the <instruction> produces correct flag side-effects
    runner = Test.init(&.{0xD001});
    runner.cpu.d[0] = 0x70;
    runner.cpu.d[1] = 0x40;
    try std.testing.expectEqual(4, try runner.run("add.b d1,d0"));
    try std.testing.expectEqual(0xB0, runner.cpu.d[0]);
    try std.testing.expectEqual(false, runner.cpu.sr.c);
    try std.testing.expectEqual(true, runner.cpu.sr.v);
    try std.testing.expectEqual(false, runner.cpu.sr.z);
    try std.testing.expectEqual(true, runner.cpu.sr.n);
    try std.testing.expectEqual(false, runner.cpu.sr.x);

    // 4) Test that the <instruction> produces correct flag side-effects
    runner = Test.init(&.{0xD001});
    runner.cpu.d[0] = 0x10;
    runner.cpu.d[1] = 0xF0;
    try std.testing.expectEqual(4, try runner.run("add.b d1,d0"));
    try std.testing.expectEqual(0x00, runner.cpu.d[0]);
    try std.testing.expectEqual(true, runner.cpu.sr.c);
    try std.testing.expectEqual(false, runner.cpu.sr.v);
    try std.testing.expectEqual(true, runner.cpu.sr.z);
    try std.testing.expectEqual(false, runner.cpu.sr.n);
    try std.testing.expectEqual(true, runner.cpu.sr.x);
}

test "nop" {
    var runner: Test = undefined;

    // 1) Test that the <instruction> is encoded correctly
    runner = Test.init(&.{0x4e71});
    try std.testing.expectEqual(4, try runner.run("nop"));
}

test "ori #imm,ccr; ori #imm,sr; ori #imm,ea" {
    var runner: Test = undefined;

    // 1) Test that the <instruction> is encoded correctly, and has correct side effects
    runner = Test.init(&.{ 0x003c, 0x007f });
    try std.testing.expectEqual(20, try runner.run("ori.b #$7f,ccr"));
    try std.testing.expectEqual(true, runner.cpu.sr.c);
    try std.testing.expectEqual(true, runner.cpu.sr.v);
    try std.testing.expectEqual(true, runner.cpu.sr.z);
    try std.testing.expectEqual(true, runner.cpu.sr.n);
    try std.testing.expectEqual(true, runner.cpu.sr.x);

    // 2) Test that the <instruction> is encoded correctly, and has correct side effects
    runner = Test.init(&.{ 0x007c, 0xff0f });
    try std.testing.expectEqual(20, try runner.run("ori.w #$ff0f,sr"));
    try std.testing.expectEqual(true, runner.cpu.sr.c);
    try std.testing.expectEqual(true, runner.cpu.sr.v);
    try std.testing.expectEqual(true, runner.cpu.sr.z);
    try std.testing.expectEqual(true, runner.cpu.sr.n);
    try std.testing.expectEqual(false, runner.cpu.sr.x);
    try std.testing.expectEqual(7, runner.cpu.sr.ipl);

    // 3) Test that the <instruction> is encoded correctly, and has correct side effects
    runner = Test.init(&.{ 0x0000, 0x00f0 });
    try std.testing.expectEqual(8, try runner.run("ori.b #$f0,d0"));
    try std.testing.expectEqual(0xf0, runner.cpu.d[0]);
    try std.testing.expectEqual(false, runner.cpu.sr.z);
    try std.testing.expectEqual(true, runner.cpu.sr.n);
}
