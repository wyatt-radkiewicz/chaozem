const std = @import("std");

cpu: *Cpu,
bus: Bus,
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
        _pad0: u2 = 0,
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
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Read byte/word/long
        rdb: *const fn (ptr: *anyopaque, addr: u32) u8,
        rdw: *const fn (ptr: *anyopaque, addr: u32) u16,
        rdl: *const fn (ptr: *anyopaque, addr: u32) u32,

        /// Write byte/word/long
        wrb: *const fn (ptr: *anyopaque, addr: u32, data: u8) void,
        wrw: *const fn (ptr: *anyopaque, addr: u32, data: u16) void,
        wrl: *const fn (ptr: *anyopaque, addr: u32, data: u32) void,
    };
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
        pub fn apply(status: *Cpu.Status, updates: anytype) void {
            var set: u5 = @intFromBool(flags.c.mask(.set)) << 0 |
                @intFromBool(flags.v.mask(.set)) << 1 |
                @intFromBool(flags.z.mask(.set)) << 2 |
                @intFromBool(flags.n.mask(.set)) << 3 |
                @intFromBool(flags.x.mask(.set)) << 4;
            var mask: u5 = @intFromBool(flags.c.mask(.overwrite)) << 0 |
                @intFromBool(flags.v.mask(.overwrite)) << 1 |
                @intFromBool(flags.z.mask(.overwrite)) << 2 |
                @intFromBool(flags.n.mask(.overwrite)) << 3 |
                @intFromBool(flags.x.mask(.overwrite)) << 4;
            inline for (std.meta.fieldNames(updates)) |flag| {
                switch (@field(flags, flag)) {
                    .set => set |= @as(u5, @intFromBool(@field(updates, flag))) <<
                        @intFromEnum(@field(FlagName, flag)),
                    .clr => mask |= @as(u5, @intFromBool(!@field(updates, flag))) <<
                        @intFromEnum(@field(FlagName, flag)),
                    else => @compileError("Unneeded flag given"),
                }
            }

            status.* = @bitCast(@as(u16, @bitCast(status.*)) & ~@as(u16, mask) | set);

            inline for (std.meta.fieldNames(flags)) |flag| {
                switch (@field(flags, flag)) {
                    .c, .v, .z, .n, .x => |copy| {
                        const bits: u16 = @bitCast(status.*);
                        const src = @intFromEnum(@field(FlagName, @tagName(copy)));
                        const dst = @intFromEnum(@field(FlagName, flag));
                        status.* = bits & ~@as(u16, 1 << dst) |
                            (@as(u16, @as(u1, @truncate(bits >> src))) << dst);
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

/// A M68K instruction definition
const Instr = struct {
    /// Name of the instruction
    name: []const u8,
    /// Opcode's encoding
    enc: Opcode,
    /// What operand to evaluate first
    eval_first: enum { src, dst } = .src,
    /// Source operand
    src: type = void,
    /// Destination operand
    dst: type = void,
    /// What operation to perform
    op: type = void,
    /// The size(s) of the instruction
    size: ?Size.Enc = null,

    /// Instruction specialization
    const Spec = struct {
        enc: Opcode,
        run: *const fn (u16, *Exec) void,
        disasm: *const fn (*std.io.Writer, *std.io.Reader) anyerror!void,

        fn init(comptime instr: Instr, comptime size: ?Size) @This() {
            const size_mask: u16 = if (instr.size) |enc| switch (enc) {
                .fixed => 0,
                .dyn => |x| @as(u16, (1 << @bitSizeOf(std.math.IntFittingRange(0, @max(
                    x.b orelse 0,
                    x.w orelse 0,
                    x.l orelse 0,
                )))) - 1) << x.at,
            } else 0;
            const spec_opcode = Opcode{
                .set = instr.enc.set & ~size_mask | if (instr.size) |enc| switch (enc) {
                    .fixed => 0,
                    .dyn => |x| @field(x, @tagName(size orelse unreachable)) << x.at,
                } else 0,
                .any = instr.enc.any & ~size_mask,
            };
            return .{
                .opcode = spec_opcode,
                .run = struct {
                    pub fn run(opcode: u16, exec: *Exec) void {
                        var operands: struct {
                            src: instr.src,
                            dst: instr.dst,
                        } = undefined;
                        var sources: struct {
                            src: switch (instr.src) {
                                void => void,
                                else => @typeInfo(instr.src.load).@"fn".return_type orelse void,
                            },
                            dst: switch (instr.dst) {
                                void => void,
                                else => @typeInfo(instr.dst.load).@"fn".return_type orelse void,
                            },
                        } = undefined;
                        inline for (switch (instr.eval_first) {
                            .src => &.{ "src", "dst" },
                            .dst => &.{ "dst", "src" },
                        }) |field| {
                            if (@field(instr, field) != void) {
                                @field(operands, field) = .init(exec, size, opcode);
                                @field(sources, field) = @field(operands, field)
                                    .load(exec, size, opcode);
                            }
                        }
                        const result = if (instr.op != void)
                            instr.op.op(exec, size, sources.src, sources.dst)
                        else if (@TypeOf(sources.src) != void) sources.src else sources.dst;
                        if (instr.dst != void) {
                            operands.dst.store(exec, size, opcode, result);
                        }
                    }
                }.run,
                .disasm = struct {
                    fn disasm(writer: *std.io.Writer, reader: *std.io.Reader) !void {
                        const opcode = try reader.takeInt(u16, .big);
                        if (spec_opcode.match(opcode)) {
                            try writer.write(instr.name);
                        } else {
                            return error.InvalidEncoding;
                        }
                        if (size) |s| switch (s) {
                            .fixed => {},
                            .dyn => try writer.print(".{s}", .{@tagName(s)}),
                        };
                        switch (@as(u2, @intFromBool(instr.src != void)) |
                            @as(u2, @intFromBool(instr.dst != void)) << 1) {
                            0b00 => {},
                            0b01 => try writer.print(" {f}", .{instr.src.Disasm(size){
                                .reader = reader,
                                .opcode = opcode,
                            }}),
                            0b01 => try writer.print(" {f}", .{instr.dst.Disasm(size){
                                .reader = reader,
                                .opcode = opcode,
                            }}),
                            0b11 => try writer.print(" {f},{f}", .{ instr.src.Disasm(size){
                                .reader = reader,
                                .opcode = opcode,
                            }, instr.dst.Disasm(size){
                                .reader = reader,
                                .opcode = opcode,
                            } }),
                        }
                    }
                }.disasm,
            };
        }
    };

    /// Get all instruction permutations for this
    fn specializations(comptime this: @This()) []const Spec {
        var buffer: [std.meta.fields(Size).len]Spec = undefined;
        var perms = std.ArrayList(Spec).initBuffer(&buffer);
        switch (this.size) {
            .fixed => |size| perms.appendAssumeCapacity(.init(this, size)),
            .dyn => |x| for (std.meta.fieldNames(x)) |field| {
                if (@FieldType(@TypeOf(x), field) == ?comptime_int) {
                    perms.appendAssumeCapacity(.init(this, @field(Size, field)));
                }
            },
        }
        const final = buffer;
        return final[0..perms.items.len];
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
            at: comptime_int = null,
            b: ?comptime_int = null,
            w: ?comptime_int = null,
            l: ?comptime_int = null,
        },

        /// Decodes a size, and returns null if there was no encoding for the bits given
        fn decode(comptime this: @This(), opcode: u16) ?Size {
            return switch (this) {
                .fixed => |size| size,
                .dyn => |enc| blk: {
                    const highest_encoding = @max(enc.b orelse 0, enc.w orelse 0, enc.l orelse 0);
                    const map = comptime build: {
                        var map = [1]?Size{null} ** (highest_encoding + 1);
                        for (std.meta.fieldNames(@TypeOf(enc))) |field| {
                            if (@FieldType(enc, field) == ?comptime_int) {
                                map[@field(enc, field) orelse continue] = @field(Size, field);
                            }
                        }
                        break :build map;
                    };
                    break :blk map[
                        extract(std.meta.Int(
                            .unsigned,
                            std.math.ceilPowerOfTwo(comptime_int, highest_encoding),
                        ), opcode, enc.at)
                    ];
                },
            };
        }

        /// Encodes a size and returns the set bits
        fn encode(comptime this: @This(), size: Size) u16 {
            return switch (this) {
                .fixed => 0,
                .dyn => |enc| switch (size) {
                    inline else => |s| @field(enc, @tagName(s)) orelse 0 << enc.at,
                },
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
        return opcode ^ this.set & ~this.any;
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
};

/// Data register source or destination target
fn DataTarget(n: u4) type {
    return struct {
        n: u3,

        fn init(_: *Exec, comptime _: ?Size, opcode: u16) @This() {
            return .{ .n = extract(u3, opcode, n) };
        }

        fn load(this: @This(), exec: *Exec, comptime size: Size, _: u16) size.Int(.unsigned) {
            return @truncate(exec.cpu.d[this.n]);
        }

        fn store(
            this: @This(),
            exec: *Exec,
            comptime size: ?Size,
            _: u16,
            data: size.Int(.unsigned),
        ) void {
            exec.cpu.d[this.n] = overwrite(exec.cpu.d[this.n], data);
        }

        fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                fn format(this: @This(), writer: *std.io.Writer) !void {
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

        fn init(_: *Exec, comptime _: Size, opcode: u16) @This() {
            return .{ .n = extract(u3, opcode, n) };
        }

        fn load(this: @This(), exec: *Exec, comptime size: Size, _: u16) size.Int(.unsigned) {
            return @truncate(exec.cpu.a[this.n]);
        }

        fn store(
            this: @This(),
            exec: *Exec,
            comptime size: Size,
            _: u16,
            data: size.Int(.unsigned),
        ) void {
            exec.cpu.a[this.n] = extend(u32, data);
        }

        fn Disasm(comptime _: Size) type {
            return struct {
                reader: *std.io.Reader,
                opcode: u16,

                fn format(this: @This(), writer: *std.io.Writer) !void {
                    try writer.print("a{}", .{extract(u3, this.opcode, n)});
                }
            };
        }
    };
}

/// Effective address source or destination target
fn EaTarget(m: u4, n: u4) type {
    const Index = packed struct {
        disp: i8,
        _pad0: u3,
        size: u1,
        n: u3,
        m: u1,

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

    return struct {
        mode: Mode,
        addr: u32,

        fn init(exec: *Exec, comptime size: Size, opcode: u16) @This() {
            const reg = extract(u3, opcode, n);
            const mode = Mode.decode(extract(u3, opcode, m), reg);
            const addr = switch (mode) {
                .data_reg, .addr_reg => reg,
                .iregdirect => exec.cpu.a[reg],
                .post_inc => post_inc: {
                    const addr = exec.cpu.a[reg];
                    exec.cpu.a[reg] += @sizeOf(size);
                    break :post_inc addr;
                },
                .pre_dec => pre_dec: {
                    exec.cpu.a[reg] -= @sizeOf(size);
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
            return .{ .mode = mode, .addr = addr };
        }

        fn load(this: @This(), exec: *Exec, comptime size: Size, _: u16) size.Int(.unsigned) {
            return switch (this.mode) {
                .data_reg => @truncate(exec.cpu.d[this.addr]),
                .addr_reg => @truncate(exec.cpu.a[this.addr]),
                .immediate => exec.fetch(size.Int(.unsigned)),
                else => exec.read(size.Int(.unsigned), this.addr),
            };
        }

        fn store(
            this: @This(),
            exec: *Exec,
            comptime size: Size,
            _: u16,
            data: size.Int(.unsigned),
        ) void {
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

                fn fetch(this: @This(), comptime Data: type) Data {
                    const Fetch = std.meta.int(.unsigned, @max(16, @bitSizeOf(Data)));
                    const fetched = try this.reader.takeInt(Fetch, .big);
                    return @bitCast(@as(
                        std.meta.int(.unsigned, @bitSizeOf(Data)),
                        @truncate(fetched),
                    ));
                }

                fn format(this: @This(), writer: *std.io.Writer) !void {
                    const reg = extract(u3, this.opcode, n);
                    switch (Mode.decode(extract(u3, this.opcode, m), reg)) {
                        .data_reg => try writer.print("d{}", .{reg}),
                        .addr_reg => try writer.print("a{}", .{reg}),
                        .iregdirect => try writer.print("(a{})", .{reg}),
                        .post_inc => try writer.print("(a{})+", .{reg}),
                        .pre_dec => try writer.print("-(a{})", .{reg}),
                        .addr_disp => try writer.print("({}, a{})", .{ this.fetch(i16), reg }),
                        .addr_idx, .pc_idx => |mode| {
                            const idx = this.fetch(Index);
                            try writer.print("({}, ", .{idx.disp});
                            switch (mode) {
                                .addr_idx => try writer.print("a{}", .{reg}),
                                .pc_idx => try writer.print("pc", .{}),
                                else => unreachable,
                            }
                            try writer.print(", {c}{}.{c})", .{ switch (idx.m) {
                                0 => 'd',
                                1 => 'a',
                            }, idx.n, switch (idx.size) {
                                0 => 'w',
                                1 => 'l',
                            } });
                        },
                        .abs_word => try writer.print("{x:0>4}.w", .{this.fetch(u16)}),
                        .abs_long => try writer.print("{x:0>8}.l", .{this.fetch(u32)}),
                        .pc_disp => try writer.print("({}, pc)", .{this.fetch(i16)}),
                        .immediate => switch (size) {
                            .b => try writer.print("#{x:0>2}", .{this.fetch(u8)}),
                            .w => try writer.print("#{x:0>4}", .{this.fetch(u16)}),
                            .l => try writer.print("#{x:0>8}", .{this.fetch(u32)}),
                        },
                    }
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
        }).apply(&exec.cpu.status, .{
            .n = negative(sum),
            .z = sum == 0,
            .v = overflow,
            .c = carry,
        });
        return sum;
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
) fn (Index) ret_type: {
    var max = 0;
    for (0..1 << @bitSizeOf(Index)) |i| {
        max = @max(max, getter(context, i));
    }
    break :ret_type ?std.math.IntFittingRange(0, max);
} {
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
                null_entry.* = @max(null_entry.*, entry orelse return entry + 1);
                return entry;
            } else {
                var node = @This(){};
                for (&node.entries, 0..1 << level_size) |*entry, i| {
                    entry.* = visit(nodes, @as(
                        std.meta.Int(.unsigned, prefix_len + 4),
                        if (prefix_len == 0) 0 else prefix,
                    ) << 4 + i);
                }
                return node.add(nodes);
            }
        }

        fn add(comptime this: @This(), comptime nodes: *std.ArrayList(@This())) usize {
            return for (nodes.items, 0..) |node, idx| {
                if (std.mem.eql(usize, this.entries, node.entries)) {
                    break idx;
                }
            } else add: {
                nodes.appendAssumeCapacity(this);
                break :add nodes.items.len - 1;
            };
        }
    };

    var node_buffer: [index_size / level_size]Node = undefined;
    var nodes = std.ArrayList(Node).initBuffer(&node_buffer);
    var null_entry = 0;

    const top_level = Node.visit(&nodes, &null_entry, void) orelse unreachable;
    const Entry = std.math.IntFittingRange(0, @max(nodes.items.len - 1, null_entry));

    var compressed: [nodes.items.len << level_size]Entry = undefined;
    for (&compressed, 0..) |*entry, i| {
        entry.* = nodes.items[i >> level_size].entries[
            i & ~@as(
                usize,
                1 << level_size,
            )
        ] orelse null_entry;
    }
    const final = compressed;
    const RetType = ?std.math.IntFittingRange(0, null_entry - 1);
    const null_return = null_entry;

    return struct {
        pub fn lut(index: Index) RetType {
            const first_level = index_size - level_size;
            var entry = final[(top_level << level_size) + (index >> first_level)];
            inline for (1..index_size / level_size) |level| {
                const nibble: u4 = @truncate(index >> first_level - level * level_size);
                entry = final[(entry << level_size) + nibble];
            }
            return if (entry == null_return) null else @intCast(entry);
        }
    }.lut;
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
        pub fn disasm(opcode: u16) ?*const fn (*std.io.Writer, *std.io.Reader) anyerror!void {
            return matcher.perms[lut(opcode) orelse return null].disasm;
        }
    };
}

/// Fetch a type from the program counter of the cpu
fn fetch(this: *Exec, comptime Data: type) Data {
    const fetch_width = @max(16, @bitSizeOf(Data));
    const data = this.read(std.meta.Int(.unsigned, fetch_width), this.cpu.pc);
    this.cpu.pc += fetch_width / 8;
    return @bitCast(@as(this.meta.Int(.unsigned, @bitSizeOf(Data)), @truncate(data)));
}

/// Read data type X from the bus
fn read(this: *Exec, comptime Data: type, addr: u32) Data {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            return @bitCast(this.bus.vtable.rdb(this.bus.ptr, addr, addr));
        },
        16 => {
            this.clk += 4;
            return @bitCast(this.bus.vtable.rdw(this.bus.ptr, addr, addr));
        },
        32 => {
            this.clk += 8;
            return @bitCast(this.bus.vtable.rdl(this.bus.ptr, addr, addr));
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to read data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Write data type X to the bus
fn write(this: *Exec, comptime Data: type, addr: u32, data: Data) void {
    switch (@bitSizeOf(Data)) {
        8 => {
            this.clk += 4;
            this.bus.vtable.wrb(this.bus.ptr, addr, @bitCast(data));
        },
        16 => {
            this.clk += 4;
            this.bus.vtable.wrw(this.bus.ptr, addr, @bitCast(data));
        },
        32 => {
            this.clk += 8;
            this.bus.vtable.wrl(this.bus.ptr, addr, @bitCast(data));
        },
        else => @compileError(std.fmt.comptimePrint(
            "Tried to write data of width {}!",
            .{@bitSizeOf(Data)},
        )),
    }
}

/// Extract a type from an integer at a position
fn extract(comptime Type: type, from: anytype, at: std.math.Log2Int(@TypeOf(from))) Type {
    const TypeInt = std.meta.Int(.unsigned, @bitSizeOf(Type));
    const FromInt = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(from)));
    return @bitCast(@as(TypeInt, @truncate(@as(FromInt, @bitCast(from)) >> at)));
}

/// Sign extend a specified integer to the specified size
fn extend(comptime To: type, from: anytype) To {
    const ToSigned = std.meta.Int(.signed, @bitSizeOf(To));
    const FromSigned = std.meta.Int(.signed, @bitSizeOf(@TypeOf(from)));
    return @bitCast(@as(ToSigned, @as(FromSigned, @bitCast(from))));
}

/// Overwrite the lower N bits with another integer
fn overwrite(int: anytype, with: anytype) @TypeOf(int) {
    const Int = std.meta.int(.unsigned, @bitSizeOf(@TypeOf(int)));
    const mask: @TypeOf(Int) = (1 << @bitSizeOf(@TypeOf(with))) - 1;
    return int & ~mask | with;
}

/// Returns true if the int has the highest bit set
fn negative(int: anytype) bool {
    return @as(std.meta.Int(.signed, @bitSizeOf(@TypeOf(int))), @bitCast(int)) < 0;
}
