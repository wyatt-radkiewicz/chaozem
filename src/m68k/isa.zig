const std = @import("std");

const Exec = @import("Exec.zig"); 

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
    /// Any extra cycles to add to the instruction execution
    clk: usize = 0,

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
                        exec.clk += instr.clk;
                        if (instr.src) |Src| {
                            const SrcOp = Operand(Src, size);
                            if (instr.dst) |Dst| {
                                const DstOp = Operand(Dst, size);
                                const src = SrcOp.init(exec, opcode);
                                var dst = DstOp.init(exec, opcode);
                                const result = runop(instr.op, DstOp.Data, size, exec, .{
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
                        const instr_src: ?type = if (instr.src) |t|
                            (if (@hasDecl(t, "Disasm")) t else null)
                        else
                            null;
                        const instr_dst: ?type = if (instr.dst) |t|
                            (if (@hasDecl(t, "Disasm")) t else null)
                        else
                            null;
                        if (instr_src) |src| {
                            if (instr_dst) |dst| {
                                try writer.print(" {f},{f}", .{
                                    initdisasm(size, src, reader, opcode),
                                    initdisasm(size, dst, reader, opcode),
                                });
                            } else {
                                try writer.print(" {f}", .{initdisasm(size, src, reader, opcode)});
                            }
                        } else if (instr_dst) |dst| {
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
    var max = 0;
    for (0..1 << @bitSizeOf(Index)) |i| {
        max = @max(max, getter(context, i) orelse 0);
    }
    return ?std.math.IntFittingRange(0, max);
}

/// Instruction set architecture faciltates runtime running of instructions and disassembling
fn Isa(comptime instrs: []const Instr) type {
    @setEvalBranchQuota(256 * 100000 + (1 << 16) * 1000);
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