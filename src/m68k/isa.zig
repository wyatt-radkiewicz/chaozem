const std = @import("std");

const page = @import("page");

const Ctx = @import("Ctx.zig");

/// A M68K instruction definition
pub const Instr = struct {
    /// Name of the instruction
    name: []const u8,
    /// Opcode's encoding
    enc: Opcode,
    /// A third source for the instruction, but not the destination
    ctx: ?type = null,
    /// Source of the instruction
    src: ?type = null,
    /// Destination of the instruction
    dst: ?type = null,
    /// What operation to perform
    op: ?type = null,
    /// The size(s) of the instruction
    size: Ctx.Size.Enc = .{ .fixed = .none },
    /// Any extra cycles to add to the instruction execution
    clk: usize = 0,
    /// This is how the disassembly should be output
    disasm: ?[]const Token = null,

    /// Get all instruction permutations for this
    fn getSizes(comptime this: @This()) []const Spec {
        var buffer: [std.meta.fields(Ctx.Size).len]Spec = undefined;
        var perms = std.ArrayList(Spec).initBuffer(&buffer);
        switch (this.size) {
            .fixed => |size| perms.appendAssumeCapacity(.init(this, size)),
            .dyn => |x| for (std.meta.fieldNames(@TypeOf(x))) |field| {
                if (@FieldType(@TypeOf(x), field) == ?comptime_int and
                    @field(x, field) != null)
                {
                    perms.appendAssumeCapacity(.init(this, @field(Ctx.Size, field)));
                }
            },
        }
        const final = buffer;
        return final[0..perms.items.len];
    }

    /// Instruction specialization
    const Spec = struct {
        enc: Opcode,
        run: *const fn (*Ctx, u16) void,
        disasm: *const fn (*std.io.Writer, *std.io.Reader, u16) std.io.Writer.Error!void,

        fn init(comptime instr: Instr, comptime size: Ctx.Size) @This() {
            const enc = instr.enc.overwrite(switch (instr.size) {
                .fixed => 0,
                .dyn => |dyn| dyn.at,
            }, instr.size.encode(size));
            return .{
                .enc = enc,
                .run = struct {
                    pub fn run(ctx: *Ctx, opcode: u16) void {
                        ctx.clk += instr.clk;
                        switch (@as(u3, @intFromBool(instr.src != null)) << 2 |
                            @as(u3, @intFromBool(instr.dst != null)) << 1 |
                            @as(u3, @intFromBool(instr.ctx != null))) {
                            0b000 => {
                                const Op = instr.op orelse return;
                                _ = Op.op(ctx, size);
                            },
                            0b100 => {
                                const Src = instr.src orelse unreachable;
                                const Op = instr.op orelse return;
                                _ = Op.op(ctx, size, Src.decode(ctx, size, opcode)
                                    .load(ctx, size, opcode));
                            },
                            0b010 => {
                                const Dst = instr.dst orelse unreachable;
                                const dst = Dst.decode(ctx, size, opcode);
                                const res = if (instr.op) |Op|
                                    Op.op(ctx, size, dst.load(ctx, size, opcode))
                                else
                                    dst.load(ctx, size, opcode);
                                if (@TypeOf(res) != void) {
                                    dst.store(ctx, size, opcode, res);
                                }
                            },
                            0b110 => {
                                const Src = instr.src orelse unreachable;
                                const Dst = instr.dst orelse unreachable;
                                const src_target = Src.decode(ctx, size, opcode);
                                const src_data = src_target.load(ctx, size, opcode);
                                const dst_target = Dst.decode(ctx, size, opcode);
                                const res = if (instr.op) |Op|
                                    Op.op(ctx, size, src_data, dst_target.load(ctx, size, opcode))
                                else
                                    src_data;
                                if (@TypeOf(res) != void) {
                                    dst_target.store(ctx, size, opcode, res);
                                }
                            },
                            0b111 => {
                                const Src = instr.src orelse unreachable;
                                const Dst = instr.dst orelse unreachable;
                                const ctx_target = (instr.ctx orelse unreachable)
                                    .decode(ctx, size, opcode);
                                const src_target = Src.decode(ctx, size, opcode);
                                const src_data = src_target.load(ctx, size, opcode);
                                const dst_target = Dst.decode(ctx, size, opcode);
                                const res = if (instr.op) |Op|
                                    Op.op(
                                        ctx,
                                        size,
                                        ctx_target.load(ctx, size, opcode),
                                        src_data,
                                        dst_target.load(ctx, size, opcode),
                                    )
                                else
                                    src_data;
                                if (@TypeOf(res) != void) {
                                    dst_target.store(ctx, size, opcode, res);
                                }
                            },
                            else => @compileError("Operand configuration unsupported"),
                        }
                    }
                }.run,
                .disasm = struct {
                    fn disasm(
                        writer: *std.io.Writer,
                        reader: *std.io.Reader,
                        opcode: u16,
                    ) std.io.Writer.Error!void {
                        inline for (comptime instr.disasm orelse Token.default(instr)) |token| {
                            try token.disasm(instr, size, writer, reader, opcode);
                        }
                    }
                }.disasm,
            };
        }
    };

    /// Tokens that can be found in the instruction disassembly
    pub const Token = enum {
        name,
        size,
        src,
        dst,
        ctx,
        comma,
        space,

        /// Make a default token list based on an instruction
        fn default(comptime instr: Instr) []const @This() {
            return switch (@as(u3, @intFromBool(instr.src != null)) << 2 |
                @as(u3, @intFromBool(instr.dst != null)) << 1 |
                @as(u3, @intFromBool(instr.ctx != null))) {
                0b000 => &[_]Token{ .name, .size },
                0b100 => &[_]Token{ .name, .size, .space, .src },
                0b010 => &[_]Token{ .name, .size, .space, .dst },
                0b110 => &[_]Token{ .name, .size, .space, .src, .comma, .dst },
                else => @compileError("Default operand configuration unsupported"),
            };
        }

        /// Disassemble a token
        fn disasm(
            comptime this: @This(),
            comptime instr: Instr,
            comptime size: Ctx.Size,
            writer: *std.io.Writer,
            reader: *std.io.Reader,
            opcode: u16,
        ) std.io.Writer.Error!void {
            switch (this) {
                .space => try writer.print(" ", .{}),
                .comma => try writer.print(",", .{}),
                .src => {
                    const Src = instr.src orelse unreachable;
                    try Src.disasm(size, writer, reader, opcode);
                },
                .dst => {
                    const Dst = instr.dst orelse unreachable;
                    try Dst.disasm(size, writer, reader, opcode);
                },
                .ctx => {
                    const Opnd = instr.ctx orelse unreachable;
                    try Opnd.disasm(size, writer, reader, opcode);
                },
                .size => switch (size) {
                    .none => {},
                    else => try writer.print(".{s}", .{@tagName(size)}),
                },
                .name => try writer.print("{s}", .{instr.name}),
            }
        }
    };
};

/// Matches against ranges of opcodes
const Opcode = struct {
    set: u16,
    any: u16,

    /// Match '0','1', or 'x'
    pub fn init(enc: *const [16]u8) @This() {
        @setEvalBranchQuota(10000);

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

    /// Checks if the opcode bits provided match the pattern of this opcode
    fn match(this: @This(), opcode: u16) bool {
        return opcode ^ this.set & ~this.any == 0;
    }

    /// Overwrite some bits at a specific position
    fn overwrite(this: @This(), at: u4, with: anytype) @This() {
        const bits = switch (@TypeOf(with)) {
            comptime_int => @as(std.math.IntFittingRange(0, with), with),
            void => return this,
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
        // Add each instruction specialization
        var perms: [512]Instr.Spec = undefined;
        var perms_builder = std.ArrayList(Instr.Spec).initBuffer(&perms);
        for (instrs) |instr| {
            perms_builder.appendSliceAssumeCapacity(instr.getSizes());
        }

        // Then sort by the specificity of each opcode
        std.sort.pdq(Instr.Spec, perms_builder.items, {}, struct {
            fn lessThanFn(_: void, lhs: Instr.Spec, rhs: Instr.Spec) bool {
                return @popCount(lhs.enc.any) < @popCount(rhs.enc.any);
            }
        }.lessThanFn);

        // Finalize variables and return
        const final = perms;
        return .{ .perms = final[0..perms_builder.items.len] };
    }

    /// Get the index of the permutation that was matched
    fn match(comptime this: @This(), opcode: u16) ?usize {
        return for (0..this.perms.len) |idx| {
            if (this.perms[idx].enc.match(opcode)) {
                break idx;
            }
        } else null;
    }
};

/// Instruction set architecture faciltates runtime running of instructions and disassembling
pub fn Isa(comptime instrs: []const Instr) type {
    // Set eval quota
    @setEvalBranchQuota(256 * 100000 + (1 << 16) * 1000);

    // Generate the matcher
    const matcher = Matcher.init(instrs);

    // Generate the look up table
    const lut = page.table(.{
        .Index = u16,
        .Entry = ?usize,
        .max_pages = 1 << 16,
        .page_size = 4,
    }, struct {
        matcher: Matcher,

        pub fn get(this: @This(), index: u16) ?usize {
            return this.matcher.match(index);
        }
    }{ .matcher = matcher });

    // Create the runtime function pointers to index the look up table
    return struct {
        /// Gets the instruction handler for the opcode
        pub fn runner(opcode: u16) ?*const fn (*Ctx, u16) void {
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
