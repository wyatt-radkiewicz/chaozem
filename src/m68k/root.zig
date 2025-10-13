const std = @import("std");

const bus_interface = @import("bus");

const arg = @import("arg.zig");
pub const Cpu = @import("Cpu.zig");
const Ctx = @import("Ctx.zig");
/// M68k vectors
pub const Vector = Ctx.Vector;
/// M68k processor width
pub const bus_width = Ctx.bus_width;
const isa = @import("isa.zig");
const op = @import("op.zig");

const Bus = bus_interface.Bus(Ctx.bus_width);

/// Process an exception
pub fn exception(vector: Ctx.Vector, cpu: *Cpu, bus: *Bus) usize {
    var ctx = Ctx{ .cpu = cpu, .bus = bus };
    vector.handle(&ctx);
    return ctx.clk;
}

/// Run one instruction
pub fn step(cpu: *Cpu, bus: *Bus) usize {
    // If the cpu is stopped, don't emulate anything
    if (cpu.stop) {
        return 0;
    }

    // Setup the execution context
    var ctx = Ctx{ .cpu = cpu, .bus = bus };

    // Fetch the opcode and either run the instruction, or illegal handler
    const opcode = ctx.fetch(u16);
    if (m68k_isa.runner(opcode)) |pfn| {
        pfn(&ctx, opcode);
    } else {
        Ctx.Vector.illegal.handle(&ctx);
    }
    return ctx.clk;
}

/// Disassemble one instruction
pub const Disasm = struct {
    reader: *std.io.Reader,

    pub fn format(this: @This(), writer: *std.io.Writer) std.io.Writer.Error!void {
        const opcode = this.reader.takeInt(u16, .big) catch return error.WriteFailed;
        if (m68k_isa.disasm(opcode)) |pfn| {
            try pfn(writer, this.reader, opcode);
        } else {
            try writer.print("<invalid opcode>", .{});
        }
    }
};

/// M68k instruction set architecture
const m68k_isa = isa.Isa(&.{
    // Add binary coded decimal
    isa.Instr{
        .name = "abcd",
        .enc = .init("1100xxx10000xxxx"),
        .src = arg.RegReg(3, 0, .{}),
        .dst = arg.RegReg(3, 9, .{ .b = .{ 2, 2 } }),
        .op = op.Abcd,
        .size = .{ .fixed = .b },
    },

    // Add
    isa.Instr{
        .name = "add",
        .enc = .init("1101xxx0xxxxxxxx"),
        .src = arg.Ea(3, 0, .{ .l = .initDefault(2, .{
            .data_reg = 4,
            .addr_reg = 4,
            .immediate = 4,
        }) }),
        .dst = arg.DataReg(9),
        .op = op.Add,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "add",
        .enc = .init("1101xxx1xxxxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Add,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Add address
    isa.Instr{
        .name = "adda",
        .enc = .init("1101xxxx11xxxxxx"),
        .src = arg.Ea(3, 0, .{
            .b = .initFill(4),
            .w = .initFill(4),
            .l = .initDefault(2, .{
                .data_reg = 4,
                .addr_reg = 4,
                .immediate = 4,
            }),
        }),
        .dst = arg.AddrReg(9),
        .op = op.Adda,
        .size = .{ .dyn = .{ .at = 8, .w = 0, .l = 1 } },
    },

    // Add immediate
    isa.Instr{
        .name = "addi",
        .enc = .init("00000110xxxxxxxx"),
        .src = arg.Imm(.{}),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = op.Add,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Add quick
    isa.Instr{
        .name = "addq",
        .enc = .init("0101xxx0xxxxxxxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = op.Add,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "addq",
        .enc = .init("0101xxx0xx001xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.AddrReg(0),
        .op = op.Adda,
        .size = .{ .dyn = .{ .at = 6, .w = 0b01, .l = 0b10 } },
        .clk = 4,
    },
    isa.Instr{
        .name = "addq",
        .enc = .init("01010000xx001xxx"),
        .src = arg.Const(u4, 8),
        .dst = arg.AddrReg(0),
        .op = op.Adda,
        .size = .{ .dyn = .{ .at = 6, .w = 0b01, .l = 0b10 } },
        .clk = 4,
    },

    // Add extend
    isa.Instr{
        .name = "addx",
        .enc = .init("1101xxx1xx00xxxx"),
        .src = arg.RegReg(3, 0, .{}),
        .dst = arg.RegReg(3, 9, .{ .b = .{ 0, 2 }, .l = .{ 4, 2 } }),
        .op = op.Addx,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Logical and
    isa.Instr{
        .name = "and",
        .enc = .init("1100xxx0xxxxxxxx"),
        .src = arg.Ea(3, 0, .{ .l = .initDefault(2, .{
            .data_reg = 4,
            .immediate = 4,
        }) }),
        .dst = arg.DataReg(9),
        .op = op.Logic(.@"and"),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "and",
        .enc = .init("1100xxx1xxxxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Logic(.@"and"),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Logical and immediate
    isa.Instr{
        .name = "andi",
        .enc = .init("00000010xx111100"),
        .src = arg.Imm(.{}),
        .dst = arg.Status,
        .op = op.Logic(.@"and"),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01 } },
        .clk = 12,
    },
    isa.Instr{
        .name = "andi",
        .enc = .init("00000010xxxxxxxx"),
        .src = arg.Imm(.{}),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = op.Logic(.@"and"),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Arithmatic shift left
    isa.Instr{
        .name = "asl",
        .enc = .init("1110000111xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Shift(false, .al),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110xxx1xx000xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Shift(true, .al),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110xxx1xx100xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Shift(true, .al),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Arithmatic shift right
    isa.Instr{
        .name = "asr",
        .enc = .init("1110000011xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Shift(false, .ar),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "asr",
        .enc = .init("1110xxx0xx000xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Shift(true, .ar),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "asr",
        .enc = .init("1110xxx0xx100xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Shift(true, .ar),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Branch on condition
    isa.Instr{
        .name = "b",
        .enc = .init("0110xxxxxxxxxxxx"),
        .src = arg.Opcode(Ctx.Cond, 8),
        .dst = arg.Opcode(u8, 0),
        .op = op.Branch,
        .size = .{ .fixed = .b },
        .disasm = &.{ .name, .src, .size, .space, .dst },
    },
    isa.Instr{
        .name = "b",
        .enc = .init("0110xxxx00000000"),
        .src = arg.Opcode(Ctx.Cond, 8),
        .dst = arg.Imm(.{ .hex = false }),
        .op = op.Branch,
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .src, .size, .space, .dst },
    },

    // Test and change a bit
    isa.Instr{
        .name = "bchg",
        .enc = .init("0000100001xxxxxx"),
        .src = arg.BitIdx,
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.chg),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "bchg",
        .enc = .init("0000100001000xxx"),
        .src = arg.BitIdx,
        .dst = arg.DataReg(0),
        .op = op.Bit(.chg),
        .size = .{ .fixed = .l },
        .clk = 2,
    },
    isa.Instr{
        .name = "bchg",
        .enc = .init("0000xxx101xxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.chg),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "bchg",
        .enc = .init("0000xxx101000xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Bit(.chg),
        .size = .{ .fixed = .l },
        .clk = 2,
    },

    // Test and clear a bit
    isa.Instr{
        .name = "bclr",
        .enc = .init("0000100010xxxxxx"),
        .src = arg.BitIdx,
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.clr),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "bclr",
        .enc = .init("0000100010000xxx"),
        .src = arg.BitIdx,
        .dst = arg.DataReg(0),
        .op = op.Bit(.clr),
        .size = .{ .fixed = .l },
        .clk = 4,
    },
    isa.Instr{
        .name = "bclr",
        .enc = .init("0000xxx110xxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.clr),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "bclr",
        .enc = .init("0000xxx110000xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Bit(.clr),
        .size = .{ .fixed = .l },
        .clk = 4,
    },

    // Branch always
    isa.Instr{
        .name = "bra",
        .enc = .init("01100000xxxxxxxx"),
        .src = arg.Const(Ctx.Cond, .t),
        .dst = arg.Opcode(u8, 0),
        .op = op.Branch,
        .size = .{ .fixed = .b },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "bra",
        .enc = .init("0110000000000000"),
        .src = arg.Const(Ctx.Cond, .t),
        .dst = arg.Imm(.{ .hex = false }),
        .op = op.Branch,
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },

    // Test and set a bit
    isa.Instr{
        .name = "bset",
        .enc = .init("0000100011xxxxxx"),
        .src = arg.BitIdx,
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.set),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "bset",
        .enc = .init("0000100011000xxx"),
        .src = arg.BitIdx,
        .dst = arg.DataReg(0),
        .op = op.Bit(.set),
        .size = .{ .fixed = .l },
        .clk = 2,
    },
    isa.Instr{
        .name = "bset",
        .enc = .init("0000xxx111xxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.set),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "bset",
        .enc = .init("0000xxx111000xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Bit(.set),
        .size = .{ .fixed = .l },
        .clk = 2,
    },

    // Branch to subroutine
    isa.Instr{
        .name = "bsr",
        .enc = .init("01100001xxxxxxxx"),
        .src = arg.Const(Ctx.Cond, .f),
        .dst = arg.Opcode(u8, 0),
        .op = op.Branch,
        .size = .{ .fixed = .b },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "bsr",
        .enc = .init("0110000100000000"),
        .src = arg.Const(Ctx.Cond, .f),
        .dst = arg.Imm(.{ .hex = false }),
        .op = op.Branch,
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },

    // Test a bit
    isa.Instr{
        .name = "btst",
        .enc = .init("0000100000xxxxxx"),
        .src = arg.BitIdx,
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.tst),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "btst",
        .enc = .init("0000100000000xxx"),
        .src = arg.BitIdx,
        .dst = arg.DataReg(0),
        .op = op.Bit(.tst),
        .size = .{ .fixed = .l },
        .clk = 2,
    },
    isa.Instr{
        .name = "btst",
        .enc = .init("0000xxx100xxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.tst),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "btst",
        .enc = .init("0000xxx100000xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Bit(.tst),
        .size = .{ .fixed = .l },
        .clk = 2,
    },

    // Check a signed integer against bounds
    isa.Instr{
        .name = "chk",
        .enc = .init("0100xxx110xxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.DataReg(9),
        .op = op.Chk,
        .size = .{ .fixed = .w },
    },

    // Clear data
    isa.Instr{
        .name = "clr",
        .enc = .init("01000010xxxxxxxx"),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 2 }) }),
        .op = op.Clr,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
        .disasm = &.{ .name, .size, .space, .dst },
    },

    // Compare operands
    isa.Instr{
        .name = "cmp",
        .enc = .init("1011xxx0xxxxxxxx"),
        .src = arg.Ea(3, 0, .{ .l = .initDefault(2, .{}) }),
        .dst = arg.DataReg(9),
        .op = op.Cmp,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Compare address operand to sign extended source operand
    isa.Instr{
        .name = "cmpa",
        .enc = .init("1011xxxx11xxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.AddrReg(9),
        .op = op.Cmpa,
        .size = .{ .dyn = .{ .at = 8, .w = 0, .l = 1 } },
        .clk = 2,
    },

    // Compare address operand to immiediate
    isa.Instr{
        .name = "cmpi",
        .enc = .init("00001100xxxxxxxx"),
        .src = arg.Imm(.{}),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(2, .{}) }),
        .op = op.Cmp,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Compare address operand to registers or memory
    isa.Instr{
        .name = "cmpm",
        .enc = .init("1011xxx1xx001xxx"),
        .src = arg.PostInc(0),
        .dst = arg.PostInc(9),
        .op = op.Cmp,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Compare, decrement, and branch
    isa.Instr{
        .name = "db",
        .enc = .init("0101xxxx11001xxx"),
        .ctx = arg.Opcode(Ctx.Cond, 8),
        .src = arg.Imm(.{ .hex = false }),
        .dst = arg.DataReg(0),
        .op = op.Dbcc,
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .ctx, .size, .space, .dst, .comma, .src },
    },

    // Signed divide
    isa.Instr{
        .name = "divs",
        .enc = .init("1000xxx111xxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.RegIdx(.data, 9),
        .op = op.Div(.signed),
        .size = .{ .fixed = .w },
    },

    // Unsigned divide
    isa.Instr{
        .name = "divu",
        .enc = .init("1000xxx011xxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.RegIdx(.data, 9),
        .op = op.Div(.unsigned),
        .size = .{ .fixed = .w },
    },

    // Exclusive or
    isa.Instr{
        .name = "eor",
        .enc = .init("1011xxx1xxxxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = op.Logic(.eor),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Exclusive or immediate
    isa.Instr{
        .name = "eori",
        .enc = .init("00001010xx111100"),
        .src = arg.Imm(.{}),
        .dst = arg.Status,
        .op = op.Logic(.eor),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01 } },
        .clk = 12,
    },
    isa.Instr{
        .name = "eori",
        .enc = .init("00001010xxxxxxxx"),
        .src = arg.Imm(.{}),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = op.Logic(.eor),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Exchange registers
    isa.Instr{
        .name = "exg",
        .enc = .init("1100xxx101000xxx"),
        .src = arg.RegIdx(.data, 9),
        .dst = arg.RegIdx(.data, 0),
        .op = op.Exg(.data, .data),
        .size = .{ .fixed = .l },
    },
    isa.Instr{
        .name = "exg",
        .enc = .init("1100xxx101001xxx"),
        .src = arg.RegIdx(.addr, 9),
        .dst = arg.RegIdx(.addr, 0),
        .op = op.Exg(.addr, .addr),
        .size = .{ .fixed = .l },
    },
    isa.Instr{
        .name = "exg",
        .enc = .init("1100xxx110001xxx"),
        .src = arg.RegIdx(.data, 9),
        .dst = arg.RegIdx(.addr, 0),
        .op = op.Exg(.data, .addr),
        .size = .{ .fixed = .l },
    },

    // Sign extend
    isa.Instr{
        .name = "ext",
        .enc = .init("010010001x000xxx"),
        .dst = arg.RegIdx(.data, 0),
        .op = op.Ext,
        .size = .{ .dyn = .{ .at = 6, .w = 0, .l = 1 } },
    },

    // Illegal
    isa.Instr{
        .name = "illegal",
        .enc = .init("0100101011111100"),
        .op = op.Illegal,
    },

    // Jump
    isa.Instr{
        .name = "jmp",
        .enc = .init("0100111011xxxxxx"),
        .dst = arg.Addr(3, 0, .{ .none = .initDefault(0, .{
            .indirect = 4,
            .abs_word = 2,
            .addr_disp = 2,
            .pc_disp = 2,
            .addr_idx = 4,
            .pc_idx = 4,
        }) }),
        .op = op.Jmp,
    },

    // Jump to subroutine
    isa.Instr{
        .name = "jsr",
        .enc = .init("0100111010xxxxxx"),
        .dst = arg.Addr(3, 0, .{ .none = .initDefault(0, .{
            .indirect = 4,
            .abs_word = 2,
            .addr_disp = 2,
            .pc_disp = 2,
            .addr_idx = 4,
            .pc_idx = 4,
        }) }),
        .op = op.Jsr,
    },

    // Load effective address
    isa.Instr{
        .name = "lea",
        .enc = .init("0100xxx111xxxxxx"),
        .src = arg.Addr(3, 0, .{ .none = .initDefault(0, .{ .addr_idx = 2, .pc_idx = 2 }) }),
        .dst = arg.AddrReg(9),
        .size = .{ .fixed = .l },
    },

    // Link and allocate
    isa.Instr{
        .name = "link",
        .enc = .init("0100111001010xxx"),
        .src = arg.Imm(.{ .hex = false }),
        .dst = arg.AddrReg(0),
        .op = op.Link,
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst, .comma, .src },
    },

    // Logical shift left
    isa.Instr{
        .name = "lsl",
        .enc = .init("1110001111xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Shift(false, .ll),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "lsl",
        .enc = .init("1110xxx1xx001xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Shift(true, .ll),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "lsl",
        .enc = .init("1110xxx1xx101xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Shift(true, .ll),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Logical shift right
    isa.Instr{
        .name = "lsr",
        .enc = .init("1110001011xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Shift(false, .lr),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "lsr",
        .enc = .init("1110xxx0xx001xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Shift(true, .lr),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "lsr",
        .enc = .init("1110xxx0xx101xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Shift(true, .lr),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Move data
    isa.Instr{
        .name = "move",
        .enc = .init("00xxxxxxxxxxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.Ea(6, 9, .{}),
        .op = op.Move(true),
        .size = .{ .dyn = .{ .at = 12, .b = 0b01, .w = 0b11, .l = 0b10 } },
    },

    // Move address
    isa.Instr{
        .name = "movea",
        .enc = .init("00xxxxx001xxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.AddrReg(9),
        .op = op.MoveExtend(u32),
        .size = .{ .dyn = .{ .at = 12, .w = 0b11, .l = 0b10 } },
    },

    // Move to CCR
    isa.Instr{
        .name = "move",
        .enc = .init("0100010011xxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.Status,
        .op = op.Move(false),
        .size = .{ .fixed = .w },
        .clk = 8,
    },

    // Move from SR
    isa.Instr{
        .name = "move",
        .enc = .init("0100000011xxxxxx"),
        .src = arg.Status,
        .dst = arg.Ea(3, 0, .{ .w = .initDefault(2, .{ .data_reg = 2 }) }),
        .op = op.Move(false),
        .size = .{ .fixed = .w },
    },

    // Store multiple registers to memory
    isa.Instr{
        .name = "movem",
        .enc = .init("010010001xxxxxxx"),
        .src = arg.Multiple(.store, 3, 0),
        .dst = arg.Multiple(.addr, 3, 0),
        .size = .{ .dyn = .{ .at = 6, .w = 0, .l = 1 } },
    },

    // Load multiple registers from memory
    isa.Instr{
        .name = "movem",
        .enc = .init("010011001xxxxxxx"),
        .src = arg.Multiple(.addr, 3, 0),
        .dst = arg.Multiple(.load, 3, 0),
        .size = .{ .dyn = .{ .at = 6, .w = 0, .l = 1 } },
    },

    // Move data to peripheral
    isa.Instr{
        .name = "movep",
        .enc = .init("0000xxx11x001xxx"),
        .src = arg.DataReg(9),
        .dst = arg.Peripheral(0),
        .size = .{ .dyn = .{ .at = 6, .w = 0, .l = 1 } },
    },

    // Move data from peripheral
    isa.Instr{
        .name = "movep",
        .enc = .init("0000xxx10x001xxx"),
        .src = arg.Peripheral(0),
        .dst = arg.DataReg(9),
        .size = .{ .dyn = .{ .at = 6, .w = 0, .l = 1 } },
    },

    // Move quick
    isa.Instr{
        .name = "moveq",
        .enc = .init("0111xxx0xxxxxxxx"),
        .src = arg.Opcode(i8, 0),
        .dst = arg.DataReg(9),
        .op = op.Move(true),
        .size = .{ .fixed = .l },
    },

    // Multiply signed
    isa.Instr{
        .name = "muls",
        .enc = .init("1100xxx111xxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.DataReg(9),
        .op = op.Muls,
        .size = .{ .fixed = .w },
    },

    // Multiply unsigned
    isa.Instr{
        .name = "mulu",
        .enc = .init("1100xxx011xxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.DataReg(9),
        .op = op.Mulu,
        .size = .{ .fixed = .w },
    },

    // Negate binary coded decimal
    isa.Instr{
        .name = "nbcd",
        .enc = .init("0100100000xxxxxx"),
        .src = arg.Const(u8, 0),
        .dst = arg.Ea(3, 0, .{ .b = .initDefault(0, .{ .data_reg = 2 }) }),
        .op = op.Sbcd(.src),
        .size = .{ .fixed = .b },
        .disasm = &.{ .name, .size, .space, .dst },
    },

    // Negate register
    isa.Instr{
        .name = "neg",
        .enc = .init("01000100xxxxxxxx"),
        .src = arg.Const(u8, 0),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 2 }) }),
        .op = op.Sub(.src),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
        .disasm = &.{ .name, .size, .space, .dst },
    },

    // Negate with extend
    isa.Instr{
        .name = "negx",
        .enc = .init("01000000xxxxxxxx"),
        .src = arg.Const(u8, 0),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 2 }) }),
        .op = op.Subx(.src),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
        .disasm = &.{ .name, .size, .space, .dst },
    },

    // No operation (NOP)
    isa.Instr{
        .name = "nop",
        .enc = .init("0100111001110001"),
    },

    // Bitwise not
    isa.Instr{
        .name = "not",
        .enc = .init("0100011000xxxxxx"),
        .src = arg.Const(u8, 0xFF),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 2 }) }),
        .op = op.Logic(.eor),
        .size = .{ .fixed = .b },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "not",
        .enc = .init("0100011001xxxxxx"),
        .src = arg.Const(u16, 0xFFFF),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 2 }) }),
        .op = op.Logic(.eor),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "not",
        .enc = .init("0100011010xxxxxx"),
        .src = arg.Const(u32, 0xFFFFFFFF),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 2 }) }),
        .op = op.Logic(.eor),
        .size = .{ .fixed = .l },
        .disasm = &.{ .name, .size, .space, .dst },
    },

    // Logical or
    isa.Instr{
        .name = "or",
        .enc = .init("1000xxx0xxxxxxxx"),
        .src = arg.Ea(3, 0, .{ .l = .initDefault(2, .{
            .data_reg = 4,
            .immediate = 4,
        }) }),
        .dst = arg.DataReg(9),
        .op = op.Logic(.@"or"),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "or",
        .enc = .init("1000xxx1xxxxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Logic(.@"or"),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Logical or immediate
    isa.Instr{
        .name = "ori",
        .enc = .init("00000000xx111100"),
        .src = arg.Imm(.{}),
        .dst = arg.Status,
        .op = op.Logic(.@"or"),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01 } },
        .clk = 12,
    },
    isa.Instr{
        .name = "ori",
        .enc = .init("00000000xxxxxxxx"),
        .src = arg.Imm(.{}),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = op.Logic(.@"or"),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Push effective address
    isa.Instr{
        .name = "pea",
        .enc = .init("0100100001xxxxxx"),
        .src = arg.Addr(3, 0, .{ .l = .initDefault(0, .{ .pc_idx = 2, .addr_idx = 2 }) }),
        .op = op.Push,
        .size = .{ .fixed = .l },
    },

    // Rotate right
    isa.Instr{
        .name = "ror",
        .enc = .init("1110011011xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Rotate(false, .r),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "ror",
        .enc = .init("1110xxx0xx011xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Rotate(true, .r),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "ror",
        .enc = .init("1110xxx0xx111xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Rotate(true, .r),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Rotate left
    isa.Instr{
        .name = "rol",
        .enc = .init("1110011111xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Rotate(false, .l),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "rol",
        .enc = .init("1110xxx1xx011xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Rotate(true, .l),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "rol",
        .enc = .init("1110xxx1xx111xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Rotate(true, .l),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Rotate right extend
    isa.Instr{
        .name = "rorx",
        .enc = .init("1110010011xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Rotate(false, .rx),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "rorx",
        .enc = .init("1110xxx0xx010xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Rotate(true, .rx),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "rorx",
        .enc = .init("1110xxx0xx110xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Rotate(true, .rx),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Rotate left extend
    isa.Instr{
        .name = "rolx",
        .enc = .init("1110010111xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Rotate(false, .lx),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "rolx",
        .enc = .init("1110xxx1xx010xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Rotate(true, .lx),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "rolx",
        .enc = .init("1110xxx1xx110xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Rotate(true, .lx),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Return and restore condition codes
    isa.Instr{
        .name = "rtr",
        .enc = .init("0100111001110111"),
        .op = op.Return(.ccr),
        .clk = 4,
    },

    // Return from subroutine
    isa.Instr{
        .name = "rts",
        .enc = .init("0100111001110101"),
        .op = op.Return(.none),
        .clk = 4,
    },

    // Subtract binary coded decimal
    isa.Instr{
        .name = "sbcd",
        .enc = .init("1000xxx10000xxxx"),
        .src = arg.RegReg(3, 0, .{}),
        .dst = arg.RegReg(3, 9, .{ .b = .{ 2, 2 } }),
        .op = op.Sbcd(.dst),
        .size = .{ .fixed = .b },
    },

    // Set according to condition
    isa.Instr{
        .name = "s",
        .enc = .init("0101xxxx11xxxxxx"),
        .src = arg.Opcode(Ctx.Cond, 8),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Scc(0),
        .size = .{ .fixed = .b },
        .disasm = &.{ .name, .src, .size, .space, .dst },
    },
    isa.Instr{
        .name = "s",
        .enc = .init("0101xxxx11000xxx"),
        .src = arg.Opcode(Ctx.Cond, 8),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Scc(2),
        .size = .{ .fixed = .b },
        .disasm = &.{ .name, .src, .size, .space, .dst },
    },

    // Subtract
    isa.Instr{
        .name = "sub",
        .enc = .init("1001xxx0xxxxxxxx"),
        .src = arg.Ea(3, 0, .{ .l = .initDefault(2, .{
            .data_reg = 4,
            .addr_reg = 4,
            .immediate = 4,
        }) }),
        .dst = arg.DataReg(9),
        .op = op.Sub(.dst),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "sub",
        .enc = .init("1001xxx1xxxxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Sub(.dst),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Subtract address
    isa.Instr{
        .name = "suba",
        .enc = .init("1001xxxx11xxxxxx"),
        .src = arg.Ea(3, 0, .{
            .b = .initFill(4),
            .w = .initFill(4),
            .l = .initDefault(2, .{
                .data_reg = 4,
                .addr_reg = 4,
                .immediate = 4,
            }),
        }),
        .dst = arg.AddrReg(9),
        .op = op.Suba,
        .size = .{ .dyn = .{ .at = 8, .w = 0, .l = 1 } },
    },

    // Subtract immediate
    isa.Instr{
        .name = "subi",
        .enc = .init("00000100xxxxxxxx"),
        .src = arg.Imm(.{}),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = op.Sub(.dst),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Subtract quick
    isa.Instr{
        .name = "subq",
        .enc = .init("0101xxx1xxxxxxxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = op.Sub(.dst),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "subq",
        .enc = .init("0101xxx1xx001xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.AddrReg(0),
        .op = op.Suba,
        .size = .{ .dyn = .{ .at = 6, .w = 0b01, .l = 0b10 } },
        .clk = 4,
    },
    isa.Instr{
        .name = "subq",
        .enc = .init("01010001xx001xxx"),
        .src = arg.Const(u4, 8),
        .dst = arg.AddrReg(0),
        .op = op.Suba,
        .size = .{ .dyn = .{ .at = 6, .w = 0b01, .l = 0b10 } },
        .clk = 4,
    },

    // Subtract extend
    isa.Instr{
        .name = "subx",
        .enc = .init("1001xxx1xx00xxxx"),
        .src = arg.RegReg(3, 0, .{}),
        .dst = arg.RegReg(3, 9, .{ .b = .{ 0, 2 }, .l = .{ 4, 2 } }),
        .op = op.Subx(.dst),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Swap register halves
    isa.Instr{
        .name = "swap",
        .enc = .init("0100100001000xxx"),
        .dst = arg.RegIdx(.data, 0),
        .op = op.Swap,
        .size = .{ .fixed = .w },
    },

    // Test and set an operand
    isa.Instr{
        .name = "tas",
        .enc = .init("0100101011000xxx"),
        .dst = arg.DataReg(0),
        .op = op.Tas,
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "tas",
        .enc = .init("0100101011xxxxxx"),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Tas,
        .clk = 2,
        .size = .{ .fixed = .b },
    },

    // Trap
    isa.Instr{
        .name = "trap",
        .enc = .init("010011100100xxxx"),
        .src = arg.Opcode(u4, 0),
        .op = op.Trap,
    },

    // Trapv
    isa.Instr{
        .name = "trapv",
        .enc = .init("0100111001110110"),
        .op = op.Trapv,
    },

    // Test an operand
    isa.Instr{
        .name = "tst",
        .enc = .init("01001010xxxxxxxx"),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Tst,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Unlink
    isa.Instr{
        .name = "unlk",
        .enc = .init("0100111001011xxx"),
        .dst = arg.AddrReg(0),
        .op = op.Unlink,
        .size = .{ .fixed = .w },
    },

    // Move from usp
    isa.Instr{
        .name = "move",
        .enc = .init("0100111001101xxx"),
        .src = arg.Usp,
        .dst = arg.AddrReg(0),
        .size = .{ .fixed = .l },
    },

    // Move to usp
    isa.Instr{
        .name = "move",
        .enc = .init("0100111001100xxx"),
        .src = arg.AddrReg(0),
        .dst = arg.Usp,
        .size = .{ .fixed = .l },
    },

    // Reset
    isa.Instr{
        .name = "reset",
        .enc = .init("0100111001110000"),
        .clk = 128,
    },

    // Stop
    isa.Instr{
        .name = "stop",
        .enc = .init("0100111001110010"),
        .src = arg.Imm(.{}),
        .op = op.Stop,
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .space, .src },
    },

    // Return from exception
    isa.Instr{
        .name = "rte",
        .enc = .init("0100111001110011"),
        .op = op.Return(.sr),
        .clk = 4,
    },
});
