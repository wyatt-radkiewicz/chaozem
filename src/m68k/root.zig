const std = @import("std");

const bus_interface = @import("bus");

const arg = @import("arg.zig");
pub const Cpu = @import("Cpu.zig");
const Ctx = @import("Ctx.zig");
const isa = @import("isa.zig");
const op = @import("op.zig");

const Bus = bus_interface.Bus(Ctx.bus_width);

/// M68k vectors
pub const Vector = Ctx.Vector;

/// M68k processor width
pub const bus_width = Ctx.bus_width;

/// Process an exception
pub fn exception(vector: Ctx.Vector, cpu: *Cpu, bus: *Bus) usize {
    var ctx = Ctx{ .cpu = cpu, .bus = bus };
    vector.handle(&ctx);
    return ctx.clk;
}

/// Run one instruction
pub fn step(cpu: *Cpu, bus: *Bus) usize {
    var ctx = Ctx{ .cpu = cpu, .bus = bus };
    const opcode = ctx.fetch(u16);
    if (m68k_isa.runner(opcode)) |pfn| {
        pfn(&ctx, opcode);
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
        .src = arg.Imm(null, .{}),
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
        .op = op.And,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "and",
        .enc = .init("1100xxx1xxxxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.And,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Logical and immediate
    isa.Instr{
        .name = "andi",
        .enc = .init("00000010xx111100"),
        .src = arg.Imm(null, .{}),
        .dst = arg.Status,
        .op = op.And,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01 } },
        .clk = 12,
    },
    isa.Instr{
        .name = "andi",
        .enc = .init("00000010xxxxxxxx"),
        .src = arg.Imm(null, .{}),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = op.And,
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Arithmatic shift left
    isa.Instr{
        .name = "asl",
        .enc = .init("1110000111xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Asd(false, .l),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110xxx1xx000xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Asd(true, .l),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110xxx1xx100xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Asd(true, .l),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },

    // Arithmatic shift right
    isa.Instr{
        .name = "asr",
        .enc = .init("1110000011xxxxxx"),
        .src = arg.Const(u4, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Asd(false, .r),
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },
    isa.Instr{
        .name = "asr",
        .enc = .init("1110xxx0xx000xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = op.Asd(true, .r),
        .size = .{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "asr",
        .enc = .init("1110xxx0xx100xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = op.Asd(true, .r),
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
        .dst = arg.Imm(u16, .{ .hex = false }),
        .op = op.Branch,
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .src, .size, .space, .dst },
    },

    // Test and change a bit
    isa.Instr{
        .name = "bchg",
        .enc = .init("0000100001xxxxxx"),
        .src = arg.Imm(u3, .{ .hex = false }),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.chg),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "bchg",
        .enc = .init("0000100001000xxx"),
        .src = arg.Imm(u5, .{ .hex = false }),
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
        .src = arg.Imm(u3, .{ .hex = false }),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.clr),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "bclr",
        .enc = .init("0000100010000xxx"),
        .src = arg.Imm(u5, .{ .hex = false }),
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
        .dst = arg.Imm(u16, .{ .hex = false }),
        .op = op.Branch,
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },

    // Test and set a bit
    isa.Instr{
        .name = "bset",
        .enc = .init("0000100011xxxxxx"),
        .src = arg.Imm(u3, .{ .hex = false }),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.set),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "bset",
        .enc = .init("0000100011000xxx"),
        .src = arg.Imm(u5, .{ .hex = false }),
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
        .dst = arg.Imm(u16, .{ .hex = false }),
        .op = op.Branch,
        .size = .{ .fixed = .w },
        .disasm = &.{ .name, .size, .space, .dst },
    },

    // Test a bit
    isa.Instr{
        .name = "btst",
        .enc = .init("0000100000xxxxxx"),
        .src = arg.Imm(u3, .{ .hex = false }),
        .dst = arg.Ea(3, 0, .{}),
        .op = op.Bit(.tst),
        .size = .{ .fixed = .b },
    },
    isa.Instr{
        .name = "btst",
        .enc = .init("0000100000000xxx"),
        .src = arg.Imm(u5, .{ .hex = false }),
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
    isa.Instr{
        .name = "chk",
        .enc = .init("0100xxx110xxxxxx"),
        .src = arg.Ea(3, 0, .{}),
        .dst = arg.DataReg(9),
        .op = op.Chk,
        .size = .{ .fixed = .w },
    },
});
