const std = @import("std");

const bus_interface = @import("bus");

const arg = @import("arg.zig");
pub const Cpu = @import("Cpu.zig");
const Ctx = @import("Ctx.zig");
const isa = @import("isa.zig");
const ops = @import("ops.zig");

const Bus = bus_interface.Bus(Cpu.width);

/// Process an exception
pub fn exception(vector: Cpu.Vector, cpu: *Cpu, bus: *Bus) usize {
    // Run exception specific code
    var ctx = Ctx{ .cpu = cpu, .bus = bus };
    switch (vector) {
        .illegal => {
            ctx.clk += 12;
            ctx.push(u32, ctx.cpu.pc);
            ctx.push(Cpu.Status, ctx.cpu.sr);
            ctx.cpu.pc = ctx.read(u32, vector.addr());
        },
        _ => {},
    }
    return ctx.clk + 2;
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
    isa.Instr{
        .name = "abcd",
        .enc = .init("1100xxx10000xxxx"),
        .src = arg.RegReg(3, 0, .{}),
        .dst = arg.RegReg(3, 9, .{ .b = .{ 2, 2 } }),
        .op = ops.Abcd,
        .size = Ctx.Size.Enc{ .fixed = .b },
    },
    isa.Instr{
        .name = "add",
        .enc = .init("1101xxx0xxxxxxxx"),
        .src = arg.Ea(3, 0, .{ .l = .initDefault(2, .{
            .data_reg = 4,
            .addr_reg = 4,
            .immediate = 4,
        }) }),
        .dst = arg.DataReg(9),
        .op = ops.Add,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "add",
        .enc = .init("1101xxx1xxxxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = ops.Add,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
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
        .op = ops.Adda,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 8, .w = 0, .l = 1 } },
    },
    isa.Instr{
        .name = "addi",
        .enc = .init("00000110xxxxxxxx"),
        .src = arg.Imm,
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = ops.Add,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "addq",
        .enc = .init("0101xxx0xxxxxxxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = ops.Add,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "addq",
        .enc = .init("0101xxx0xx001xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.AddrReg(0),
        .op = ops.Adda,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .w = 0b01, .l = 0b10 } },
        .clk = 4,
    },
    isa.Instr{
        .name = "addx",
        .enc = .init("1101xxx1xx00xxxx"),
        .src = arg.RegReg(3, 0, .{}),
        .dst = arg.RegReg(3, 9, .{ .b = .{ 0, 2 }, .l = .{ 4, 2 } }),
        .op = ops.Addx,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "and",
        .enc = .init("1100xxx0xxxxxxxx"),
        .src = arg.Ea(3, 0, .{ .l = .initDefault(2, .{
            .data_reg = 4,
            .immediate = 4,
        }) }),
        .dst = arg.DataReg(9),
        .op = ops.And,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "and",
        .enc = .init("1100xxx1xxxxxxxx"),
        .src = arg.DataReg(9),
        .dst = arg.Ea(3, 0, .{}),
        .op = ops.And,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "andi",
        .enc = .init("00000010xx111100"),
        .src = arg.Imm,
        .dst = arg.Status,
        .op = ops.And,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01 } },
        .clk = 12,
    },
    isa.Instr{
        .name = "andi",
        .enc = .init("00000010xxxxxxxx"),
        .src = arg.Imm,
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = ops.And,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110000111xxxxxx"),
        .src = arg.Const(u3, 1),
        .dst = arg.Ea(3, 0, .{}),
        .op = ops.Asl,
        .size = Ctx.Size.Enc{ .fixed = .w },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110xxx1xx000xxx"),
        .src = arg.Opcode(u3, 9),
        .dst = arg.DataReg(0),
        .op = ops.Asl,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110xxx1xx100xxx"),
        .src = arg.DataReg(9),
        .dst = arg.DataReg(0),
        .op = ops.Asl,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "ori",
        .enc = .init("00000000xx111100"),
        .src = arg.Imm,
        .dst = arg.Status,
        .op = ops.Or,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01 } },
        .clk = 12,
    },
    isa.Instr{
        .name = "ori",
        .enc = .init("00000000xxxxxxxx"),
        .src = arg.Imm,
        .dst = arg.Ea(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = ops.Or,
        .size = Ctx.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "nop",
        .enc = .init("0100111001110001"),
    },
});
