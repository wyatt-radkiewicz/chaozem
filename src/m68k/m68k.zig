const std = @import("std");

pub const Bus = @import("Bus.zig");
pub const Cpu = @import("Cpu.zig");
const Exec = @import("Exec.zig");
const isa = @import("isa.zig");
const ops = @import("ops.zig");
const targets = @import("targets.zig");
pub const Disasm = isa.Disasm;

/// Process an exception
pub fn exception(vector: Cpu.Vector, cpu: *Cpu, bus: *Bus) usize {
    // Run exception specific code
    var exec = Exec{
        .cpu = cpu,
        .bus = bus,
    };
    switch (vector) {
        .illegal => {
            exec.clk += 12;
            exec.push(u32, exec.cpu.pc);
            exec.push(Cpu.Status, exec.cpu.sr);
            exec.cpu.pc = exec.read(u32, vector.addr());
        },
        _ => {},
    }
    return exec.clk + 2;
}

/// Run one instruction
pub fn step(cpu: *Cpu, bus: *Bus) usize {
    var exec = Exec{
        .cpu = cpu,
        .bus = bus,
    };
    const opcode = Exec.fetch(&exec, u16);
    if (m68k_isa.handler(opcode)) |pfn| {
        pfn(opcode, &exec);
    }
    return exec.clk;
}

/// M68k instruction set architecture
const m68k_isa = isa.Isa(&.{
    isa.Instr{
        .name = "abcd",
        .enc = .init("1100xxx10000xxxx"),
        .src = targets.RegRegTarget(3, 0, .{}),
        .dst = targets.RegRegTarget(3, 9, .{ .b = .{ 2, 2 } }),
        .op = ops.Abcd,
        .size = Exec.Size.Enc{ .fixed = .b },
    },
    isa.Instr{
        .name = "add",
        .enc = .init("1101xxx0xxxxxxxx"),
        .src = targets.EaTarget(3, 0, .{ .l = .initDefault(2, .{
            .data_reg = 4,
            .addr_reg = 4,
            .immediate = 4,
        }) }),
        .dst = targets.DataTarget(9),
        .op = ops.Add,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "add",
        .enc = .init("1101xxx1xxxxxxxx"),
        .src = targets.DataTarget(9),
        .dst = targets.EaTarget(3, 0, .{}),
        .op = ops.Add,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "adda",
        .enc = .init("1101xxxx11xxxxxx"),
        .src = targets.EaTarget(3, 0, .{
            .b = .initFill(4),
            .w = .initFill(4),
            .l = .initDefault(2, .{
                .data_reg = 4,
                .addr_reg = 4,
                .immediate = 4,
            }),
        }),
        .dst = targets.AddrTarget(9),
        .op = ops.Adda,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 8, .w = 0, .l = 1 } },
    },
    isa.Instr{
        .name = "addi",
        .enc = .init("00000110xxxxxxxx"),
        .src = targets.ImmTarget,
        .dst = targets.EaTarget(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = ops.Add,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "addq",
        .enc = .init("0101xxx0xxxxxxxx"),
        .src = targets.QuickTarget(u3, 9),
        .dst = targets.EaTarget(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = ops.Add,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "addq",
        .enc = .init("0101xxx0xx001xxx"),
        .src = targets.QuickTarget(u3, 9),
        .dst = targets.AddrTarget(0),
        .op = ops.Adda,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .w = 0b01, .l = 0b10 } },
        .clk = 4,
    },
    isa.Instr{
        .name = "addx",
        .enc = .init("1101xxx1xx00xxxx"),
        .src = targets.RegRegTarget(3, 0, .{}),
        .dst = targets.RegRegTarget(3, 9, .{ .b = .{ 0, 2 }, .l = .{ 4, 2 } }),
        .op = ops.Addx,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "and",
        .enc = .init("1100xxx0xxxxxxxx"),
        .src = targets.EaTarget(3, 0, .{ .l = .initDefault(2, .{
            .data_reg = 4,
            .immediate = 4,
        }) }),
        .dst = targets.DataTarget(9),
        .op = ops.And,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "and",
        .enc = .init("1100xxx1xxxxxxxx"),
        .src = targets.DataTarget(9),
        .dst = targets.EaTarget(3, 0, .{}),
        .op = ops.And,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "andi",
        .enc = .init("00000010xx111100"),
        .src = targets.ImmTarget,
        .dst = targets.SrTarget,
        .op = ops.And,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01 } },
        .clk = 12,
    },
    isa.Instr{
        .name = "andi",
        .enc = .init("00000010xxxxxxxx"),
        .src = targets.ImmTarget,
        .dst = targets.EaTarget(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = ops.And,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110000111xxxxxx"),
        .src = targets.ConstTarget(u3, 1),
        .dst = targets.EaTarget(3, 0, .{}),
        .op = ops.Asl,
        .size = Exec.Size.Enc{ .fixed = .w },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110xxx1xx000xxx"),
        .src = targets.QuickTarget(u3, 9),
        .dst = targets.DataTarget(0),
        .op = ops.Asl,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "asl",
        .enc = .init("1110xxx1xx100xxx"),
        .src = targets.DataTarget(9),
        .dst = targets.DataTarget(0),
        .op = ops.Asl,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "ori",
        .enc = .init("00000000xx111100"),
        .src = targets.ImmTarget,
        .dst = targets.SrTarget,
        .op = ops.Or,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01 } },
        .clk = 12,
    },
    isa.Instr{
        .name = "ori",
        .enc = .init("00000000xxxxxxxx"),
        .src = targets.ImmTarget,
        .dst = targets.EaTarget(3, 0, .{ .l = .initDefault(0, .{ .data_reg = 4 }) }),
        .op = ops.Or,
        .size = Exec.Size.Enc{ .dyn = .{ .at = 6, .b = 0b00, .w = 0b01, .l = 0b10 } },
    },
    isa.Instr{
        .name = "nop",
        .enc = .init("0100111001110001"),
    },
});
