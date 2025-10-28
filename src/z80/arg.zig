const std = @import("std");
const Writer = std.io.Writer;
const Reader = std.io.Reader;
const WriteError = std.io.Writer.Error;

const int = @import("int");

const Cpu = @import("Cpu.zig");
const exec = @import("exec.zig");
const Bus = exec.Bus;
const Prefix = exec.Prefix;

/// 8 bit operand
pub fn r8(comptime pos: u3) type {
    return struct {
        pub fn load(cpu: *Cpu, mem: *Bus, _: *Bus, prefix: Prefix) u8 {
            return switch (int.extract(u3, prefix.bytes[0], pos)) {
                0...5 => |n| @as(Cpu.Bank.R8, @enumFromInt(n)).get(cpu.banks[cpu.set]),
                6 => exec.read(mem, u8, Cpu.Bank.R16.hl.get(cpu.banks[cpu.set])),
                7 => cpu.banks[cpu.set].a,
            };
        }

        pub fn store(cpu: *Cpu, mem: *Bus, _: *Bus, prefix: Prefix, value: u8) void {
            switch (int.extract(u3, prefix.bytes[0], pos)) {
                0...5 => |n| @as(Cpu.Bank.R8, @enumFromInt(n)).set(cpu.banks[cpu.set], value),
                6 => exec.write(mem, u8, Cpu.Bank.R16.hl.get(cpu.banks[cpu.set]), value),
                7 => cpu.banks[cpu.set].a = value,
            }
        }

        pub fn disasm(writer: *Writer, _: *Reader, opcode: Prefix) WriteError!void {
            try writer.print("{s}", .{switch (int.extract(u3, opcode.bytes, pos)) {
                0...5 => |n| @tagName(@as(Cpu.Bank.R8, @enumFromInt(n))),
                6 => "[hl]",
                7 => "a",
            }});
        }
    };
}

/// 16 bit operand
pub fn r16(comptime pos: u3) type {
    return struct {
        pub fn load(cpu: *Cpu, _: *Bus, _: *Bus, prefix: Prefix) u16 {
            return switch (int.extract(u2, prefix.bytes[0], pos)) {
                0...2 => |n| @as(Cpu.Bank.R16, @enumFromInt(n)).get(cpu.banks[cpu.set]),
                3 => cpu.sp,
            };
        }

        pub fn store(cpu: *Cpu, _: *Bus, _: *Bus, prefix: Prefix, value: u16) void {
            switch (int.extract(u2, prefix.bytes[0], pos)) {
                0...2 => |n| @as(Cpu.Bank.R16, @enumFromInt(n)).set(cpu.banks[cpu.set], value),
                3 => cpu.sp = value,
            }
        }

        pub fn disasm(writer: *Writer, _: *Reader, prefix: Prefix) WriteError!void {
            try writer.print("{s}", .{switch (int.extract(u2, prefix.bytes[0], pos)) {
                0...2 => |n| @tagName(@as(Cpu.Bank.R16, @enumFromInt(n))),
                3 => "sp",
            }});
        }
    };
}

/// Immediate operand
pub fn imm(comptime Int: type) type {
    return struct {
        pub fn load(cpu: *Cpu, mem: *Bus, _: *Bus, _: Prefix) Int {
            return exec.fetch(cpu, mem, Int);
        }

        pub fn disasm(writer: *Writer, reader: *Reader, _: Prefix) WriteError!void {
            const data = reader.takeInt(Int, .little) orelse return error.WriteFailed;
            switch (@bitSizeOf(Int)) {
                8 => try writer.print("#${x:0>2}", .{data}),
                16 => try writer.print("#${x:0>4}", .{data}),
                else => @compileError("Exepected 8 or 16 bit immediate for z80 disassembly"),
            }
        }
    };
}
