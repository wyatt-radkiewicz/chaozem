const std = @import("std");

pub const Cpu = struct {
    cycles: usize = 0,
    data: [8]u32 = [1]u32{0} ** 8,
    addr: [8]u32 = [1]u32{0} ** 8,
    pc: u32 = 0,
    sr: Status = .{},
    ir: u16 = 0,

    fn addBusCycles(this: *@This(), num_transfer_bits: comptime_int) void {
        this.cycles += switch (num_transfer_bits) {
            8, 16 => 4,
            32 => 8,
            else => @compileError("Cpu tried transfering type not of 8,16, or 32 bits in size!"),
        };
    }

    fn read(this: *@This(), bus: Bus, comptime Data: type, addr: u32) Data {
        this.addBusCycles(@bitSizeOf(Data));
        return bus.read(Data, addr);
    }

    fn write(this: *@This(), bus: Bus, comptime Data: type, addr: u32, data: Data) void {
        this.addBusCycles(@bitSizeOf(Data));
        return bus.write(Data, addr, data);
    }

    fn fetch(this: *@This(), bus: Bus, comptime Data: type) Data {
        const fetch_bits = @max(16, @bitSizeOf(Data));
        const raw_data = this.read(bus, std.meta.Int(.unsigned, fetch_bits), this.pc);
        this.*.pc += fetch_bits / 8;
        return @bitCast(@as(std.meta.Int(.unsigned, @bitSizeOf(Data)), @truncate(raw_data)));
    }
};

pub const Bus = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        read_byte: *const fn (ptr: *anyopaque, addr: u32) u8,
        read_word: *const fn (ptr: *anyopaque, addr: u32) u16,
        read_long: *const fn (ptr: *anyopaque, addr: u32) u32,

        write_byte: *const fn (ptr: *anyopaque, addr: u32, data: u8) void,
        write_word: *const fn (ptr: *anyopaque, addr: u32, data: u16) void,
        write_long: *const fn (ptr: *anyopaque, addr: u32, data: u32) void,
    };

    fn read(this: @This(), comptime Data: type, addr: u32) Data {
        return @bitCast(switch (@bitSizeOf(Data)) {
            8 => this.vtable.read_byte(this.ptr, addr),
            16 => this.vtable.read_word(this.ptr, addr),
            32 => this.vtable.read_long(this.ptr, addr),
            else => @compileError("Tried reading data not of size 8,16, or 32 bits!"),
        });
    }

    fn write(this: @This(), comptime Data: type, addr: u32, data: Data) void {
        switch (@bitSizeOf(Data)) {
            8 => this.vtable.write_byte(this.ptr, addr, @bitCast(data)),
            16 => this.vtable.write_word(this.ptr, addr, @bitCast(data)),
            32 => this.vtable.write_long(this.ptr, addr, @bitCast(data)),
            else => @compileError("Tried writed data not of size 8,16, or 32 bits!"),
        }
    }
};

pub const Status = packed struct {
    carry: u1 = 0,
    overflow: u1 = 0,
    zero: u1 = 0,
    negative: u1 = 0,
    extend: u1 = 0,
    _pad0: u2 = 0,
    ipl: u3 = 0,
    _pad1: u1 = 0,
    master: u1 = 0,
    supervisor: u1 = 0,
    trace: u2 = 0,
};

pub const ExtensionWord = packed struct {
    disp: i8,
    _pad0: u3,
    size: u1,
    reg: u3,
    mode: u1,

    fn getReg(this: @This(), cpu: Cpu) u32 {
        const reg = switch (this.mode) {
            0 => cpu.data[this.reg],
            1 => cpu.addr[this.reg],
        };
        return switch (this.size) {
            0 => extend(u32, @as(u16, @truncate(reg))),
            1 => reg,
        };
    }
};

const AdrMode = enum { data, addr, ind, inc, dec, adisp, aidx, absw, absl, pcdisp, pcidx, imm };

fn EffAddr(comptime Data: type) type {
    return struct {
        mode: AdrMode,
        addr: u32,

        fn calc(cpu: *Cpu, bus: Bus, m: u3, n: u3) @This() {
            return switch (m) {
                0b000 => .{ .mode = .data, .addr = n },
                0b001 => .{ .mode = .addr, .addr = n },
                0b010 => .{ .mode = .ind, .addr = cpu.addr[n] },
                0b011 => inc: {
                    const addr = cpu.addr[n];
                    cpu.addr[n] += @sizeOf(Data);
                    break :inc .{ .mode = .inc, .addr = addr };
                },
                0b100 => dec: {
                    cpu.addr[n] -= @sizeOf(Data);
                    cpu.cycles += 2;
                    break :dec .{ .mode = .dec, .addr = cpu.addr[n] };
                },
                0b101 => .{
                    .mode = .adisp,
                    .addr = cpu.addr[n] +% extend(u32, cpu.fetch(bus, i16)),
                },
                0b110 => aidx: {
                    const ext_word = cpu.fetch(bus, ExtensionWord);
                    const disp = extend(u32, ext_word.disp);
                    break :aidx .{
                        .mode = .aidx,
                        .addr = cpu.addr[n] +% disp +% ext_word.getReg(cpu.*),
                    };
                },
                0b111 => switch (n) {
                    0b000 => .{ .mode = .absw, .addr = extend(u32, cpu.fetch(bus, i16)) },
                    0b001 => .{ .mode = .absl, .addr = cpu.fetch(bus, u32) },
                    0b010 => .{
                        .mode = .pcdisp,
                        .addr = cpu.pc +% extend(u32, cpu.fetch(bus, i16)),
                    },
                    0b011 => pcidx: {
                        const ext_word = cpu.fetch(bus, ExtensionWord);
                        const disp = extend(u32, ext_word.disp);
                        break :pcidx .{
                            .mode = .pcidx,
                            .addr = cpu.pc +% disp +% ext_word.getReg(cpu.*),
                        };
                    },
                    0b100 => imm: {
                        const addr = cpu.pc;
                        cpu.pc += @sizeOf(Data);
                        break :imm .{ .mode = .imm, .addr = addr };
                    },
                    else => undefined,
                },
            };
        }

        fn load(this: @This(), cpu: *Cpu, bus: Bus) Data {
            const Int = std.meta.Int(.unsigned, @bitSizeOf(Data));
            return switch (this.mode) {
                .data => @bitCast(@as(Int, @truncate(cpu.data[this.addr]))),
                .addr => @bitCast(@as(Int, @truncate(cpu.addr[this.addr]))),
                else => cpu.read(bus, Data, this.addr),
            };
        }

        fn store(this: @This(), cpu: *Cpu, bus: Bus, data: Data) void {
            const Int = std.meta.Int(.unsigned, @bitSizeOf(Data));
            switch (this.mode) {
                .data => {
                    cpu.data[this.addr] &= ~@as(u32, (1 << @bitSizeOf(Data)) - 1);
                    cpu.data[this.addr] |= @as(Int, @bitCast(data));
                },
                .addr => {
                    cpu.addr[this.addr] &= ~@as(u32, (1 << @bitSizeOf(Data)) - 1);
                    cpu.addr[this.addr] |= @as(Int, @bitCast(data));
                },
                else => cpu.write(bus, Data, this.addr, data),
            }
        }
    };
}

pub fn step(cpu: *Cpu, bus: Bus) void {
    runInstr(cpu, bus);
}

fn runInstr(cpu: *Cpu, bus: Bus) void {
    switch (extract(u4, cpu.ir, 12)) {
        0b0000 => {
            const op = extract(u3, cpu.ir, 7);
            const size_bits = extract(u2, cpu.ir, 6);
            const Size = switch (size_bits) {
                inline 0...3 => SizeEnc(size_bits),
                else => @panic("Bad size encoding"),
            };
            if (extract(u1, cpu.ir, 8) == 1) {
                switch (size_bits) {
                    0b00 => _ = btst(cpu, bus, op),
                    0b01 => bchg(cpu, bus, op),
                    0b10 => bclr(cpu, bus, op),
                    0b11 => bset(cpu, bus, op),
                }
            } else {
                switch (op) {
                    0b000 => ori(cpu, bus, Size),
                    0b100 => switch (size_bits) {
                        0b00 => _ = btst(cpu, bus, null),
                        0b01 => bchg(cpu, bus, null),
                        0b10 => bclr(cpu, bus, null),
                        0b11 => bset(cpu, bus, null),
                    },
                }
            }
        },
        inline 0b0001, 0b0011, 0b0010 => |size_enc| move(cpu, bus, size_enc),
        else => {},
    }
}

fn normEffAddr(cpu: *Cpu, bus: Bus, comptime Data: type) EffAddr(Data) {
    return EffAddr(Data).calc(cpu, bus, extract(u3, cpu.ir, 3), extract(u3, cpu.ir, 0));
}

fn extract(Something: type, from: anytype, at: std.math.Log2Int(@TypeOf(from))) Something {
    return @truncate(from >> at);
}

fn extend(To: type, int: anytype) To {
    const ToSigned = std.meta.Int(.signed, @bitSizeOf(To));
    const IntSigned = std.meta.Int(.signed, @bitSizeOf(@TypeOf(int)));
    return @bitCast(@as(ToSigned, @as(IntSigned, @bitCast(int))));
}

fn btst(cpu: *Cpu, bus: Bus, reg: ?u3) struct { EffAddr(u8), u8, u3 } {
    defer cpu.ir = cpu.fetch(bus, u16);
    const ea = normEffAddr(cpu, bus, u8);
    const bit = if (reg) |n|
        @as(u3, @truncate(cpu.addr[n]))
    else
        @as(u3, @truncate(cpu.fetch(bus, u8)));
    const byte = ea.load(cpu, bus);
    cpu.sr.zero = byte & 1 << bit == 0;
    return .{ ea, byte, bit };
}

fn bchg(cpu: *Cpu, bus: Bus, reg: ?u3) void {
    const ea, const byte, const bit = btst(cpu, bus, reg);
    ea.store(cpu, bus, byte ^ 1 << bit);
}

fn bclr(cpu: *Cpu, bus: Bus, reg: ?u3) void {
    const ea, const byte, const bit = btst(cpu, bus, reg);
    ea.store(cpu, bus, byte & ~1 << bit);
}

fn bset(cpu: *Cpu, bus: Bus, reg: ?u3) void {
    const ea, const byte, const bit = btst(cpu, bus, reg);
    ea.store(cpu, bus, byte | 1 << bit);
}

fn ori(cpu: *Cpu, bus: Bus, comptime Data: type) void {
    defer cpu.ir = cpu.fetch(bus, u16);
    const ea = normEffAddr(cpu, bus, Data);
    const imm = cpu.fetch(bus, Data);
    var output: Data = undefined;
    if (ea.mode == .imm) {
        output = @bitCast(@as(u16, @bitCast(cpu.sr)) | imm);
        cpu.sr = output;
    } else {
        output = ea.load(cpu, bus) | imm;
        ea.store(output);
    }
    if (@sizeOf(Data) == 4 and ea.mode == .data) {
        cpu.cycles += 4;
    }
    cpu.sr.negative = @intFromBool(output & 1 << @bitSizeOf(Data) - 1 != 0);
    cpu.sr.zero = @intFromBool(output == 0);
    cpu.sr.overflow = 0;
    cpu.sr.carry = 0;
}

fn SizeEnc(comptime enc: u2) type {
    return switch (enc) {
        0b00 => u8,
        0b01 => u16,
        0b10 => u32,
        0b11 => void,
    };
}

fn move(cpu: *Cpu, bus: Bus, comptime size_enc: u2) void {
    defer cpu.ir = cpu.fetch(bus, u16);
    const Data = std.meta.Int(.unsigned, switch (size_enc) {
        0b01 => 8,
        0b11 => 16,
        0b10 => 32,
        else => unreachable,
    });
    const src = normEffAddr(cpu, bus, Data);
    switch (extract(u3, cpu.ir, 6)) {
        0b001 => cpu.addr[extract(u3, cpu.ir, 9)] = extend(u32, src.load(cpu, bus)),
        else => |m| {
            const dst = EffAddr(Data).calc(cpu, bus, m, extract(u3, cpu.ir, 9));
            dst.store(cpu, bus, src.load(cpu, bus));
        },
    }
}

const TestRam = struct {
    ram: [1024 * 4]u8 = [1]u8{1} ** (1024 * 4),

    fn bus(this: *@This()) Bus {
        return .{
            .ptr = @ptrCast(this),
            .vtable = &.{
                .read_byte = read_byte,
                .read_word = read_word,
                .read_long = read_long,
                .write_byte = write_byte,
                .write_word = write_word,
                .write_long = write_long,
            },
        };
    }

    fn read_n(ctx: *anyopaque, addr: u32, comptime Data: type) Data {
        const this: *@This() = @ptrCast(ctx);
        return if (addr + @sizeOf(Data) > this.ram.len) 0 else get: {
            break :get std.mem.readInt(Data, this.ram[addr..][0..@sizeOf(Data)], .big);
        };
    }

    fn read_byte(ctx: *anyopaque, addr: u32) u8 {
        return read_n(ctx, addr, u8);
    }
    fn read_word(ctx: *anyopaque, addr: u32) u16 {
        return read_n(ctx, addr, u16);
    }
    fn read_long(ctx: *anyopaque, addr: u32) u32 {
        return read_n(ctx, addr, u32);
    }

    fn write_n(ctx: *anyopaque, addr: u32, comptime Data: type, data: Data) void {
        const this: *@This() = @ptrCast(ctx);
        if (addr + @sizeOf(Data) <= this.ram.len) {
            std.mem.writeInt(Data, this.ram[addr..][0..@sizeOf(Data)], data, .big);
        }
    }

    fn write_byte(ctx: *anyopaque, addr: u32, data: u8) void {
        write_n(ctx, addr, u8, data);
    }
    fn write_word(ctx: *anyopaque, addr: u32, data: u16) void {
        write_n(ctx, addr, u16, data);
    }
    fn write_long(ctx: *anyopaque, addr: u32, data: u32) void {
        write_n(ctx, addr, u32, data);
    }
};

test "move ea,ea" {
    var ram = TestRam{};
    var cpu = Cpu{};

    cpu.data[1] = 0x12345678;
    cpu.ir = 0x3001;
    step(&cpu, ram.bus());
    try std.testing.expectEqual(4, cpu.cycles);
    try std.testing.expectEqual(0x00005678, cpu.data[0]);
}

test "movea ea,an" {
    var ram = TestRam{};
    var cpu = Cpu{};

    cpu.data[1] = 0x1234a678;
    cpu.ir = 0x3041;
    step(&cpu, ram.bus());
    try std.testing.expectEqual(4, cpu.cycles);
    try std.testing.expectEqual(0xffffa678, cpu.addr[0]);
}
