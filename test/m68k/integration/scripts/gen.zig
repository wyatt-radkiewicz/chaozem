//! Generates integration test info
//! Makes a zon file with rom words, setup ram words, and expected ram words
const std = @import("std");

const config = @import("config");
const Test = @import("Test");

const target_rom = @embedFile("target_rom");
const target_ram = @embedFile("target_ram");
const c = @cImport(@cInclude("test.h"));

/// Run the main conversion script
pub fn main() !void {
    const allocator = std.heap.smp_allocator;

    // Get the input
    const max_len: usize = 0x1000 * 0x1000 * 4;
    const input_path = try std.fs.path.join(allocator, &.{ config.tests_path, "input.zon" });
    defer allocator.free(input_path);
    const input_file = try std.fs.cwd().readFileAllocOptions(
        allocator,
        input_path,
        max_len,
        null,
        .@"16",
        0,
    );
    defer allocator.free(input_file);
    const inputs = try std.zon.parse.fromSlice([]const c.Input, allocator, input_file, null, .{});
    defer std.zon.parse.free(allocator, inputs);

    // Round size of input and output to 2
    const input_len = @divFloor(@sizeOf(c.Input) + 1, 2) * 2 + 2;
    const output_len = @divFloor(@sizeOf(c.Output) + 1, 2) * 2 + 2;
    const ram_len = @divFloor(target_ram.len + 1, 2) * 2 + 2;
    const rom_len = @divFloor(target_rom.len + 1, 2) * 2 + 2;

    // Generate base rom image
    var rom: [rom_len + 2]u8 = undefined;
    @memcpy(rom[0..target_rom.len], target_rom);
    std.mem.writeInt(u16, rom[rom_len..][0..2], 0x4E72, .big);

    // Generate base stack image
    var stack: [12]u8 = undefined;
    std.mem.writeInt(u32, stack[0..][0..4], rom_len, .big);
    std.mem.writeInt(u32, stack[4..][0..4], 0xffff0000 + ram_len, .big);
    std.mem.writeInt(u32, stack[8..][0..4], 0xffff0000 + input_len + ram_len, .big);

    // Generate each test case
    const cases = try allocator.alloc(Test.Case, inputs.len);
    defer allocator.free(cases);
    const setups = try allocator.alloc([@sizeOf(c.Input)]u8, inputs.len);
    defer allocator.free(setups);
    const expects = try allocator.alloc([@sizeOf(c.Output)]u8, inputs.len);
    defer allocator.free(expects);
    for (0..inputs.len) |i| {
        // Generate the expect state
        var output: c.Output = undefined;
        c.run(&inputs[i], &output);
        var writer = std.io.Writer.fixed(&expects[i]);
        try writer.writeStruct(output, .big);

        // Setup the input
        writer = std.io.Writer.fixed(&setups[i]);
        try writer.writeStruct(inputs[i], .big);

        // Setup the case
        cases[i].setup = &setups[i];
        cases[i].expect = &expects[i];
    }

    // Create the output
    const output_path = try std.fs.path.join(allocator, &.{
        config.gen_path,
        std.fmt.comptimePrint("{s}.zon", .{config.name}),
    });
    defer allocator.free(output_path);
    var output_file = try std.fs.cwd().createFile(output_path, .{});
    var buffer: [128]u8 = undefined;
    var writer = output_file.writer(&buffer);
    try std.zon.stringify.serialize(Test{
        .rom = &rom,
        .ram = target_ram,
        .stack = &stack,
        .setup_base = ram_len,
        .expect_base = ram_len + input_len,
        .ram_end = ram_len + input_len + output_len,
        .cases = cases,
    }, .{}, &writer.interface);
    try writer.interface.flush();
    output_file.close();
}
