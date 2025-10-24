//! Generates integration test info
//! Makes a zon file with rom words, setup ram words, and expected ram words
const std = @import("std");

const config = @import("config");
const Test = @import("Test");

const target_rom = @embedFile("target_rom");
const target_ram = @embedFile("target_ram");
const c = @cImport({
    @cInclude("test.h");
    @cInclude("run.h");
});

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
    const input_len = @divFloor(@sizeOf(c.Input) + 1, 2);
    const output_len = @divFloor(@sizeOf(c.Output) + 1, 2);
    const ram_len = @divExact(target_ram.len, 2);
    const rom_len = @divExact(target_rom.len, 2);

    // Generate base rom image
    var rom: [rom_len + 1]u16 = undefined;
    var reader = std.io.Reader.fixed(target_rom);
    try reader.readSliceEndian(u16, rom[0..rom_len], .big);

    // Add stop instruction
    rom[rom_len] = 0x4E72;

    // Generate base ram image
    var ram: [ram_len]u16 = undefined;
    reader = std.io.Reader.fixed(target_ram);
    try reader.readSliceEndian(u16, &ram, .big);

    // Generate base stack image
    var stack_bytes: [12]u8 = undefined;
    std.mem.writeInt(u32, stack_bytes[0..][0..4], rom_len * 2, .big);
    std.mem.writeInt(u32, stack_bytes[4..][0..4], 0xffff0000 + ram_len * 2, .big);
    std.mem.writeInt(u32, stack_bytes[8..][0..4], 0xffff0000 + (input_len + ram_len) * 2, .big);
    var stack: [6]u16 = undefined;
    reader = std.io.Reader.fixed(&stack_bytes);
    try reader.readSliceEndian(u16, &stack, .big);

    // Generate each test case
    const cases = try allocator.alloc(Test.Case, inputs.len);
    defer {
        for (cases) |case| {
            allocator.free(case.name);
            allocator.free(case.setup);
            allocator.free(case.expect);
        }
        allocator.free(cases);
    }
    for (0..inputs.len) |i| {
        // Generate the expect state
        var output: c.Output = undefined;
        c.run(&inputs[i], &output);

        // Get the bytes of the output
        var output_bytes = [1]u8{0} ** (output_len * 2);
        var writer = std.io.Writer.fixed(&output_bytes);
        try writer.writeStruct(output, .big);

        // Get the words of the output
        const expect = try allocator.alloc(u16, output_len);
        reader = std.io.Reader.fixed(&output_bytes);
        try reader.readSliceEndian(u16, expect, .big);

        // Setup the input bytes
        var input_bytes = [1]u8{0} ** (input_len * 2);
        writer = std.io.Writer.fixed(&input_bytes);
        try writer.writeStruct(inputs[i], .big);

        // Get the input words
        const setup = try allocator.alloc(u16, input_len);
        reader = std.io.Reader.fixed(&input_bytes);
        try reader.readSliceEndian(u16, setup, .big);

        // Generate a name for the test
        var name = std.io.Writer.Allocating.init(allocator);
        try name.writer.print("{any}", .{inputs[i]});

        // Setup the case
        cases[i].name = try name.toOwnedSlice();
        cases[i].setup = setup;
        cases[i].expect = expect;
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
        .ram = &ram,
        .stack = &stack,
        .setup_base = ram_len,
        .expect_base = ram_len + input_len,
        .ram_end = ram_len + input_len + output_len,
        .cases = cases,
    }, .{}, &writer.interface);
    try writer.interface.flush();
    output_file.close();
}
