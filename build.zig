const std = @import("std");

const Options = struct {
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,

    fn init(b: *std.Build) @This() {
        return .{
            .target = b.standardTargetOptions(.{}),
            .optimize = b.standardOptimizeOption(.{}),
        };
    }
};

const Module = struct {
    module: *std.Build.Module,
    name: []const u8,

    fn init(
        b: *std.Build,
        name: []const u8,
        opts: Options,
        test_step: ?*std.Build.Step,
        imports: []const *const Module,
    ) !*@This() {
        const mod_imports = try b.allocator.alloc(std.Build.Module.Import, imports.len);
        for (mod_imports, imports) |*mod_import, import| {
            mod_import.* = .{
                .name = import.name,
                .module = import.module,
            };
        }
        const mod = try b.allocator.create(@This());
        mod.* = .{
            .module = b.createModule(.{
                .root_source_file = b.path(b.pathJoin(&.{
                    "src",
                    name,
                    b.fmt("{s}.zig", .{name}),
                })),
                .imports = mod_imports,
                .target = opts.target,
                .optimize = opts.optimize,
            }),
            .name = name,
        };
        if (test_step) |tests| {
            const dir = try std.fs.cwd().openDir(b.pathJoin(&.{ "src", name }), .{});
            if (dir.statFile("test.zig")) |_| {} else |_| {
                return mod;
            }

            const test_exe = b.addTest(.{
                .name = b.fmt("{s}_test", .{name}),
                .root_module = b.createModule(.{
                    .root_source_file = b.path(b.pathJoin(&.{ "src", name, "test.zig" })),
                    .imports = mod_imports,
                    .target = opts.target,
                    .optimize = opts.optimize,
                }),
            });

            const test_run = b.addRunArtifact(test_exe);
            tests.dependOn(&test_run.step);
        }
        return mod;
    }
};

pub fn build(b: *std.Build) !void {
    const opts = Options.init(b);
    const test_step = b.step("test", "run the module tests");
    const page_mod = try Module.init(b, "page", opts, test_step, &.{});
    _ = try Module.init(b, "bus", opts, test_step, &.{page_mod});
}
