const std = @import("std");

pub fn build(b: *std.Build) void {
    // Standard compilation target info
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // M68k integration test compilation target info
    const m68k_target = b.resolveTargetQuery(.{
        .cpu_arch = .m68k,
        .cpu_model = .baseline,
        .os_tag = .freestanding,
        .abi = .eabi,
    });
    _ = m68k_target; // autofix
    const m68k_integration_test_path = b.option(
        []const u8,
        "m68k-integration-tests",
        "M68k integration tests directory",
    ) orelse b.pathJoin(&.{ test_dir, m68k_mod_name, "integration" });
    _ = m68k_integration_test_path; // autofix
    const m68k_unit_test_path = b.option(
        []const u8,
        "m68k-unit-tests",
        "M68k unit test directory",
    ) orelse b.pathJoin(&.{ test_dir, m68k_mod_name, "unit" });

    // Major top level steps
    const install_step = b.getInstallStep();
    const test_step = b.step("test", "Run tests");
    const docs_step = b.step("docs", "Emit documentation");
    const fmt_step = b.step("fmt", "Check formatting");
    const clean_step = b.step("clean", "Clean artifacts");
    _ = docs_step; // autofix

    // Compile the "int" module and tests
    const int_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{ src_dir, int_mod_name, "root.zig" })),
    });
    const int_test_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{ src_dir, int_mod_name, "test.zig" })),
    });
    const int_test_exe = b.addTest(.{
        .name = b.fmt("\"{s}\" module tests", .{int_mod_name}),
        .root_module = int_test_mod,
    });
    const int_test_run = b.addRunArtifact(int_test_exe);
    test_step.dependOn(&int_test_run.step);

    // Compile the "page" module and tests
    const page_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{ src_dir, page_mod_name, "root.zig" })),
    });
    const page_test_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{ src_dir, page_mod_name, "test.zig" })),
    });
    const page_test_exe = b.addTest(.{
        .name = b.fmt("\"{s}\" module tests", .{page_mod_name}),
        .root_module = page_test_mod,
    });
    const page_test_run = b.addRunArtifact(page_test_exe);
    test_step.dependOn(&page_test_run.step);

    // Compile the "bus" module and tests
    const bus_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{ src_dir, bus_mod_name, "root.zig" })),
    });
    bus_mod.addImport(page_mod_name, page_mod);

    const bus_test_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{ src_dir, bus_mod_name, "test.zig" })),
    });
    bus_test_mod.addImport(page_mod_name, page_mod);

    const bus_test_exe = b.addTest(.{
        .name = b.fmt("\"{s}\" module tests", .{bus_mod_name}),
        .root_module = bus_test_mod,
    });
    const bus_test_run = b.addRunArtifact(bus_test_exe);
    test_step.dependOn(&bus_test_run.step);

    // Create the "m68k" module
    const m68k_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{ src_dir, m68k_mod_name, "root.zig" })),
    });
    m68k_mod.addImport(int_mod_name, int_mod);
    m68k_mod.addImport(bus_mod_name, bus_mod);
    m68k_mod.addImport(page_mod_name, page_mod);

    // Create the "m68k" unit tester
    const m68k_unit_test_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{ src_dir, m68k_mod_name, "test", "unit.zig" })),
    });
    m68k_unit_test_mod.addImport(m68k_mod_name, m68k_mod);
    m68k_unit_test_mod.addImport(bus_mod_name, bus_mod);

    // Add the config options
    const m68k_unit_test_options = b.addOptions();
    m68k_unit_test_options.addOption([]const u8, "tests_path", m68k_unit_test_path);
    m68k_unit_test_mod.addOptions("config", m68k_unit_test_options);

    // Build the test executable and add it to the "test" step
    const m68k_unit_test_exe = b.addTest(.{
        .name = "\"m68k\" module unit tests",
        .root_module = m68k_unit_test_mod,
    });
    const m68k_unit_test_run = b.addRunArtifact(m68k_unit_test_exe);
    test_step.dependOn(&m68k_unit_test_run.step);

    // Check formatting
    const fmt = b.addFmt(.{
        .check = true,
        .paths = &.{
            "src/",
            "test/",
            "build.zig",
            "build.zig.zon",
        },
    });
    fmt_step.dependOn(&fmt.step);
    install_step.dependOn(fmt_step);

    // Clean artifacts
    const remove_install_dir = b.addRemoveDirTree(.{ .cwd_relative = b.install_path });
    const remove_cache_dir = b.addRemoveDirTree(b.path(".zig-cache"));
    clean_step.dependOn(&remove_install_dir.step);
    if (@import("builtin").os.tag != .windows) {
        clean_step.dependOn(&remove_cache_dir.step);
    }
}

/// Constants
const src_dir = "src";
const test_dir = "test";
const int_mod_name = "int";
const page_mod_name = "page";
const bus_mod_name = "bus";
const m68k_mod_name = "m68k";
