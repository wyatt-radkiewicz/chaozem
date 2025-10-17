const std = @import("std");

pub fn build(b: *std.Build) void {
    // Standard compilation target info
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // M68k integration test compilation target info
    const m68k_query = std.Target.Query{
        .cpu_arch = .m68k,
        .cpu_model = .baseline,
        .os_tag = .freestanding,
        .abi = .eabi,
    };
    const m68k_integration_test_path = b.option(
        []const u8,
        "m68k-integration-tests",
        "M68k integration tests directory",
    ) orelse b.pathJoin(&.{ test_dir, m68k_mod_name, "integration" });
    const m68k_integration_linker_path = b.option(
        []const u8,
        "m68k-integration-linker",
        "Path to a m68k compatible linker",
    );
    const m68k_unit_test_path = b.option(
        []const u8,
        "m68k-unit-tests",
        "M68k unit test directory",
    ) orelse b.pathJoin(&.{ test_dir, m68k_mod_name, "unit" });

    // Major top level steps
    const install_step = b.getInstallStep();
    const test_step = b.step("test", "Run tests");
    _ = b.step("docs", "Emit documentation");
    const fmt_step = b.step("fmt", "Check formatting");
    const clean_step = b.step("clean", "Clean artifacts");

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

    // This tells us if we have m68k compilation support
    const have_m68k_support = get_m68k_support: {
        _ = std.zig.system.resolveTargetQuery(m68k_query) catch {
            std.log.warn("Unable to resolve m68k target, m68k integration tests disabled.", .{});
            break :get_m68k_support false;
        };
        if (m68k_integration_linker_path == null) {
            std.log.warn("No linker set for m68k target, m68k integration tests disabled.", .{});
            break :get_m68k_support false;
        } else {
            break :get_m68k_support true;
        }
    };
    if (have_m68k_support) {
        // Create the resolved target adn the linker path
        const m68k_target = b.resolveTargetQuery(m68k_query);
        const linker_path = m68k_integration_linker_path orelse unreachable;

        // Each integration test is its own executable since each one is compiled with different code
        // and interfaces. This will create a module for each test and return it
        var m68k_integration_test_dir = std.fs.cwd().openDir(
            m68k_integration_test_path,
            .{ .iterate = true },
        ) catch @panic("Couldn't open integration test directory");
        defer m68k_integration_test_dir.close();
        var m68k_integration_test_iter = m68k_integration_test_dir.iterate();
        while (m68k_integration_test_iter.next() catch @panic("Error iterating")) |entry| {
            // Only get source files (assume one for each test)
            if (!std.mem.eql(u8, ".zig", std.fs.path.extension(entry.name))) {
                continue;
            }

            // The name of the test we're running
            const test_name = std.fs.path.stem(entry.name);

            // Create the "m68k" integration tester
            const test_mod = b.createModule(.{
                .target = target,
                .optimize = optimize,
                .root_source_file = b.path(b.pathJoin(&.{
                    src_dir,
                    m68k_mod_name,
                    "test",
                    "integration.zig",
                })),
            });
            test_mod.addImport(m68k_mod_name, m68k_mod);
            test_mod.addImport(bus_mod_name, bus_mod);

            // Add this to the parent testing step
            const test_mod_exe = b.addTest(.{
                .name = b.fmt("m68k integration \"{s}\" test", .{test_name}),
                .root_module = test_mod,
            });
            const test_mod_run = b.addRunArtifact(test_mod_exe);
            test_step.dependOn(&test_mod_run.step);

            // The root source file and data file of the actual specific test to compile
            const test_ld = b.path(b.pathJoin(&.{ m68k_integration_test_path, "linker.ld" }));
            const test_zig = b.path(b.pathJoin(&.{ m68k_integration_test_path, entry.name }));
            const test_zon = b.path(b.pathJoin(&.{
                m68k_integration_test_path,
                b.fmt("{s}.zon", .{test_name}),
            }));
            test_mod.addAnonymousImport("input", .{
                .root_source_file = test_zon,
            });

            // Compile the host version of the test
            const host_mod = b.createModule(.{
                .target = target,
                .optimize = optimize,
                .root_source_file = test_zig,
            });
            test_mod.addImport("host", host_mod);

            // Compile the target version of the test
            const target_mod = b.createModule(.{
                .target = m68k_target,
                .optimize = .ReleaseSmall,
                .omit_frame_pointer = true,
                .unwind_tables = .none,
                .root_source_file = test_zig,
            });
            const target_obj = b.addObject(.{
                .name = test_name,
                .root_module = target_mod,
            });

            // Now that we have an object we'll link it with the provided linker.
            // This is a work around because as of now, Zig can not link m68k files
            const linker = b.addSystemCommand(&.{ linker_path, "-T" });
            linker.addFileArg(test_ld);
            linker.addFileArg(target_obj.getEmittedBin());
            linker.addArg("-o");
            const target_exe = linker.addOutputFileArg(b.fmt("{s}.elf", .{test_name}));

            // Set the dependencies for the linking step
            linker.addFileInput(target_obj.getEmittedBin());
            linker.addFileInput(test_ld);
            linker.step.dependOn(&target_obj.step);

            // Create the output rom binary and import it
            const target_rom = b.addObjCopy(target_exe, .{
                .basename = test_name,
                .only_section = ".rom",
                .format = .bin,
            });
            target_rom.step.dependOn(&linker.step);
            test_mod.addAnonymousImport("target_rom", .{
                .root_source_file = target_rom.getOutput(),
            });

            // Create the output ram binary and import it
            const target_ram = b.addObjCopy(target_exe, .{
                .basename = test_name,
                .only_section = ".ram",
                .format = .bin,
            });
            target_ram.step.dependOn(&linker.step);
            test_mod.addAnonymousImport("target_ram", .{
                .root_source_file = target_ram.getOutput(),
            });
        }
    }

    // Check formatting
    const fmt = b.addFmt(.{ .check = true, .paths = &.{
        "src/",
        "test/",
        "build.zig",
        "build.zig.zon",
    } });
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
