const std = @import("std");

pub fn build(b: *std.Build) void {
    // Standard compilation target info
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // M68k integration test compilation target info
    const m68k_integration_test_path = b.option(
        []const u8,
        "m68k-integration-tests",
        "M68k integration tests directory",
    ) orelse b.pathJoin(&.{ test_dir, m68k_mod_name, "integration" });
    const m68k_integration_compiler_path = b.option(
        []const u8,
        "m68k-integration-compiler",
        "Path to a gnu compatible compiler for m68k",
    );
    const m68k_integration_format = b.option([]const u8, "m68k-integration-format",
        \\The formatting before each intruction is run.
        \\The info printed is formatted using a formatting string with "{}" like in zig
        \\Here are the format specifiers:
        \\\t{pc}                Program counter
        \\\t{i}                 Print instruction disassembly
        \\\t{d:n}               Data registers
        \\\t{a:n}               Address registers
        \\\t{sp:n}              Stack pointers
        \\\t{sr},{sr:_}         Status register or specific status register's flag '_'
        \\\t{stop}              Stop status
        \\\t{b:n-m}             Print bytes from n to m
        \\\t{st},{st:n}         Print up to 'n' bytes of stack
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

    // Create the common "Test" type for the integration tester
    const m68k_integration_test_format_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{
            m68k_integration_test_path, "scripts", "Test.zig",
        })),
    });

    // Get where to store the integration tests
    const m68k_integration_gen_path = b.pathJoin(&.{ m68k_integration_test_path, "gen" });

    // Create the "m68k" integration tester
    std.fs.cwd().makeDir(m68k_integration_gen_path) catch |err| {
        if (err != error.PathAlreadyExists) {
            std.log.err("Could not create m68k integration test generation directory", .{});
        }
    };
    const m68k_integration_test_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path(b.pathJoin(&.{
            src_dir,
            m68k_mod_name,
            "test",
            "integration.zig",
        })),
    });
    m68k_integration_test_mod.addImport(m68k_mod_name, m68k_mod);
    m68k_integration_test_mod.addImport(bus_mod_name, bus_mod);
    m68k_integration_test_mod.addImport("Test", m68k_integration_test_format_mod);

    // Add the config options
    const m68k_integration_test_options = b.addOptions();
    m68k_integration_test_options.addOption([]const u8, "tests_path", m68k_integration_gen_path);
    m68k_integration_test_mod.addOptions("config", m68k_integration_test_options);

    // Add this to the parent testing step
    const m68k_integration_test_exe = b.addTest(.{
        .name = "m68k integration test",
        .root_module = m68k_integration_test_mod,
    });
    const m68k_integration_test_run = b.addRunArtifact(m68k_integration_test_exe);
    if (m68k_integration_format) |format| {
        m68k_integration_test_run.setEnvironmentVariable("M68K_INTEGRATION_FORMAT", format);
    }
    test_step.dependOn(&m68k_integration_test_run.step);

    // Generate the integration tests
    if (m68k_integration_compiler_path) |compiler_path| {
        // Each integration test is its own executable since each one is compiled with different
        // code and interfaces. This will create a module for each test and return it
        var dir = std.fs.cwd().openDir(
            m68k_integration_test_path,
            .{ .iterate = true },
        ) catch @panic("Couldn't open integration test directory");
        defer dir.close();
        var iter = dir.iterate();
        while (iter.next() catch @panic("Error iterating")) |entry| {
            // Only get source files (assume one for each test)
            if (!std.mem.eql(u8, ".c", std.fs.path.extension(entry.name))) {
                continue;
            }

            // The name of the test we're running
            const test_name = std.fs.path.stem(entry.name);

            // Get the directory of the helper scripts and some other commonly used paths
            const scripts_dir = b.pathJoin(&.{ m68k_integration_test_path, "scripts" });
            const test_src = b.path(b.pathJoin(&.{ m68k_integration_test_path, entry.name }));
            const linker_src = b.path(b.pathJoin(&.{ scripts_dir, "linker.ld" }));

            // Compile the generator for this test
            const gen_mod = b.createModule(.{
                .target = target,
                .optimize = optimize,
                .root_source_file = b.path(b.pathJoin(&.{ scripts_dir, "gen.zig" })),
            });
            gen_mod.addIncludePath(b.path(m68k_integration_test_path));
            gen_mod.addCSourceFile(.{ .language = .c, .file = test_src });
            gen_mod.addImport("Test", m68k_integration_test_format_mod);

            // Add a run step to run it during our tests and make the integration tests depend on
            // this running
            const gen_exe = b.addExecutable(.{
                .name = b.fmt("{s}_gen", .{test_name}),
                .root_module = gen_mod,
            });
            const gen_run = b.addRunArtifact(gen_exe);
            gen_run.addFileInput(b.path(b.pathJoin(&.{
                m68k_integration_test_path,
                b.fmt("{s}.zon", .{test_name}),
            })));
            m68k_integration_test_run.step.dependOn(&gen_run.step);

            // Add the config options
            const options = b.addOptions();
            options.addOption([]const u8, "tests_path", m68k_integration_test_path);
            options.addOption([]const u8, "gen_path", m68k_integration_gen_path);
            options.addOption([]const u8, "name", test_name);
            gen_mod.addOptions("config", options);

            // Compile the target version of the test
            const compile = b.addSystemCommand(
                &.{ compiler_path, "-nostdlib", "-nostartfiles", "-ffreestanding", "-Os" },
            );
            compile.addFileArg(test_src);
            compile.addFileInput(test_src);
            compile.addArg(b.fmt("-I{s}", .{scripts_dir}));
            compile.addArg("-T");
            compile.addFileArg(linker_src);
            compile.addFileInput(linker_src);
            compile.addArg("-o");
            const target_exe = compile.addOutputFileArg(b.fmt("{s}.elf", .{test_name}));

            // Create the output rom binary and import it
            const target_rom = b.addObjCopy(target_exe, .{
                .basename = b.fmt("{s}_rom.bin", .{test_name}),
                .only_section = ".rom",
                .format = .bin,
            });
            target_rom.step.dependOn(&compile.step);
            gen_mod.addAnonymousImport("target_rom", .{
                .root_source_file = target_rom.getOutput(),
            });

            // Create the output ram binary and import it
            const target_ram = b.addObjCopy(target_exe, .{
                .basename = b.fmt("{s}_ram.bin", .{test_name}),
                .only_section = ".ram",
                .format = .bin,
            });
            target_ram.step.dependOn(&compile.step);
            gen_mod.addAnonymousImport("target_ram", .{
                .root_source_file = target_ram.getOutput(),
            });
        }
    } else {
        std.log.warn("No compiler set for m68k target, m68k integration tests disabled.", .{});
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
    const remove_gen_dir = b.addRemoveDirTree(.{ .cwd_relative = m68k_integration_gen_path });
    const remove_cache_dir = b.addRemoveDirTree(b.path(".zig-cache"));
    clean_step.dependOn(&remove_install_dir.step);
    clean_step.dependOn(&remove_gen_dir.step);
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
