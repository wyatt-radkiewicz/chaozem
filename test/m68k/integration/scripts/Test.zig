//! Each test has this pattern

/// Rom image
rom: []const u8,

/// Base ram image
ram: []const u8,

/// What the stack should be initialized to
stack: []const u8,

/// This is where to start injecting setup bytes to
setup_base: usize,

/// This is where to start checking the output from
expect_base: usize,

/// This is the address after the end of the output buffer
ram_end: usize,

/// Each test overwrites data at `setup_base` with setup and checks starting at `expect_base` for
/// expect
cases: []const Case,

/// A case of a test
pub const Case = struct {
    /// Name of the case
    name: []const u8,

    /// Setup bytes
    setup: []const u8,

    /// Check the expected bytes for this
    expect: []const u8,
};
