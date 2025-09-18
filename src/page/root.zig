//! Compile time flat sparse page table
const std = @import("std");

/// When creating a compile time page table, fill these out to finalize the structure
pub const Options = struct {
    /// What type is used to index the page table. It must be an unsigned integer.
    Index: type,

    /// What we get back from the page table. It must be an unsigned integer. It can be optional.
    Entry: type,

    /// The maximum amount of pages allowed to be allocated at comptime and thus runtime as well.
    max_pages: comptime_int,

    /// How big each 'page' of the page table is in bits. Note that the bit size of `Index` has to
    /// be a multiple of this.
    page_size: comptime_int = 4,

    /// Returns the type for an uncompressed page
    fn Page(comptime this: @This()) type {
        return [1 << this.page_size]?comptime_int;
    }
};

/// Debug info about a table
pub const Info = struct {
    /// For the entry type, whats the signedness?
    entry_signedness: std.builtin.Signedness,

    /// How many bits each entry in the table takes up
    entry_bits: u16,

    /// How many entries there are in the table
    table_len: usize,
};

/// Build a new page table at compile time.
/// - `options` used to configure the page table.
/// - `ctx` is a type that has the function:
///     pub fn get(this: @This(), Index) Entry
pub fn table(comptime options: Options, comptime ctx: anytype) fn (options.Index) options.Entry {
    return buildTable(options, ctx).getFn;
}

/// Get debug info about a page table
/// - `options` used to configure the page table.
/// - `ctx` is a type that has the function:
///     pub fn get(this: @This(), Index) Entry
pub fn info(comptime options: Options, comptime ctx: anytype) Info {
    const build_info = buildTable(options, ctx);
    return .{
        .entry_signedness = @typeInfo(build_info.Entry).int.signedness,
        .entry_bits = @typeInfo(build_info.Entry).int.bits,
        .table_len = build_info.len,
    };
}

/// Compile time info about the size of the generated table
fn BuildInfo(comptime options: Options) type {
    return struct {
        getFn: fn (options.Index) options.Entry,
        Entry: type,
        len: usize,
    };
}

/// Build a new page table at compile time, and also return compile time info about the table
/// - `options` used to configure the page table.
/// - `ctx` is a type that has the function:
///     pub fn get(this: @This(), Index) Entry
fn buildTable(comptime options: Options, comptime ctx: anytype) BuildInfo(options) {
    // Set the branch eval quota and check parameters
    @setEvalBranchQuota(options.max_pages * 100000);

    // Create the backing buffer so we can generate the first pass of the table
    var page_buffer: [options.max_pages]options.Page() = undefined;
    var pages = std.ArrayList(options.Page()).initBuffer(&page_buffer);

    // Create the sparse version of the table with `comptime_int`
    var min, var max = .{ 0, 0 };
    const page_size = blk: {
        const offset = @bitSizeOf(options.Index) % options.page_size;
        break :blk if (offset == 0) options.page_size else offset;
    };
    const top_level = visitPageLevel(options, ctx, &pages, &min, &max, void, page_size) orelse
        unreachable;

    // Check if the entry type the user supplied is optional
    const optional = switch (@typeInfo(options.Entry)) {
        .optional => true,
        else => false,
    };

    // This is the smallest type that can hold both the entries and page indexes
    const Entry = std.math.IntFittingRange(
        min,
        @max(pages.items.len - 1, max + @as(comptime_int, @intFromBool(optional))),
    );

    // Create a version of that table that has the smallest entry type possible
    var compressed: [pages.items.len << options.page_size]Entry = undefined;
    for (&compressed, 0..) |*entry, i| {
        entry.* = pages.items[i >> options.page_size][i % (1 << options.page_size)] orelse
            if (optional) max + 1 else 0;
    }
    const final = compressed;
    const @"null" = max + 1;

    // Return a small function that indexes this table at runtime
    return .{
        .getFn = struct {
            pub fn lut(index: options.Index) options.Entry {
                // This is the bit index of where the first level starts
                const first_level = @bitSizeOf(options.Index) - page_size;

                // Then for every page level in the page table, traverse it
                var entry = final[(top_level << options.page_size) + (index >> first_level)];
                inline for (1..std.math.divCeil(
                    comptime_int,
                    @bitSizeOf(options.Index),
                    options.page_size,
                ) catch unreachable) |level| {
                    const level_index: std.meta.Int(.unsigned, options.page_size) =
                        @truncate(index >> first_level - level * options.page_size);
                    entry = final[(@as(usize, @intCast(entry)) << options.page_size) + level_index];
                }
                return if (!optional) entry else if (entry == @"null") null else @intCast(entry);
            }
        }.lut,
        .Entry = Entry,
        .len = final.len,
    };
}

/// This will populate the nodes array with the page at that level with that prefix, and every one
/// under that page.
/// - `options` used in configuring the table
/// - `ctx` the context passed in that contains a function:
///     pub fn get(this: @This(), Index) Entry
/// - `pages` the list of uncompressed but unique pages in the table
/// - `min` the minimum entry value encountered
/// - `max` the maximum entry value encountered
/// - `prefix` what branch of the tree is the parent to the level we will calculate
fn visitPageLevel(
    comptime options: Options,
    comptime ctx: anytype,
    comptime pages: *std.ArrayList(options.Page()),
    comptime min: *comptime_int,
    comptime max: *comptime_int,
    comptime prefix: anytype,
    comptime page_size: comptime_int,
) ?comptime_int {
    // Get how long of a prefix we were given
    const prefix_len = if (@TypeOf(prefix) == void) 0 else @bitSizeOf(@TypeOf(prefix));

    // Check if the prefix is fully specialized
    if (prefix_len == @bitSizeOf(options.Index)) {
        // If so just return the entry index and set the min and max entries encountered
        const entry = ctx.get(@bitCast(prefix));
        switch (@typeInfo(options.Entry)) {
            .optional => {
                min.* = @min(min.*, entry orelse 0);
                max.* = @max(max.*, entry orelse 0);
            },
            else => {
                min.* = @min(min.*, entry);
                max.* = @max(max.*, entry);
            },
        }
        return switch (@typeInfo(options.Entry)) {
            .optional => if (entry) |value| @as(?comptime_int, value) else null,
            else => entry,
        };
    } else {
        // Create a new page entry since we are not fully specialized prefix
        var new_page = [1]?comptime_int{null} ** (1 << options.page_size);
        for (new_page[0 .. 1 << page_size], 0..) |*entry, i| {
            // Extend the prefix into an int that can also fit the the next page level
            var next_prefix = @as(
                std.meta.Int(.unsigned, prefix_len + page_size),
                if (prefix_len == 0) 0 else prefix,
            );
            if (prefix_len > 0) {
                next_prefix <<= page_size;
            }

            // Add what page entry we are actually pointing to
            next_prefix += @as(std.meta.Int(.unsigned, page_size), @intCast(i));

            // Get the entry for this page
            entry.* = visitPageLevel(options, ctx, pages, min, max, next_prefix, options.page_size);
        }

        // Either find that this page is already refrenced in the orignial pages array, or add it
        for (pages.items, 0..) |page, idx| {
            if (std.mem.eql(?comptime_int, &page, &new_page)) {
                return idx;
            }
        }

        // Add the new page
        pages.appendAssumeCapacity(new_page);
        return pages.items.len - 1;
    }
}
