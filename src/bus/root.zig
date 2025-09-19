const std = @import("std");

const page_table = @import("page");

/// Used to define the width of the types on the bus
pub const Width = struct {
    /// Native address bus width
    addr: comptime_int,

    /// Native data bus width
    data: comptime_int,

    /// Address type
    pub fn Addr(this: @This()) type {
        return std.meta.Int(.unsigned, this.addr);
    }

    /// Data type to be passed on addresses and used for signals
    pub fn Data(this: @This()) type {
        return std.meta.Int(.unsigned, this.data);
    }
    
    /// Byte mask type
    pub fn Mask(this: @This()) type {
        return std.meta.Int(.unsigned, @divExact(this.data, 8));
    }
};

/// A bus device mapping, these are passed into a bus at compile time in order to create
/// a fast look up table for getting bus devices.
pub fn Mapping(comptime width: Width) type {
    return struct {
        /// Where in the memory map does this device start servicing requests?
        start: width.Addr(),

        /// Size of the memory region before it starts mirroring (minus 1).
        /// This must be a multiple of two otherwise mirroring won't work.
        size: width.Addr(),

        /// What is the end (inclusive) of this memory region? It will mirror `size` until this end.
        end: ?width.Addr() = null,
    };
}

/// Interface for devices on a bus network
/// - Addresses passed into the functions here are relative to the first mirror of the device's
/// mapping
/// - The mask is the byte mask of the bytes that are cared about in the
/// transaction on the bus.
pub fn Device(comptime width: Width) type {
    return struct {
        /// This function is used to get data from the device.
        read: ?*const fn (*@This(), width.Addr(), width.Mask()) ?width.Data() = null,

        /// This function is used to write data to the device.
        write: ?*const fn (*@This(), width.Addr(), width.Mask(), width.Data()) void = null,
    };
}

/// This is the bus network and also interface for the bus
pub fn Bus(comptime width: Width) type {
    return struct {
        devices: []const *Device(width),
        mappings: []const Mapping(width),
        map: *const fn (width.Addr()) ?usize,

        const This = @This();

        /// Initialize a new bus network with the following mapping and devices. A mapping must be
        /// specified at compile time to create a fast look up table. Device interfaces are provided
        /// at runtime and correspond with the mappings provided.
        pub fn init(comptime map: []const Mapping(width), devices: []const *Device(width)) This {
            // Get the mapping page table function
            const mapFn = page_table.table(.{
                .Index = width.Addr(),
                .Entry = ?usize,
                .max_pages = 1024,
                .page_size = 8,
            }, struct {
                pub fn get(_: @This(), addr: width.Addr()) ?usize {
                    // Try to see if any of the mappings overlap with the address else return null
                    @setEvalBranchQuota(map.len * 10000);
                    return for (map, 0..) |mapping, mapping_idx| {
                        if (addr >= mapping.start and addr <= mapping.end orelse
                            mapping.start + mapping.size - 1)
                        {
                            break mapping_idx;
                        }
                    } else null;
                }
            }{});
            return .{
                .devices = devices,
                .mappings = map,
                .map = mapFn,
            };
        }

        /// Read some data from the bus
        pub fn read(this: This, addr: width.Addr(), mask: width.Mask()) ?width.Data() {
            const index = this.map(addr) orelse return null;
            const rd = this.devices[index].read orelse return null;
            const mapping = this.mappings[index];
            return rd(this.devices[index], (addr - mapping.start) & (mapping.size - 1), mask);
        }

        /// Write some data to the bus
        pub fn write(this: This, addr: width.Addr(), mask: width.Mask(), data: width.Data()) void {
            const index = this.map(addr) orelse return null;
            const wr = this.devices[index].write orelse return null;
            const mapping = this.mappings[index];
            wr(this.devices[index], (addr - mapping.start) & (mapping.size - 1), mask, data);
        }

        /// Get a reader interface starting reading at `addr` spot.
        /// Since the reader buffers in data in chunks of the data width, an endianess must
        /// be provided for the bus data. Due to this, the buffer provided must be at least of the
        /// length of the data bus width. The `step` value is how much to change `addr` by every
        /// time data from the bus is written to the buffer.
        pub fn reader(
            this: *const This,
            addr: width.Addr(),
            buffer: []u8,
            endian: std.builtin.Endian,
            step: width.Addr(),
        ) Reader {
            return Reader{
                .bus = this,
                .addr = addr,
                .step = step,
                .endian = endian,
                .interface = .{
                    .vtable = &.{
                        .stream = Reader.stream,
                    },
                    .buffer = buffer,
                    .seek = 0,
                    .end = 0,
                },
            };
        }

        /// The bus bytes reader interface implementation
        const Reader = struct {
            bus: *const This,
            addr: width.Addr(),
            step: width.Addr(),
            endian: std.builtin.Endian,
            interface: std.io.Reader,

            /// Standard reader stream implementation
            pub fn stream(
                interface: *std.Io.Reader,
                writer: *std.Io.Writer,
                limit: std.Io.Limit,
            ) std.Io.Reader.StreamError!usize {
                const this: *@This() = @alignCast(@fieldParentPtr("interface", interface));
                const size = width.data / 8;
                const dest = limit.slice(try writer.writableSliceGreedy(size));
                std.debug.assert(dest.len % size == 0);

                for (0..dest.len / size) |i| {
                    const data = this.bus.read(this.addr, std.math.maxInt(width.Mask())) orelse 0;
                    std.mem.writeInt(width.Data(), dest[i * size ..][0..size], data, this.endian);
                    this.addr +%= this.step;
                }
                return dest.len;
            }
        };
    };
}
