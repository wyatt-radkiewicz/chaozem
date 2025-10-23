//! Rom and ram chips for sega genesis
const std = @import("std");

const bus_interface = @import("bus");
const Width = bus_interface.Width;

/// What type of chip to use
pub const Type = enum {
    /// Read only (ROM)
    ro,

    /// Read and write (RAM)
    rw,
};

/// Create a rom/ram chip width 'width'
pub fn Rxm(comptime rxm: Type, comptime width: Width) type {
    // Create aliases
    const Device = bus_interface.Device(width);
    const Addr = width.Addr();
    const Mask = width.Mask();
    const Data = width.Data();
    const Buffer = switch (rxm) {
        .ro => []const Data,
        .rw => []Data,
    };

    // Generate mask values for writing
    const bus_mask = comptime blk: {
        var masks: [1 << @bitSizeOf(Mask)]Data = undefined;
        for (0..masks.len) |pattern| {
            var mask: Data = 0;
            for (0..@bitSizeOf(Mask)) |bit| {
                mask |= if (pattern & 1 << bit != 0) 0xff << bit * 8 else 0;
            }
            masks[pattern] = mask;
        }
        break :blk masks;
    };

    // Return the struct
    return struct {
        words: Buffer,
        device: Device,

        /// Initialize ram, refrences buffer for the lifetime of this device
        pub fn init(buffer: Buffer) @This() {
            return .{
                .words = buffer,
                .device = .{ .read = read, .write = switch (rxm) {
                    .ro => null,
                    .rw => write,
                } },
            };
        }

        /// Read implementation
        pub fn read(device: *Device, addr: Addr, _: Mask) ?Data {
            const this: *@This() = @fieldParentPtr("device", device);
            return if (addr < this.words.len) this.words[addr] else null;
        }

        /// Write implementation
        pub fn write(device: *Device, addr: Addr, mask: Mask, data: Data) void {
            const this: *@This() = @fieldParentPtr("device", device);
            switch (rxm) {
                .ro => {},
                .rw => if (addr < this.words.len) {
                    const valid = bus_mask[mask];
                    this.words[addr] &= ~valid;
                    this.words[addr] |= data & valid;
                },
            }
        }
    };
}
