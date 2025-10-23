//! Ram chip for sega genesis
const std = @import("std");

const bus_interface = @import("bus");
const Width = bus_interface.Width;

/// Create a ram chip width 'width'
pub fn Ram(comptime width: Width) type {
    // Create aliases
    const Device = bus_interface.Device(width);
    const Addr = width.Addr();
    const Mask = width.Mask();
    const Data = width.Data();

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
        words: []Data,
        device: Device,

        /// Initialize ram, refrences buffer for the lifetime of this device
        pub fn init(buffer: []Data) @This() {
            return .{
                .words = buffer,
                .device = .{ .read = read, .write = write },
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
            if (addr < this.words.len) {
                this.words[addr] = (this.words[addr] & ~bus_mask[mask]) | (data & bus_mask[mask]);
            }
        }
    };
}
