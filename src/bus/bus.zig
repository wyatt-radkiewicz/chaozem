//! Bus interface for the system no actual code make things work here, this is just for the
//! abstraction of the system.
const std = @import("std");

/// This is the interface devices use on the bus to communicate with other devices. In other words,
/// this is the "consumer" side of the bus interface.
///
/// Implementors of this interface actually facilitate the interactions on the bus, but users are
/// the devices on the bus.
pub fn Bus(config: BusConfig) type {
    return struct {
        /// Implementation of the base "read" and "write" function
        readFn: *const fn (*@This(), Mask, Addr) Error!Data,
        writeFn: *const fn (*@This(), Mask, Addr, Data) Error!void,
        signalFn: *const fn (*@This(), RuntimeSignal(config)) void,

        /// Data types
        pub const Addr = config.Addr();
        pub const Data = config.Data();
        pub const Mask = config.Mask();

        /// Read data from the bus
        pub fn read(this: *@This(), mask: Mask, addr: Addr) Error!Data {
            return this.readFn(this, mask, addr);
        }

        /// Write data to the bus
        pub fn write(this: *@This(), mask: Mask, addr: Addr, data: Data) Error!void {
            try this.writeFn(this, mask, addr, data);
        }

        /// Assert a signal on the bus
        pub fn signal(this: *@This(), target_signal: RuntimeSignal(config)) void {
            this.signalFn(this, target_signal);
        }
    };
}

/// This is the device "producer" side of the bus. Devices on the bus can choose to implement this
/// interface.
///
/// The user of this interface is the glue the implentator of the `Bus` interface. The implementor
/// of this interface is a device on the bus.
pub fn Device(bus_config: BusConfig, device_config: DeviceConfig) type {
    return struct {
        /// Functions to handle reads, writes, and signals
        readFn: *const fn (*@This(), Mask, Addr) Error!Data = defaultRead,
        writeFn: *const fn (*@This(), Mask, Addr, Data) Error!void = defaultWrite,
        signalFn: *const fn (*@This(), device_config.Signal) void = defaultSignal,

        /// Data types
        pub const Addr = bus_config.Addr();
        pub const Data = bus_config.Data();
        pub const Mask = bus_config.Mask();

        /// Config for the bus
        pub const start: Addr = device_config.start;
        pub const end: Addr = device_config.end;

        /// Used to request a read from the device
        pub fn read(this: *@This(), mask: Mask, addr: Addr) Error!Data {
            return this.readFn(this, mask, (addr - start) & device_config.size);
        }

        /// Used to request to write data to the device
        pub fn write(this: *@This(), mask: Mask, addr: Addr, data: Data) Error!void {
            return this.writeFn(this, mask, (addr - start) & device_config.size, data);
        }

        /// Used to send a signal to this device
        pub fn sendSignal(this: *@This(), runtime_signal: RuntimeSignal(bus_config)) void {
            this.signalFn(this, switch (@typeInfo(device_config.Signal)) {
                .@"enum" => @enumFromInt(runtime_signal.data),
                else => @bitCast(@as(
                    std.meta.Int(.unsigned, @bitSizeOf(device_config.Signal)),
                    @truncate(runtime_signal.data),
                )),
            });
        }

        /// Make a signal for this device
        pub fn makeSignal(data: device_config.Signal) RuntimeSignal(bus_config) {
            return .{ .device_addr = start, .data = data };
        }

        /// Default implementations of the functions
        fn defaultSignal(_: *@This(), _: device_config.Signal) void {}
        fn defaultRead(_: *@This(), _: Mask, _: Addr) Error!Data {
            return Error;
        }
        fn defaultWrite(_: *@This(), _: Mask, _: Addr, _: Data) Error!void {
            return Error;
        }
    };
}

/// Open bus, no device responded to the bus request
pub const Error = error.OpenBus;

/// The config passed into bus consumers and producers
pub const BusConfig = struct {
    /// Native address bus width
    addr_width: comptime_int,

    /// Native data bus width
    data_width: comptime_int,

    /// Address type
    pub fn Addr(this: @This()) type {
        return std.meta.Int(.unsigned, this.addr_width);
    }

    /// Data type to be passed on addresses and used for signals
    pub fn Data(this: @This()) type {
        return std.meta.Int(.unsigned, this.data_width);
    }

    /// So sometimes we don't care for all of the bytes on the data bus. Here we can mask out the
    /// ones we want and don't want.
    pub fn Mask(this: @This()) type {
        return std.math.IntFittingRange(0, @divExact(this.data_width, 8) - 1);
    }
};

/// These are the options passed into devices
pub const DeviceConfig = struct {
    /// Where in the memory map does this device start servicing requests?
    start: comptime_int,

    /// Size of the memory region before it starts mirroring (minus 1).
    size: comptime_int,

    /// What is the end (inclusive) of this memory region? It will mirror `size` until this end.
    end: comptime_int,

    /// What type is used for signals. It has to be a type thats able to be converted to a bus
    /// integer.
    Signal: type,
};

/// Where a device is in a memory map
fn RuntimeSignal(config: BusConfig) type {
    return struct {
        /// What device this signal is for (set to the base address of the device)
        device_addr: config.Addr(),

        /// What the signal is. This can be any type, but it must be convertible to the bus data
        data: config.Data(),
    };
}
