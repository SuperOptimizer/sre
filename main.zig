const std = @import("std");

/// CPU represents the LR35902 processor state
pub const CPU = struct {
    // 8-bit registers
    a: u8 = 0, // Accumulator
    b: u8 = 0,
    c: u8 = 0,
    d: u8 = 0,
    e: u8 = 0,
    h: u8 = 0,
    l: u8 = 0,

    // Program counter and stack pointer
    pc: u16 = 0,
    sp: u16 = 0,

    // Flag register (stored in high nibble of A register)
    flags: Flags = Flags{},

    // Memory bus
    bus: *Bus,

    // Clock cycles
    cycles: u64 = 0,

    pub const Flags = struct {
        z: bool = false, // Zero
        n: bool = false, // Subtract
        h: bool = false, // Half Carry
        c: bool = false, // Carry

        pub fn toByte(self: Flags) u8 {
            return @as(u8, if (self.z) 0x80 else 0) |
                @as(u8, if (self.n) 0x40 else 0) |
                @as(u8, if (self.h) 0x20 else 0) |
                @as(u8, if (self.c) 0x10 else 0);
        }

        pub fn fromByte(byte: u8) Flags {
            return Flags{
                .z = (byte & 0x80) != 0,
                .n = (byte & 0x40) != 0,
                .h = (byte & 0x20) != 0,
                .c = (byte & 0x10) != 0,
            };
        }
    };

    pub fn init(bus: *Bus) CPU {
        return CPU{
            .bus = bus,
        };
    }

    // Memory access helpers
    fn read(self: *CPU, addr: u16) u8 {
        self.cycles += 4;
        return self.bus.read(addr);
    }

    fn write(self: *CPU, addr: u16, value: u8) void {
        self.cycles += 4;
        self.bus.write(addr, value);
    }

    // Register pair access
    fn getBC(self: CPU) u16 {
        return (@as(u16, self.b) << 8) | self.c;
    }

    fn setBC(self: *CPU, value: u16) void {
        self.b = @truncate(value >> 8);
        self.c = @truncate(value);
    }

    fn getDE(self: CPU) u16 {
        return (@as(u16, self.d) << 8) | self.e;
    }

    fn setDE(self: *CPU, value: u16) void {
        self.d = @truncate(value >> 8);
        self.e = @truncate(value);
    }

    fn getHL(self: CPU) u16 {
        return (@as(u16, self.h) << 8) | self.l;
    }

    fn setHL(self: *CPU, value: u16) void {
        self.h = @truncate(value >> 8);
        self.l = @truncate(value);
    }

    // Stack operations
    fn push(self: *CPU, value: u16) void {
        self.sp -%= 1;
        self.write(self.sp, @truncate(value >> 8));
        self.sp -%= 1;
        self.write(self.sp, @truncate(value));
    }

    fn pop(self: *CPU) u16 {
        const low = self.read(self.sp);
        self.sp +%= 1;
        const high = self.read(self.sp);
        self.sp +%= 1;
        return (@as(u16, high) << 8) | low;
    }

    // Main execution function
    pub fn step(self: *CPU) void {
        const opcode = self.read(self.pc);
        self.pc +%= 1;

        switch (opcode) {
        // Here we implement all opcodes
        // NOP
            0x00 => {},

            // LD B,n
            0x06 => {
                self.b = self.read(self.pc);
                self.pc +%= 1;
            },

            // LD C,n
            0x0E => {
                self.c = self.read(self.pc);
                self.pc +%= 1;
            },

            // LD A,(BC)
            0x0A => {
                self.a = self.read(self.getBC());
            },

            // LD A,(DE)
            0x1A => {
                self.a = self.read(self.getDE());
            },

            // INC B
            0x04 => {
                self.flags.h = (self.b & 0x0F) == 0x0F;
                self.b +%= 1;
                self.flags.z = self.b == 0;
                self.flags.n = false;
            },

            // DEC B
            0x05 => {
                self.flags.h = (self.b & 0x0F) == 0;
                self.b -%= 1;
                self.flags.z = self.b == 0;
                self.flags.n = true;
            },

            // ADD A,B
            0x80 => {
                const result = @addWithOverflow(self.a, self.b);
                self.flags.h = ((self.a & 0x0F) + (self.b & 0x0F)) > 0x0F;
                self.flags.c = result[1] != 0;
                self.a = result[0];
                self.flags.z = self.a == 0;
                self.flags.n = false;
            },

            // JP nn
            0xC3 => {
                const low = self.read(self.pc);
                self.pc +%= 1;
                const high = self.read(self.pc);
                self.pc = (@as(u16, high) << 8) | low;
            },

            // CALL nn
            0xCD => {
                const low = self.read(self.pc);
                self.pc +%= 1;
                const high = self.read(self.pc);
                self.pc +%= 1;
                self.push(self.pc);
                self.pc = (@as(u16, high) << 8) | low;
            },

            // RET
            0xC9 => {
                self.pc = self.pop();
            },

            // Add more opcodes here...
            else => {
                std.debug.print("Unknown opcode: 0x{X:0>2}\n", .{opcode});
            },
        }
    }
};

/// Bus represents the memory bus of the system
pub const Bus = struct {
    // ROM (32KB)
    rom: [0x8000]u8 = [_]u8{0} ** 0x8000,

    // VRAM (8KB)
    vram: [0x2000]u8 = [_]u8{0} ** 0x2000,

    // External RAM (8KB)
    eram: [0x2000]u8 = [_]u8{0} ** 0x2000,

    // Working RAM (8KB)
    wram: [0x2000]u8 = [_]u8{0} ** 0x2000,

    // Zero-page RAM (127 bytes)
    zram: [0x7F]u8 = [_]u8{0} ** 0x7F,

    pub fn init() Bus {
        return Bus{};
    }

    pub fn read(self: Bus, addr: u16) u8 {
        return switch (addr) {
            0x0000...0x7FFF => self.rom[addr], // ROM
            0x8000...0x9FFF => self.vram[addr - 0x8000], // VRAM
            0xA000...0xBFFF => self.eram[addr - 0xA000], // External RAM
            0xC000...0xDFFF => self.wram[addr - 0xC000], // Working RAM
            0xFF80...0xFFFE => self.zram[addr - 0xFF80], // Zero-page RAM
            else => 0xFF, // Unmapped
        };
    }

    pub fn write(self: *Bus, addr: u16, value: u8) void {
        switch (addr) {
            0x0000...0x7FFF => {}, // ROM is read-only
            0x8000...0x9FFF => self.vram[addr - 0x8000] = value,
            0xA000...0xBFFF => self.eram[addr - 0xA000] = value,
            0xC000...0xDFFF => self.wram[addr - 0xC000] = value,
            0xFF80...0xFFFE => self.zram[addr - 0xFF80] = value,
            else => {}, // Unmapped
        }
    }
};

pub fn main() !void {
    var bus = Bus.init();
    var cpu = CPU.init(&bus);

    // Example program: Load value into B register and increment it
    bus.rom[0] = 0x06; // LD B,n
    bus.rom[1] = 0x42; // n = 0x42
    bus.rom[2] = 0x04; // INC B
    bus.rom[3] = 0x00; // NOP

    // Run for a few instructions
    var i: usize = 0;
    while (i < 4) : (i += 1) {
        cpu.step();
        std.debug.print("Step {}: B = 0x{X:0>2}\n", .{i, cpu.b});
    }
}