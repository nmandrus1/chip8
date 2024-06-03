const std = @import("std");

// CHIP 8 Emulator
//
// TODO:
// Basic Struct Definitions
// instruction decoding
// fetch, decode, execute fns
// ROM processing/loading

const CPU = struct {
    // 0x000 - 0x200 = Stack
    // 0x200 - 0xFFF = RAM
    memory: [4096]u8,

    // program counter & stack ptr
    pc: u12, // program start is 0x200
    sp: u8, // starts at 0x200 and grows down

    registers: [16]u8,

    fn init() CPU {
        const cpu = CPU{
            .pc = 0x200,
            .sp = 0,
            .memory = [_]u8{0} ** 4096,
            .registers = [_]u8{0} ** 16,
        };

        return cpu;
    }

    // push a 2 byte value to the stack
    fn stack_push(self: *CPU, value: u16) void {
        self.memory[self.sp] = @truncate(value >> 8);
        self.memory[self.sp + 1] = @truncate(value);
        self.sp += 2;
    }

    // pop a 2 byte value from the stack
    fn stack_pop(self: *CPU) u16 {
        self.sp -= 2;
        const msb = @as(u16, self.memory[self.sp]) << 8;
        const lsb = @as(u16, self.memory[self.sp + 1]);
        return msb | lsb;
    }

    // read byte and move pc
    fn fetch(self: *CPU) u16 {
        defer self.pc += 2;
        const msb = @as(u16, self.memory[self.pc]) << 8;
        const lsb = @as(u16, self.memory[self.pc + 1]);
        return msb | lsb;
    }

    fn decode(self: *CPU, instr: u16) void {
        const nibble = instr & 0xF000;
        switch (nibble) {
            0x0000 => {
                switch (instr) {
                    // CLS
                    0x00E0 => std.debug.print("CLS Instruction Unimplemented!\n", .{}),
                    // RET
                    0x00EE => self.jump(@truncate(self.stack_pop())),
                    else => self.jump(@truncate(instr)),
                }
            },

            // jp nnn
            0x1000 => self.jump(@truncate(instr)),

            // call subroutine
            0x2000 => {
                self.stack_push(self.pc);
                self.jump(@truncate(instr));
            },

            // 3xkk - SE Vx, byte
            0x3000 => {
                const x = instr & 0x0F00;
                const kk = instr & 0x00FF;

                if (self.registers[x] == kk) {
                    self.pc += 2;
                }
            },

            // 4xkk - SNE Vx, byte
            0x4000 => {
                const x = instr & 0x0F00;
                const kk = instr & 0x00FF;

                if (self.registers[x] != kk) {
                    self.pc += 2;
                }
            },

            // 5xy0 - SE Vx, Vy
            0x5000 => {
                const x = instr & 0x0F00;
                const y = instr & 0x00F0;

                if (self.registers[x] == self.registers[y]) {
                    self.pc += 2;
                }
            },

            // 6xkk - LD Vx, byte
            0x6000 => {
                const x = instr & 0x0F00;
                const kk = instr & 0x00FF;
                self.registers[x] = @truncate(kk);
            },

            0x7000 => {},

            0x8000 => {},

            0x9000 => {},

            0xA000 => {},

            else => std.process.exit(1),
        }
    }

    // set pc to given address
    fn jump(self: *CPU, addr: u12) void {
        self.pc = addr;
    }
};

pub fn main() !void {
    // Prints to stderr (it's a shortcut based on `std.io.getStdErr()`)
    std.debug.print("All your {s} are belong to us.\n", .{"codebase"});

    // stdout is for the actual output of your application, for example if you
    // are implementing gzip, then only the compressed bytes should be sent to
    // stdout, not any debugging messages.
    const stdout_file = std.io.getStdOut().writer();
    var bw = std.io.bufferedWriter(stdout_file);
    const stdout = bw.writer();

    try stdout.print("Run `zig build test` to run the tests.\n", .{});

    try bw.flush(); // don't forget to flush!
}

test "stack_push and stack_pop" {
    var cpu = CPU.init();

    // Test stack_push
    cpu.stack_push(0xABCD);
    try std.testing.expectEqual(0xAB, cpu.memory[0]);
    try std.testing.expectEqual(0xCD, cpu.memory[1]);
    try std.testing.expectEqual(2, cpu.sp);

    // Test stack_pop
    const value = cpu.stack_pop();
    try std.testing.expectEqual(0xABCD, value);
    try std.testing.expectEqual(0, cpu.sp);
}

test "fetch instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x12;
    cpu.memory[0x201] = 0x34;
    cpu.pc = 0x200;

    const instr = cpu.fetch();

    try std.testing.expectEqual(@as(u16, 0x1234), instr);
    try std.testing.expectEqual(@as(u12, 0x202), cpu.pc);
}

test "CLS instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x00;
    cpu.memory[0x201] = 0xE0;

    cpu.decode(cpu.fetch());

    // No specific state change to check for CLS, just ensure no crash
    try std.testing.expectEqual(@as(u12, 0x201), cpu.pc);
}

test "RET instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x00;
    cpu.memory[0x201] = 0xEE;
    cpu.stack_push(0x300);

    cpu.decode(cpu.fetch());

    try std.testing.expectEqual(0x300, cpu.pc);
    try std.testing.expectEqual(0, cpu.sp);
}

test "JP instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x12;

    cpu.decode(cpu.fetch());

    try std.testing.expectEqual(@as(u12, 0x12), cpu.pc);
}

test "CALL instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x22;
    cpu.memory[0x201] = 0x22;

    cpu.decode(cpu.fetch());

    try std.testing.expectEqual(@as(u12, 0x222), cpu.pc);
    try std.testing.expectEqual(@as(u8, 0x02), cpu.memory[0]);
    try std.testing.expectEqual(@as(u8, 0x01), cpu.memory[1]);
    try std.testing.expectEqual(2, cpu.sp);
}

test "SE instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x3A;
    cpu.memory[0x201] = 0x00;
    cpu.registers[0xA] = 0x00;

    cpu.decode(cpu.fetch());

    try std.testing.expectEqual(@as(u12, 0x202), cpu.pc);
}

test "SNE instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x4A;
    cpu.memory[0x201] = 0x01;
    cpu.registers[0xA] = 0x00;

    cpu.decode(cpu.fetch());

    try std.testing.expectEqual(@as(u12, 0x202), cpu.pc);
}

test "SE register instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x5A;
    cpu.memory[0x201] = 0xB0;
    cpu.registers[0xA] = 0x01;
    cpu.registers[0xB] = 0x01;

    cpu.decode(cpu.fetch());

    try std.testing.expectEqual(@as(u12, 0x202), cpu.pc);
}

test "LD instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x6A;
    cpu.memory[0x201] = 0x01;

    cpu.decode(cpu.fetch());

    try std.testing.expectEqual(@as(u8, 0x01), cpu.registers[0xA]);
}
