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
    i: u12,

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

    fn decode_and_execute(self: *CPU, instr: u16) void {
        const nibble = instr & 0xF000;
        const x = (instr & 0x0F00) >> 8;
        const y = (instr & 0x00F0) >> 4;
        const kk = instr & 0x00FF;
        const nnn = instr & 0x0FFF;
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
            // 4xkk - SNE Vx, byte
            0x3000, 0x4000 => {
                // conditions to skip instructions for both SE and SNE
                if ((self.registers[x] == kk and nibble == 0x3000) or
                    (self.registers[x] != kk and nibble == 0x4000))
                    self.pc += 2;
            },

            // 5xy0 - SE Vx, Vy
            // 9xy0 - SNE Vx, Vy
            0x5000, 0x9000 => {
                // conditions to skip next instr if either are true
                if ((self.registers[x] == self.registers[y] and nibble == 0x5000) or
                    (self.registers[x] != self.registers[y] and nibble == 0x9000))
                    self.pc += 2;
            },

            // 6xkk - LD Vx, byte
            0x6000 => self.registers[x] = @truncate(kk),

            // 7xkk - ADD Vx, byte
            0x7000 => self.registers[x] += @truncate(kk),

            0x8000 => {
                const lsb = instr & 0x000F;
                switch (lsb) {
                    // 0x8xy0 - LD Vx, Vy
                    0x0 => self.registers[x] = self.registers[y],
                    // 0x8xy1 - OR Vx, Vy
                    0x1 => self.registers[x] |= self.registers[y],
                    // 0x8xy2 - AND Vx, Vy
                    0x2 => self.registers[x] &= self.registers[y],
                    // 0x8xy3 - XOR Vx, Vy
                    0x3 => self.registers[x] ^= self.registers[y],
                    // 0x8xy4 - ADD Vx, Vy setting carry flag if neccessary
                    0x4 => {},
                    // 0x8xy5 - SUB Vx, Vy setting carry flag to NOT overflow
                    0x5 => {
                        // Vx = Vx - Vy
                        // If Vx > Vy, then VF is set to 1, otherwise 0. Then Vy is subtracted from Vx, and the results stored in Vx.
                        const result = @subWithOverflow(self.registers[x], self.registers[y]);
                        self.registers[x] = result.@"0";
                        self.registers[0xF] = ~result.@"1";
                    },

                    // 0x8xy6 - SHR Vx
                    0x6 => {
                        // Vx = Vx >> 1 and Vf is set to 1 if lsb(Vx) = 1
                        self.registers[0xF] = if (self.registers[x] & 0x01) 1 else 0;
                        self.registers[x] >> 1;
                    },
                    // 8xy7 - SUBN Vx, Vy
                    0x7 => {
                        // Set Vx = Vy - Vx, set VF = NOT borrow.
                        const result = @subWithOverflow(self.registers[y], self.registers[x]);
                        self.registers[y] = result.@"0";
                        self.registers[0xF] = ~result.@"1";
                    },
                    // 8xyE - SHL Vx {, Vy}
                    0xE => {
                        // Set Vx = Vx SHL 1.
                        self.registers[0xF] = if (self.registers[x] >> 7) 1 else 0;
                        self.registers[x] << 1;
                    },
                    else => unreachable,
                }
            },

            // Annn - LD I, addr
            0xA000 => self.i = nnn,

            // Bnnn - JP V0, addr
            0xB000 => self.pc = self.registers[0] + nnn,

            // Cxkk - RND Vx, byte
            0xC000 => {
                const random_byte = std.crypto.random.int(u8);
                self.registers[x] = random_byte & kk;
            },

            // Dxyn - Draw to screen

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

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();

    const stdin = std.io.getStdIn();
    // const poller = std.io.Poller(std.fs.File);
    var poller = std.io.poll(allocator, enum { stdin }, .{ .stdin = stdin });
    defer poller.deinit();

    while (try poller.poll()) {
        std.debug.print("{any}", .{poller.fifo(.stdin).readItem().?});
    }
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

    cpu.decode_and_execute(cpu.fetch());

    // No specific state change to check for CLS, just ensure no crash
    try std.testing.expectEqual(@as(u12, 0x201), cpu.pc);
}

test "RET instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x00;
    cpu.memory[0x201] = 0xEE;
    cpu.stack_push(0x300);

    cpu.decode_and_execute(cpu.fetch());

    try std.testing.expectEqual(0x300, cpu.pc);
    try std.testing.expectEqual(0, cpu.sp);
}

test "JP instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x12;

    cpu.decode_and_execute(cpu.fetch());

    try std.testing.expectEqual(@as(u12, 0x12), cpu.pc);
}

test "CALL instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x22;
    cpu.memory[0x201] = 0x22;

    cpu.decode_and_execute(cpu.fetch());

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

    const instr = cpu.fetch();
    try std.testing.expectEqual(instr, 0x3A00);
    cpu.decode_and_execute(instr);

    try std.testing.expectEqual(@as(u12, 0x204), cpu.pc);
}

test "SNE instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x4A;
    cpu.memory[0x201] = 0x01;
    cpu.registers[0xA] = 0x00;

    cpu.decode_and_execute(cpu.fetch());

    try std.testing.expectEqual(@as(u12, 0x202), cpu.pc);
}

test "SE register instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x5A;
    cpu.memory[0x201] = 0xB0;
    cpu.registers[0xA] = 0x01;
    cpu.registers[0xB] = 0x01;

    cpu.decode_and_execute(cpu.fetch());

    try std.testing.expectEqual(@as(u12, 0x202), cpu.pc);
}

test "LD instruction" {
    var cpu = CPU.init();
    cpu.memory[0x200] = 0x6A;
    cpu.memory[0x201] = 0x01;

    cpu.decode_and_execute(cpu.fetch());

    try std.testing.expectEqual(@as(u8, 0x01), cpu.registers[0xA]);
}

test "sub with overflow" {
    const a: u8 = 1;
    const b: u8 = 3;
    const result = @subWithOverflow(a, b);
    try std.testing.expectEqual(1, result.@"0");
    try std.testing.expectEqual(253, result.@"0");
}
