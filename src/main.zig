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

    // read byte and move pc
    fn fetch(self: *CPU) u8 {
        defer self.pc += 1;
        return self.memory[self.pc];
    }

    fn decode(self: *CPU, instr: u8) void {
        switch (instr) {

            // 00E0 - CLS
            0x00E0 => return,

            // return from subroutine
            0x00EE => {
                // read stack memory
                const byte = self.memory[self.sp];
                defer self.sp -= 1;

                // jump to addr at top of stack
                self.jump(@truncate(byte));
            },

            // 0x0nnn = jump to nnn
            0x0000...0x1FFF => self.jump(@truncate(instr)),

            // call subroutine
            0x2000...0x2FFF => {
                self.sp += 1;
                self.memory[self.sp] = self.pc;
                self.jump(@truncate(instr));
            },

            // 3xkk - SE Vx, byte
            0x3000...0x3FFF => {
                const x = instr & 0x0F00;
                const kk = instr & 0x00FF;

                if (self.registers[x] == kk) {
                    self.pc += 1;
                }
            },

            // 4xkk - SNE Vx, byte
            0x4000...0x4FFF => {
                const x = instr & 0x0F00;
                const kk = instr & 0x00FF;

                if (self.registers[x] != kk) {
                    self.pc += 1;
                }
            },

            // 5xy0 - SE Vx, Vy
            0x5000...0x5FFF => {
                const x = instr & 0x0F00;
                const y = instr & 0x00F0;

                if (self.registers[x] == self.registers[y]) {
                    self.pc += 1;
                }
            },

            // 6xkk - LD Vx, byte
            0x6000...0x6FFF => {
                const x = instr & 0x0F00;
                const kk = instr & 0x00FF;
                self.registers[x] = kk;
            },

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

test "simple test" {
    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // try commenting this out and see if zig detects the memory leak!
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}
