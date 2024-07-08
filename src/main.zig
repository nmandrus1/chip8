const std = @import("std");
const SDL = @import("sdl2");

const fs = std.fs;
const posix = std.posix;
const time = std.time;

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

    display: [256]u8 = [_]u8{0} ** 256,

    renderer: SDL.Renderer,
    window: SDL.Window,

    fn init() !CPU {
        // Window setup
        try SDL.init(.{
            .video = true,
            .events = true,
        });

        const window = try SDL.createWindow(
            "SDL2 Wrapper Demo",
            .{ .centered = {} },
            .{ .centered = {} },
            640,
            320,
            .{ .vis = .shown },
        );

        const renderer = try SDL.createRenderer(window, null, .{ .accelerated = true });

        const sprite_slice = [_]u8{
            0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
            0x20, 0x60, 0x20, 0x20, 0x70, // 1
            0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
            0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
            0x90, 0x90, 0xF0, 0x10, 0x10, // 4
            0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
            0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
            0xF0, 0x10, 0x20, 0x40, 0x40, // 7
            0xF0, 0x90, 0xF0, 0x90, 0xF0, // 9
            0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
            0xF0, 0x90, 0xF0, 0x90, 0x90, // A
            0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
            0xF0, 0x80, 0x80, 0x80, 0xF0, // C
            0xE0, 0x90, 0x90, 0x90, 0xE0, // D
            0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
            0xF0, 0x80, 0xF0, 0x80, 0x80, // F
        };

        const cpu = CPU{
            .pc = 0x200,
            .sp = 0,
            // fill memory with sprite data and the rest with 0s
            .memory = sprite_slice ++ [_]u8{0} ** (4096 - sprite_slice.len),
            .registers = [_]u8{0} ** 16,
            .i = 0,
            .window = window,
            .renderer = renderer,
        };

        return cpu;
    }

    fn loadROM(self: *CPU, rom: std.fs.File) !void {
        _ = try rom.readAll(self.memory[0x200..]);
    }

    fn deinit(self: CPU) void {
        self.renderer.destroy();
        self.window.destroy();
        SDL.quit();
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
        std.log.debug("PC: 0x{X:0>3}", .{self.pc});
        defer self.pc += 2;
        const msb = @as(u16, self.memory[self.pc]) << 8;
        const lsb = @as(u16, self.memory[self.pc + 1]);
        return msb | lsb;
    }

    fn decode_and_execute(self: *CPU, instr: u16) void {
        const nibble = instr & 0xF000;
        const x: u8 = @truncate((instr & 0x0F00) >> 8);
        const y: u8 = @truncate((instr & 0x00F0) >> 4);
        const kk: u8 = @truncate(instr & 0x00FF);
        const nnn: u12 = @truncate(instr & 0x0FFF);
        switch (nibble) {
            0x0000 => {
                switch (instr) {
                    // CLS
                    0x00E0 => {
                        for (&self.display) |*byte| byte.* = 0;
                    },
                    // RET
                    0x00EE => self.jump(@truncate(self.stack_pop())),
                    else => self.jump(@truncate(instr)),
                }
            },

            // jp nnn
            0x1000 => self.jump(@truncate(instr)),

            // 0x2nn call subroutine
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
            0x6000 => self.registers[x] = kk,

            // 7xkk - ADD Vx, byte
            0x7000 => self.registers[x] += kk,

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
                        // TODO: Maybe just truncate??
                        self.registers[0xF] = if (self.registers[x] & 0x01 == 1) 1 else 0;
                        self.registers[x] >>= 1;
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
                        self.registers[0xF] = if (self.registers[x] >> 7 == 1) 1 else 0;
                        self.registers[x] <<= 1;
                    },
                    else => unreachable,
                }
            },

            // Annn - LD I, addr
            0xA000 => self.i = nnn,

            // Bnnn - JP V0, addr
            // TODO: fix? is there a better way to do this?
            0xB000 => self.pc = nnn + self.registers[0],

            // Cxkk - RND Vx, byte
            0xC000 => {
                const random_byte = std.crypto.random.int(u8);
                self.registers[x] = random_byte & kk;
            },

            // Dxyn - DRW Vx, Vy, nibble
            // Display n-byte sprite starting at memory location I at (Vx, Vy), set VF = collision.
            0xD000 => {
                const n = instr & 0x000F;
                self.drawSprite(self.memory[self.i .. self.i + n], self.registers[x], self.registers[y]);
            },

            // Ex9E - SKP Vx and ExA1 - SKNP Vx
            0xE000 => switch (kk) {
                0x9E | 0xA1 => {
                    if (pollKeyboard()) |key| {
                        // Skip next instruction if key with the value of Vx is pressed.
                        if ((key == self.registers[x] and kk == 0x9E) or
                            // Skip next instruction if key with the value of Vx is not pressed.
                            (key != self.registers[x] and kk == 0xA1)) self.pc += 2;
                    }
                },
                else => unreachable,
            },

            0xF000 => switch (kk) {

                // Fx07 - LD Vx, DT
                // Set Vx = delay timer value.
                0x07 => std.debug.print("LD Vx, DT -- TODO!", .{}),

                // Fx0A - LD Vx, K
                // Wait for a key press, store the value of the key in Vx.
                0x0A => self.registers[x] = waitForKey(),

                // Fx15 - LD DT, Vx
                // Set delay timer = Vx.
                0x15 => std.debug.print("LD DT, Vx -- TODO!", .{}),

                // Fx18 - LD ST, Vx
                // Set sound timer = Vx.
                0x18 => std.debug.print("LD ST, Vx -- TODO!", .{}),

                // Fx1E - ADD I, Vx
                // Set I = I + Vx.
                0x1E => self.i += self.registers[x],

                // Fx29 - LD F, Vx
                // Set I = location of sprite for digit Vx.
                0x29 => std.debug.print("LD F, Vx -- TODO!", .{}),

                // Fx33 - LD B, Vx
                // Store BCD representation of Vx in memory locations I, I+1, and I+2.
                0x33 => {
                    var buf: [3]u8 = undefined;
                    _ = std.fmt.bufPrintIntToSlice(buf[0..], self.registers[x], 10, .lower, std.fmt.FormatOptions{});
                    for (buf, 0..) |digit, idx| self.memory[self.i + idx] = digit;
                },

                // Fx55 - LD [I], Vx
                // Store registers V0 through Vx in memory starting at location I.
                0x55 => {
                    for (0..x) |idx| self.memory[self.i + idx] = self.registers[idx];
                },

                // Fx65 - LD Vx, [I]
                // Read registers V0 through Vx from memory starting at location I.
                0x65 => {
                    for (0..x) |idx| self.registers[idx] = self.memory[self.i + idx];
                },
                else => unreachable,
            },

            else => unreachable,
        }
    }

    // set pc to given address
    fn jump(self: *CPU, addr: u12) void {
        self.pc = addr;
    }

    fn drawScreen(self: *CPU) !void {
        // clear and redraw the current screen
        try self.renderer.clear();

        for (self.display, 0..) |byte, idx| {
            const y: usize = @divTrunc(idx, 8);
            const x: usize = idx - (y * 8);

            for (0..8) |xx| {
                var pxl = Pixel;
                pxl.x = @as(c_int, @intCast(x + xx)) * pxl.width;
                pxl.y = @as(c_int, @intCast(y)) * pxl.height;

                const clr = if ((byte >> @intCast(xx)) & 0x1 == 1) SDL.Color.white else SDL.Color.black;

                try self.renderer.setColor(clr);
                try self.renderer.fillRect(pxl);
                try self.renderer.drawRect(pxl);
            }
        }

        self.renderer.present();
    }

    fn drawSprite(self: *CPU, bytes: []u8, x: u8, y: u8) void {
        std.debug.assert(bytes.len < 16);

        // screen is 32 rows of 8 bytes, so we need to identify the bytes we are editing
        const x_coord = @divTrunc(x, 8);
        const starting_bit: u3 = @truncate(@mod(x, 8));
        const split_byte = starting_bit != 0;

        for (bytes, y..) |sprite, yy| {
            const idx1 = yy * 8 + x_coord;
            if (split_byte) {
                // handle wrap around
                const idx2 = yy * 8 + @divTrunc(@mod(x + 8, 64), 8);
                const left_byte: u8 = sprite >> starting_bit;
                const right_byte: u8 = (sprite << 1) << (7 - starting_bit - 1);

                self.display[idx1] ^= left_byte;
                self.display[idx2] ^= right_byte;
            } else self.display[idx1] ^= sprite;
        }
    }

    pub fn run(self: *CPU) !void {
        // 60 fps
        const frame_time_ms = 16;
        try self.drawScreen();

        // run as many instructions as we can per frame
        mainLoop: while (true) {
            const start = try time.Instant.now();
            exeLoop: while (true) {
                try self.step();

                const end = try time.Instant.now();
                std.debug.print("since start: {d}\n", .{end.since(start)});
                if (end.since(start) >= frame_time_ms * time.ns_per_ms) break :exeLoop else {
                    if (SDL.pollEvent()) |ev| switch (ev) {
                        .quit => break :mainLoop,
                        else => {},
                    };
                }
            }

            std.debug.print("here!!\n", .{});
            try self.drawScreen();
        }
    }

    pub fn step(self: *CPU) !void {
        const instr = self.fetch();
        std.log.debug("Opcode: 0x{X:0>4}", .{instr});
        self.decode_and_execute(instr);
    }
};

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    const allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    _ = args.skip();
    const rom_file = args.next().?;

    const rom = try std.fs.cwd().openFile(rom_file, .{ .mode = .read_only });

    var cpu = try CPU.init();
    defer cpu.deinit();

    try cpu.loadROM(rom);
    try cpu.run();
}

const Pixel = SDL.Rectangle{ .width = 10, .height = 10 };

fn pollKeyboard() ?u8 {
    return if (SDL.pollEvent()) |ev| switch (ev) {
        .key_down => |kev| switch (kev.keycode) {
            .@"0" => return 0x0,
            .@"1" => return 0x1,
            .@"2" => return 0x2,
            .@"3" => return 0x3,
            .@"4" => return 0x4,
            .@"5" => return 0x5,
            .@"6" => return 0x6,
            .@"7" => return 0x7,
            .@"8" => return 0x8,
            .@"9" => return 0x9,
            .a => return 0xA,
            .b => return 0xB,
            .c => return 0xC,
            .d => return 0xD,
            .e => return 0xE,
            .f => return 0xF,
            else => return null,
        },
        else => null,
    } else null;
}

fn waitForKey() u8 {
    while (true) {
        if (pollKeyboard()) |code| return code;
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
