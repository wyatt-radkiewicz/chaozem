test "add stuff" {

    // 2) Test that the <instruction> is encoded correctly
    runner = Test.init(&.{ 0xD1B8, 0x0080 });
    Test.wr(&runner.interface, 0x80, u32, 350000);
    runner.cpu.d[0] = 10;
    try std.testing.expectEqual(24, try runner.run("add.l d0,($0080).w"));
    try std.testing.expectEqual(350010, Test.rd(&runner.interface, 0x80, u32));

    // 3) Test that the <instruction> produces correct flag side-effects
    runner = Test.init(&.{0xD001});
    runner.cpu.d[0] = 0x70;
    runner.cpu.d[1] = 0x40;
    try std.testing.expectEqual(4, try runner.run("add.b d1,d0"));
    try std.testing.expectEqual(0xB0, runner.cpu.d[0]);
    try std.testing.expectEqual(false, runner.cpu.sr.c);
    try std.testing.expectEqual(true, runner.cpu.sr.v);
    try std.testing.expectEqual(false, runner.cpu.sr.z);
    try std.testing.expectEqual(true, runner.cpu.sr.n);
    try std.testing.expectEqual(false, runner.cpu.sr.x);

    // 4) Test that the <instruction> produces correct flag side-effects
    runner = Test.init(&.{0xD001});
    runner.cpu.d[0] = 0x10;
    runner.cpu.d[1] = 0xF0;
    try std.testing.expectEqual(4, try runner.run("add.b d1,d0"));
    try std.testing.expectEqual(0x00, runner.cpu.d[0]);
    try std.testing.expectEqual(true, runner.cpu.sr.c);
    try std.testing.expectEqual(false, runner.cpu.sr.v);
    try std.testing.expectEqual(true, runner.cpu.sr.z);
    try std.testing.expectEqual(false, runner.cpu.sr.n);
    try std.testing.expectEqual(true, runner.cpu.sr.x);
}

test "addi.w #imm,dn; addi.l #imm,dn; addi.b #imm,(d16,an)" {
    var runner: Test = undefined;

    // 1) Test that the <instruction> is encoded correctly, and produces correct side effects
    runner = Test.init(&.{ 0x0640, 50 });
    runner.cpu.d[0] = 50;
    try std.testing.expectEqual(8, try runner.run("addi.w #$0032,d0"));
    try std.testing.expectEqual(100, runner.cpu.d[0]);

    // 2) Test that the <instruction> is encoded correctly, and produces correct side effects
    runner = Test.init(&.{ 0x0680, 0, 50 });
    runner.cpu.d[0] = 1000000;
    try std.testing.expectEqual(16, try runner.run("addi.l #$00000032,d0"));
    try std.testing.expectEqual(1000050, runner.cpu.d[0]);

    // 3) Test that the <instruction> evaluates operands correctly
    runner = Test.init(&.{ 0x0628, 50, 0x80 });
    try std.testing.expectEqual(20, try runner.run("addi.b #$32,(128,a0)"));
    try std.testing.expectEqual(50, Test.rd(&runner.interface, 0x80, u8));
}

test "addq #imm,dn; addq #imm,an" {
    var runner: Test = undefined;

    // 1) Test that the <instruction> is encoded correctly, and produces correct side effects
    runner = Test.init(&.{ 0x5e00 });
    runner.cpu.d[0] = 249;
    try std.testing.expectEqual(4, try runner.run("addq.b #7,d0"));
    try std.testing.expectEqual(0, runner.cpu.d[0]);
    try std.testing.expectEqual(true, runner.cpu.sr.c);
    try std.testing.expectEqual(true, runner.cpu.sr.z);

    // 2) Test that the <instruction> is encoded correctly, and produces correct side effects
    runner = Test.init(&.{ 0x5e48 });
    runner.cpu.a[0] = 0xffffffff;
    try std.testing.expectEqual(8, try runner.run("addq.w #7,a0"));
    try std.testing.expectEqual(6, runner.cpu.a[0]);
    try std.testing.expectEqual(false, runner.cpu.sr.c);
}

test "ori #imm,ccr; ori #imm,sr; ori #imm,ea" {
    var runner: Test = undefined;

    // 1) Test that the <instruction> is encoded correctly, and has correct side effects
    runner = Test.init(&.{ 0x003c, 0x007f });
    try std.testing.expectEqual(20, try runner.run("ori.b #$7f,ccr"));
    try std.testing.expectEqual(true, runner.cpu.sr.c);
    try std.testing.expectEqual(true, runner.cpu.sr.v);
    try std.testing.expectEqual(true, runner.cpu.sr.z);
    try std.testing.expectEqual(true, runner.cpu.sr.n);
    try std.testing.expectEqual(true, runner.cpu.sr.x);

    // 2) Test that the <instruction> is encoded correctly, and has correct side effects
    runner = Test.init(&.{ 0x007c, 0xff0f });
    try std.testing.expectEqual(20, try runner.run("ori.w #$ff0f,sr"));
    try std.testing.expectEqual(true, runner.cpu.sr.c);
    try std.testing.expectEqual(true, runner.cpu.sr.v);
    try std.testing.expectEqual(true, runner.cpu.sr.z);
    try std.testing.expectEqual(true, runner.cpu.sr.n);
    try std.testing.expectEqual(false, runner.cpu.sr.x);
    try std.testing.expectEqual(7, runner.cpu.sr.ipl);

    // 3) Test that the <instruction> is encoded correctly, and has correct side effects
    runner = Test.init(&.{ 0x0000, 0x00f0 });
    try std.testing.expectEqual(8, try runner.run("ori.b #$f0,d0"));
    try std.testing.expectEqual(0xf0, runner.cpu.d[0]);
    try std.testing.expectEqual(false, runner.cpu.sr.z);
    try std.testing.expectEqual(true, runner.cpu.sr.n);
}

test "nop" {
    var runner: Test = undefined;

    // 1) Test that the <instruction> is encoded correctly
    runner = Test.init(&.{0x4e71});
    try std.testing.expectEqual(4, try runner.run("nop"));
}
