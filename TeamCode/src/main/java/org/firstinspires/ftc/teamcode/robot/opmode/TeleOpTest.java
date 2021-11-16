package org.firstinspires.ftc.teamcode.robot.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.FedEx;

public class TeleOpTest extends OpMode {
    private FedEx bot;

    @Override
    public void init() {
        bot = new FedEx(this);
        bot.initTeleOp();
    }

    @Override
    public void loop() {
        bot.update();
    }
}
