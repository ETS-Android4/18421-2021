package org.firstinspires.ftc.teamcode.robot.opmode;

import com.amarcolini.joos.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.FedEx;

@TeleOp(name = "TeleOpTest")
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
