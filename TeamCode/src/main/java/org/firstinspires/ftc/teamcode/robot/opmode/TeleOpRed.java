package org.firstinspires.ftc.teamcode.robot.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.FedEx;

@TeleOp(name = "TeleOpRed")
public class TeleOpRed extends OpMode {
    private FedEx bot;

    @Override
    public void init() {
        bot = new FedEx(this);
        bot.initTeleOp();
        bot.spinner.reversed = true;
    }

    @Override
    public void loop() {
        bot.update();
    }
}
