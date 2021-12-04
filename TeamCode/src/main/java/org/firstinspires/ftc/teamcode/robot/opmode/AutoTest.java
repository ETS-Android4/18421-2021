package org.firstinspires.ftc.teamcode.robot.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.FedEx;

@Autonomous(name = "AutoBlueWarehouse Test")
public class AutoTest extends LinearOpMode {
    private FedEx bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new FedEx(this);
        bot.initAutoBlueWarehouse();

        waitForStart();

        for (int i = 0; i < 10; i++) {
            bot.update();
        }
    }
}
