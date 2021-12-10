package org.firstinspires.ftc.teamcode.robot.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.robot.FedEx;

@Autonomous(name = "AutoBlueWarehouse Test")
public class AutoTest extends OpMode {
    private FedEx bot;

    @Override
    public void init() {
        bot = new FedEx(this);
        bot.initAutoBlueWarehouse();
    }

    @Override
    public void loop() {
        bot.update();
    }
}
