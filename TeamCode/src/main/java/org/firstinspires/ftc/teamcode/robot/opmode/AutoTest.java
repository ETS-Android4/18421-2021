package org.firstinspires.ftc.teamcode.robot.opmode;

import com.amarcolini.joos.command.RobotOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.robot.FedEx;

@Autonomous(name = "New Auto Test")
public class AutoTest extends RobotOpMode {
    private FedEx bot;

    @Override
    public void init() {
        bot = new FedEx(this);
        bot.initAutoBlueWarehouse();
        initialize(bot);
    }
}
