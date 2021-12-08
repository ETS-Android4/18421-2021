package org.firstinspires.ftc.teamcode.robot.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.amarcolini.joos.hardware.drive.TankDrive;
import com.amarcolini.joos.util.DashboardUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        TankDrive drive = TuningBot.get(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(drive.trajectoryBuilder().turn(Math.toRadians(ANGLE)).build()).onExecute((cmd) -> {
            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(packet.fieldOverlay(), drive.getPoseEstimate());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }).run();
    }
}
