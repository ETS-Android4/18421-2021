package org.firstinspires.ftc.teamcode.robot.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.drive.TankDrive;
import com.amarcolini.joos.trajectory.Trajectory;
import com.amarcolini.joos.util.DashboardUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TankDrive drive = TuningBot.get(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory).onExecute((cmd) -> {
            TelemetryPacket packet = new TelemetryPacket();
            DashboardUtil.drawRobot(packet.fieldOverlay(), drive.getPoseEstimate());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }).run();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.x);
        telemetry.addData("finalY", poseEstimate.y);
        telemetry.addData("finalHeading", poseEstimate.heading);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
