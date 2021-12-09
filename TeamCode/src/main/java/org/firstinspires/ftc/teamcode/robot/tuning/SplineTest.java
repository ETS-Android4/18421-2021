package org.firstinspires.ftc.teamcode.robot.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.hardware.drive.TankDrive;
import com.amarcolini.joos.trajectory.Trajectory;
import com.amarcolini.joos.util.DashboardUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TankDrive drive = new TuningBot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj)
//                .onExecute((cmd) -> {
//                    TelemetryPacket packet = new TelemetryPacket();
//                    DashboardUtil.drawRobot(packet.fieldOverlay(), drive.getPoseEstimate());
//                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
//                })
                .run();

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        )
//                .onExecute((cmd) -> {
//                    TelemetryPacket packet = new TelemetryPacket();
//                    DashboardUtil.drawRobot(packet.fieldOverlay(), drive.getPoseEstimate());
//                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
//                })
                .run();
    }
}
