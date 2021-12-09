package org.firstinspires.ftc.teamcode.robot.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.drive.TankDrive;
import com.amarcolini.joos.util.Angle;
import com.amarcolini.joos.util.DashboardUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

/*
 * This routine determines the effective track width. The procedure works by executing a point turn
 * with a given angle and measuring the difference between that angle and the actual angle (as
 * indicated by an external IMU/gyro, track wheels, or some other localizer). The quotient
 * given angle / actual angle gives a multiplicative adjustment to the estimated track width
 * (effective track width = estimated track width * given angle / actual angle). The routine repeats
 * this procedure a few times and averages the values for additional accuracy. Note: a relatively
 * accurate track width estimate is important or else the angular constraints will be thrown off.
 */
@Config
@Autonomous(group = "drive")
public class TrackWidthTuner extends LinearOpMode {
    public static double ANGLE = 180; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TankDrive drive = new TuningBot(hardwareMap);
        // TODO: if you haven't already, set the localizer to something that doesn't depend on
        //  drive encoders for computing the heading

        telemetry.addLine("Press play to begin the track width tuner routine");
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;

            Command turn = drive.followTrajectory(drive.trajectoryBuilder().turn(Math.toRadians(ANGLE)).build());

            turn.init();
            do {
                double heading = drive.getPoseEstimate().heading;
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;

//                TelemetryPacket packet = new TelemetryPacket();
//                DashboardUtil.drawRobot(packet.fieldOverlay(), drive.getPoseEstimate());
//                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                turn.execute();
            }
            while (!isStopRequested() && !turn.isFinished());
            turn.end(false);

            double trackWidth = TuningBot.TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator;
            trackWidthStats.add(trackWidth);

            sleep(DELAY);
        }

        telemetry.clearAll();
        telemetry.addLine("Tuning complete");
        telemetry.addLine(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(),
                trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
