package org.firstinspires.ftc.teamcode.robot.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.drive.TankDrive;
import com.amarcolini.joos.util.DashboardUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum velocity your bot can achieve under load. It
 * will also calculate the effective kF value for your velocity PID.
 * <p>
 * Upon pressing start, your bot will run at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of kF may be desired.
 */
@Config
@Autonomous(group = "drive")
public class MaxVelocityTuner extends LinearOpMode {
    public static double RUNTIME = 2.0;

    private ElapsedTime timer;
    private double maxVelocity = 0.0;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        TankDrive drive = new TuningBot(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Your bot will go at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setWeightedDrivePower(new Pose2d(1, 0, 0));
        timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

//            TelemetryPacket packet = new TelemetryPacket();
//            DashboardUtil.drawRobot(packet.fieldOverlay(), drive.getPoseEstimate());
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxVelocity = Math.max(poseVelo.vec().norm(), maxVelocity);
        }

        drive.setDrivePower(new Pose2d());

//        double effectiveKf = DriveConstants.getMotorVelocityF(veloInchesToTicks(maxVelocity));

        telemetry.addData("Max Velocity", maxVelocity);
//        telemetry.addData("Voltage Compensated kF", effectiveKf * batteryVoltageSensor.getVoltage() / 12);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) idle();
    }

//    private double veloInchesToTicks(double inchesPerSec) {
//        return inchesPerSec / (2 * Math.PI * TuningBot.WHEEL_RADIUS) / TuningBot.GEAR_RATIO * TuningBot.TICKS_PER_REV;
//    }
}
