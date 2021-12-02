package org.firstinspires.ftc.teamcode.robot;

import static java.util.Set.*;

import androidx.annotation.NonNull;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.Servo;
import com.amarcolini.joos.hardware.drive.TankDrive;
import com.amarcolini.joos.kinematics.TankKinematics;
import com.amarcolini.joos.trajectory.config.TankConstraints;
import com.amarcolini.joos.util.MathUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class FedEx extends Robot {
    public final Lift lift;
    public final Spinner spinner;
    public final Intake intake;
    public final Conveyor conveyor;
    public final Bucket bucket;
    public final TankDrive drive;
    public final Camera camera;

    public FedEx(@NonNull OpMode opMode) {
        super(opMode);

        lift = new Lift(new Motor(hMap, "lift", 435, 384.5), this);
        bucket = new Bucket(new Servo(hMap, "bucket"));
        Motor left = new Motor(hMap, 312.0, "front_left", "back_left");
        left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        Motor right = new Motor(hMap, 312.0, "front_right", "back_right");
        right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drive = new TankDrive(
                left, right, null,
                new TankConstraints(
                        left.getMaxRPM(),
                        18.0,
                        30.0,
                        30.0,
                        Math.toRadians(90.0),
                        Math.toRadians(100.0)
                )
        );
        spinner = new Spinner(new Motor(hMap, "spinner", 1620));
        intake = new Intake(new Motor(hMap, "intake", 1620));
        conveyor = new Conveyor(new Motor(hMap, "conveyor", 1620));
        camera = new Camera("webcam", this);

        register(lift, bucket, drive, spinner, intake, conveyor, gamepad);
    }

    public void initTeleOp() {
        schedule(Command.of(() -> {
            Vector2d stick = new Vector2d(
                    gamepad.p1.getInternal().left_stick_x,
                    gamepad.p1.getInternal().left_stick_y
            );
            telemetry.addData("stick", stick);
            Pose2d vel = new Pose2d(
                    stick.getX(), 0,
                    -stick.getY()
            ).times(2);
            telemetry.addData("power", TankKinematics.robotToWheelVelocities(vel, 1));
            drive.setDrivePower(vel);
        }).requires(drive).runUntil(false));

        map(gamepad.p1.a::justActivated, Command.select(() -> {
            int newLevel = lift.getLevel() + 1;
            if (newLevel > 3) newLevel = 1;
            return lift.setLevel(newLevel);
        }).requires(lift));
        map(() -> gamepad.p1.b.justActivated() && requiring(lift) == null, bucket::toggle);
        map(gamepad.p1.x::justActivated, () -> {
            intake.toggle();
            conveyor.toggle();
        });
        map(gamepad.p1.y::justActivated, spinner::toggle);
    }

    public void initAuto() {
        camera.start();

        schedule(Command.of(() -> {
            camera.close();
            schedule(Command.select(() -> {
                final Command move = drive.followTrajectory(
                        drive.trajectoryBuilder(
                                drive.getPoseEstimate(),
                                0.0
                        )
                                .splineTo(new Vector2d(-14.26, 43.73), Math.toRadians(90.0))
                                .build()
                );
                final Vector2d warehouse = new Vector2d(40.84, 41.03);
                switch(camera.getLastPosition()) {
                    case TWO:
                        return move.then(lift.setLevel(2)).then(bucket::open).then(Command.select(() -> drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                                .splineTo(warehouse, 0.0)
                                .build()
                        )));
                    case THREE:
                        return move.then(lift.setLevel(3)).then(bucket::open).then(Command.select(() -> drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                                        .splineTo(warehouse, 0.0)
                                        .build()
                        )));
                    default:
                        return move.then(lift.setLevel(1)).then(bucket::open).then(Command.select(() -> drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                                        .splineTo(warehouse, 0.0)
                                        .build()
                        )));
                }
            }));
        }));
    }
}
