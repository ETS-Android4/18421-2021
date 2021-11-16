package org.firstinspires.ftc.teamcode.robot;

import static java.util.Set.*;

import androidx.annotation.NonNull;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.Servo;
import com.amarcolini.joos.hardware.drive.TankDrive;
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

    public FedEx(@NonNull OpMode opMode) {
        super(opMode);

        lift = new Lift(new Motor(hMap, "lift", 312, 537.7));
        bucket = new Bucket(new Servo(hMap, "bucket"));
        drive = new TankDrive(
                new Motor(hMap, 312.0, "front_left", "back_left"),
                new Motor(hMap, 312.0, "front_right", "back_right")
        );
        spinner = new Spinner(new Motor(hMap, "spinner", 1620));
        intake = new Intake(new Motor(hMap, "intake", 1620));
        conveyor = new Conveyor(new Motor(hMap, "conveyor", 1620));

        register(lift, bucket, drive, spinner);
    }

    public void initTeleOp() {
        schedule(Command.of(() -> drive.setDrivePower(new Pose2d(
                gamepad.p1.getLeftStick().getX(), 0,
                gamepad.p1.getLeftStick().getY()
        ))).requires(drive).runUntil(false));

        map(gamepad.p1.a::justActivated, Command.select(() -> {
            int newLevel = (int) MathUtil.wrap(lift.getLevel() + 1, 1, 3);
            return lift.setLevel(newLevel);
        }).requires(lift));
        map(() -> gamepad.p1.b.justActivated() && requiring(lift) == null, bucket::toggle);
        map(gamepad.p1.x::justActivated, () -> {
            intake.toggle();
            conveyor.toggle();
        });
        map(gamepad.p1.y::justActivated, spinner::toggle);
    }
}
