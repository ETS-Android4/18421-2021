package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.amarcolini.joos.command.AbstractComponent;
import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandScheduler;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.command.FunctionalCommand;
import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Collections;
import java.util.Set;

public class Lift extends AbstractComponent {
    private final Motor motor;
//    private final Robot robot;
    private final int min = -880;
    private final int max = 0;
    private int level = 1;

    public Lift(Motor motor) {
        this.motor = motor;
//        this.robot = robot;
        motor.setRunMode(Motor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);
        motor.setPositionTolerance(50);
        motor.setPositionCoefficients(new PIDCoefficients(0.5, 0, 0));
        motor.resetEncoder();
        motor.setTargetPosition(0);
    }

    public int getLevel() {
        return level;
    }

    public Command setLevel(int newLevel) {
        int newPosition = max;
        switch(Math.max(Math.min(newLevel, 3), 1)) {
            case 1: {
                break;
            }
            case 2: {
                newPosition = min;
                break;
            }
            case 3: {
                newPosition = -400;
                break;
            }
        }
        int finalNewPosition = newPosition;
        return Command.of(() -> {
            motor.setTargetPosition(finalNewPosition);
            motor.setRunMode(Motor.RunMode.RUN_TO_POSITION);
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.01);
            motor.update();
        })
                .runUntil(() -> {
                    int actual = Math.max(Math.min(motor.getTargetPosition(), max), min);
                    return Math.abs(actual - motor.getCurrentPosition()) <= 50;
                })
                .requires(this)
                .isInterruptable(true)
                .onEnd((command, interrupted) -> {
                    if (!interrupted) this.level = newLevel;
                    motor.setPower(0);
                });
    }

    public void stop() {
        final CommandScheduler scheduler = getScheduler();
        if (scheduler != null)
        scheduler.cancel(scheduler.requiring(this));
        motor.setPower(0.0);
    }

    public void reset() {
        motor.resetEncoder();
    }

    @Override
    public void update() {
        motor.update();
//        robot.telemetry.addData("position", motor.getCurrentPosition());
//        robot.telemetry.addData("target", motor.getTargetPosition());
    }
}
