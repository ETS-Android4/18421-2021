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
    private final Robot robot;
    private int level = 1;

    public Lift(Motor motor, Robot robot) {
        this.motor = motor;
        this.robot = robot;
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.set(0.0);
        motor.setPositionTolerance(1);
        motor.setPositionCoefficients(new PIDCoefficients(0.3, 0.0, 0.1));
        motor.resetEncoder();
        motor.setTargetPosition(0);
    }

    public int getLevel() {
        return level;
    }

    public Command setLevel(int newLevel) {
        int newPosition = 0;
        switch(Math.max(Math.min(newLevel, 3), 1)) {
            case 1: {
                break;
            }
            case 2: {
                newPosition = -600;
                break;
            }
            case 3: {
                newPosition = -1150;
                break;
            }
        }
        int finalNewPosition = newPosition;
        motor.setTargetPosition(finalNewPosition);
        return new FunctionalCommand(
                () -> {},
                () -> {
            motor.setRunMode(Motor.RunMode.PositionControl);
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.set(0.01);
            motor.setTargetPosition(finalNewPosition);
            motor.update();
            robot.telemetry.addData("working", motor.getCurrentPosition());
        },
                (interrupted) -> {
                    robot.telemetry.addData("worked?", Math.abs(motor.getCurrentPosition() - finalNewPosition) <= 2);
                    if (!interrupted) this.level = newLevel;
                    motor.set(0);
                }, () -> false, false,
                Collections.singleton(this)
        );
    }

    public void stop() {
        final CommandScheduler scheduler = getScheduler();
        if (scheduler != null)
        scheduler.cancel(scheduler.requiring(this));
        motor.set(0.0);
    }

    public void reset() {
        motor.resetEncoder();
    }

    @Override
    public void update() {
        motor.update();
        robot.telemetry.addData("position", motor.getCurrentPosition());
        robot.telemetry.addData("target", motor.getTargetPosition());
    }
}
