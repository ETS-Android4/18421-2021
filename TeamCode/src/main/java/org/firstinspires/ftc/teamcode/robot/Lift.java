package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.amarcolini.joos.command.AbstractComponent;
import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandScheduler;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.command.FunctionalCommand;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Collections;
import java.util.Set;

public class Lift extends AbstractComponent {
    private final Motor motor;
    private int level = 1;

    public Lift(Motor motor) {
        this.motor = motor;
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.set(0.0);
        motor.setPositionTolerance(1);
        motor.setPositionCoefficients(new PIDCoefficients(0.3, 0.0, 0.1));
        motor.resetEncoder();
    }

    public int getLevel() {
        return level;
    }

    public Command setLevel(int level) {
        return Command.of(() -> {
            motor.setRunMode(Motor.RunMode.PositionControl);
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.set(0.01);
            switch(Math.max(Math.min(level, 3), 1)) {
                case 1: {
                    motor.setTargetPosition(0);
                    break;
                }
                case 2: {
                    motor.setTargetPosition(10);
                }
                case 3: {
                    motor.setTargetPosition(-1115);
                }
            }
        })
                .requires(Collections.singleton(this))
                .runUntil(() -> !motor.isBusy())
                .onEnd((command, interrupted) -> {
                    if (!interrupted) this.level = level;
                    motor.set(0);
                });
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
    }
}
