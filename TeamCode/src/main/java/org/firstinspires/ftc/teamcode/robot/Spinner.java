package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.hardware.Motor;

public class Spinner implements Component {
    private final Motor motor;
    private boolean isActive = false;

    public Spinner(Motor motor) {
        this.motor = motor;
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        motor.set(1.0);
        isActive = true;
    }

    public void stop() {
        motor.set(0.0);
        isActive = false;
    }

    public boolean isActive() {
        return isActive;
    }

    public void toggle() {
        if (isActive) stop();
        else start();
    }

    @Override
    public void update() {
        motor.update();
    }
}
