package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.hardware.Motor;

public class Conveyor implements Component {
    private final Motor motor;
    private Boolean isActive = false;

    public Conveyor(Motor motor) {
        this.motor = motor;
    }

    public void start() {
        motor.set(1.0);
        isActive = true;
    }

    public void stop() {
        motor.set(0.0);
        isActive = false;
    }

    public void toggle() {
        if (isActive) stop();
        else start();
    }

    public Boolean isActive() {
        return isActive;
    }

    @Override
    public void update() {
        motor.update();
    }
}
