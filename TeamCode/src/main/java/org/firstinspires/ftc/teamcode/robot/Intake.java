package org.firstinspires.ftc.teamcode.robot;

import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.hardware.Motor;

public class Intake implements Component {
    private final Motor motor;
    private Boolean isActive = false;

    public Intake(Motor motor) {
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        motor.setReversed(true);
        this.motor = motor;
    }

    public void start() {
        motor.setPower(1.0);
        isActive = true;
    }

    public void stop() {
        motor.setPower(0.0);
        isActive = false;
    }

    public void toggle() {
        if (isActive) stop();
        else start();
    }

    public void reverse() {
        motor.setPower(-1.0);
    }

    public Boolean isActive() {
        return isActive;
    }

    @Override
    public void update() {
        motor.update();
    }
}
