package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandScheduler;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift implements Component {
    private Motor motor;

    public Lift(Motor motor) {
        this.motor = motor;
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.resetEncoder();
    }

    public void setLevel(int level) {
        motor.setRunMode(Motor.RunMode.PositionControl);
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
    }

    public void reset() {
        motor.resetEncoder();
    }

    @Override
    public Command getDefaultCommand() {
        return null;
    }

    @Override
    public void update(CommandScheduler commandScheduler) {
        motor.update(commandScheduler);
    }
}
