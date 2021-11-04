package org.firstinspires.ftc.teamcode;

import com.griffinrobotics.lib.command.Command;
import com.griffinrobotics.lib.command.CommandScheduler;
import com.griffinrobotics.lib.command.Component;
import com.griffinrobotics.lib.hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift implements Component {
    private Motor motor;

    public Lift(Motor motor) {
        this.motor = motor;
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.resetEncoder();
    }

    public void setLevel(int level) {
        switch(Math.max(Math.min(level, 3), 1)) {
            case 1: {
                motor.setTargetPosition(0);
                break;
            }
            case 2: {
                motor.setTargetPosition(10);
            }
            case 3: {
                motor.setTargetPosition(20);
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
