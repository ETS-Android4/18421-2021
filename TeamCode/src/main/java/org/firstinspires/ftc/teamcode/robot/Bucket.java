package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.hardware.Servo;

public class Bucket implements Component {
    private final Servo servo;
    private boolean isOpen = false;

    public Bucket(Servo servo) {
        this.servo = servo;
        servo.setPosition(0.1);
    }

    public void open() {
        servo.setPosition(0.5);
        isOpen = true;
    }

    public void close() {
        servo.setPosition(0.1);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) close();
        else open();
    }

    public boolean isOpen() {
        return isOpen;
    }

    @Override
    public void update() {
        servo.update();
    }
}
