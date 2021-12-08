package org.firstinspires.ftc.teamcode.robot.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.amarcolini.joos.control.FeedforwardCoefficients;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.MotorGroup;
import com.amarcolini.joos.hardware.drive.TankDrive;
import com.amarcolini.joos.trajectory.config.TankConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class TuningBot {
    public static double TRACK_WIDTH = 18.0;
    public static double MAX_VEL = 30.0;
    public static double MAX_ACCEL = 30.0;
    public static double MAX_ANG_VEL = Math.toRadians(180.0);
    public static double MAX_ANG_ACCEL = Math.toRadians(180.0);
    public static double WHEEL_RADIUS = 2.0;
    public static double GEAR_RATIO = 1.0;
    public static PIDCoefficients AXIAL_COEFFICIENTS = new PIDCoefficients(1);
    public static PIDCoefficients HEADING_COEFFICIENTS = new PIDCoefficients(1);
    public static FeedforwardCoefficients FF_COEFFICIENTS = new FeedforwardCoefficients(1);

    static TankDrive get(HardwareMap hMap) {
        //Motors left and right both have a wheel radius set to 2.0 inches. Fix that if necessary.
        MotorGroup left = new MotorGroup(
                new Motor(hMap, "front_left", 312.0, 537.7, WHEEL_RADIUS, GEAR_RATIO),
                new Motor(hMap, "back_left", 312.0, 537.7, WHEEL_RADIUS, GEAR_RATIO)
        );
        MotorGroup right = new MotorGroup(
                new Motor(hMap, "front_right", 312.0, 537.7, WHEEL_RADIUS, GEAR_RATIO),
                new Motor(hMap, "back_right", 312.0, 537.7, WHEEL_RADIUS, GEAR_RATIO)
        );
        left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right.setReversed(true);
        left.setFeedforwardCoefficients(FF_COEFFICIENTS);
        right.setFeedforwardCoefficients(FF_COEFFICIENTS);

        return new TankDrive(
                right, left, null,
                new TankConstraints(
                        left.maxRPM,
                        TRACK_WIDTH,
                        MAX_VEL,
                        MAX_ACCEL,
                        MAX_ANG_VEL,
                        MAX_ANG_ACCEL
                ), AXIAL_COEFFICIENTS, HEADING_COEFFICIENTS
        );
    }
}
