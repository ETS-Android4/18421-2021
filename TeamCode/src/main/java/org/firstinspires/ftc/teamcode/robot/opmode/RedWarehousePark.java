package org.firstinspires.ftc.teamcode.robot.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.control.PIDFController;
import com.amarcolini.joos.hardware.Imu;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.Servo;
import com.amarcolini.joos.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Camera;
import org.firstinspires.ftc.teamcode.robot.FedEx;
import org.firstinspires.ftc.teamcode.robot.Lift;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "RedWarehousePark")
@Config
public class RedWarehousePark extends LinearOpMode {
    public static double GEAR_RATIO = 0.9;
    public static PIDCoefficients pid = new PIDCoefficients(0.05, 0, 0);
    public static PIDCoefficients headingPID = new PIDCoefficients(0.5, 0, 0);

    private Motor back_left;
    private Motor front_left;
    private Motor front_right;
    private Motor back_right;
    private List<Motor> motors;
    private Imu imu;
    private PIDFController headingController;
    private double ipt;

    @Override
    public void runOpMode() throws InterruptedException {
        double radius = 2.4;

        back_left = new Motor(hardwareMap, "front_left", 312.0, 537.7, radius, GEAR_RATIO);
        front_left = new Motor(hardwareMap, "back_left", 312.0, 537.7, radius, GEAR_RATIO);
        front_right = new Motor(hardwareMap, "front_right", 312.0, 537.7, radius, GEAR_RATIO);
        back_right = new Motor(hardwareMap, "back_right", 312.0, 537.7, radius, GEAR_RATIO);
        imu = new Imu(hardwareMap, "imu");
        headingController = new PIDFController(headingPID);
        headingController.setInputBounds(-Math.PI, Math.PI);
        imu.setAxis(Imu.Axis.Z);
        motors = Arrays.asList(back_right, back_left, front_right, front_left);
        front_right.setReversed(true);
        back_right.setReversed(true);
//        FedEx bot = new FedEx(this);
        Lift lift = new Lift(new Motor(hardwareMap, "lift", 435, 384.5));
        Bucket bucket = new Bucket(new Servo(hardwareMap, "bucket"));

        double dpr = front_left.getDistancePerRev();
        double tpr = front_left.TPR;
        ipt = tpr / dpr;

        waitForStart();

        straight(-12);
        sleep(500);
        turn(-90);
        sleep(500);
        straight(-43);
    }

    private void straight(double distance, double speed) {
        for (Motor motor: motors) {
            motor.resetEncoder();
            motor.setRunMode(Motor.RunMode.RUN_TO_POSITION);
            motor.setPositionTolerance(10);
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motor.setPositionCoefficients(pid);
            motor.setTargetPosition((int) (motor.getCurrentPosition() + distance * ipt));
            motor.setPower(speed);
        }
        while (back_left.isBusy() && front_left.isBusy() && back_right.isBusy() && front_right.isBusy()) {
            for (Motor motor: motors) {
                motor.update();
            }
        }
        for (Motor motor: motors) {
            motor.setPower(0.0);
        }
    }
    private void straight(double distance) {
        straight(distance, 1.0);
    }

    private void turn(double angle) {
        for (Motor motor: motors) {
            motor.setRunMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double start = imu.getHeading();
        headingController.setTargetPosition(Angle.norm(start + Math.toRadians(angle)));
        headingController.setTolerance(Math.toRadians(10.0));
        headingController.update(Angle.norm(imu.getHeading()));
        while (Math.abs(headingController.getLastError()) < Math.toRadians(20.0)) {
            double output = headingController.update(Angle.norm(imu.getHeading()));
            back_left.setPower(output);
            front_left.setPower(output);
            front_right.setPower(-output);
            back_right.setPower(-output);
        }
        for (Motor motor: motors) {
            motor.setPower(0.0);
        }
    }
}
