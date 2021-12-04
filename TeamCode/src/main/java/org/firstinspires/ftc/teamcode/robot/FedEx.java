package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.Servo;
import com.amarcolini.joos.hardware.drive.TankDrive;
import com.amarcolini.joos.kinematics.TankKinematics;
import com.amarcolini.joos.trajectory.config.TankConstraints;
import com.amarcolini.joos.util.DashboardUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class FedEx extends Robot {
    public final Lift lift;
    public final Spinner spinner;
    public final Intake intake;
    public final Conveyor conveyor;
    public final Bucket bucket;
    public final TankDrive drive;
    public final Camera camera;

    //Config variables:
    /*
    If you're having trouble visualizing the positions or dashboard isn't displaying these properly,
    know that positive x is towards the warehouse, and positive y is towards the blue side. (0, 0)
    is the center of the field.
    Additionally, there is a joos_testing module in the code, if you want to try out the GUI. It might not work.
     */
    public static Pose2d start = new Pose2d(-39.00, 62.40, -1.57080);
    public static Double bucketOpenDuration = 0.5;
    public static Double spinnerSpinningDuration = 2.0;
    public static Vector2d shippingHubSpot = new Vector2d(-13.95, 42.80);
    public static Vector2d warehouseSpot = new Vector2d(46.85, 37.85);
    public static Vector2d carouselSpot = new Vector2d(-54.85, 52.30);

    public FedEx(@NonNull OpMode opMode) {
        super(opMode);

        //Initializing components
        lift = new Lift(new Motor(hMap, "lift", 435, 384.5), this);
        bucket = new Bucket(new Servo(hMap, "bucket"));
        //Motors left and right both have a wheel radius set to 2.0 inches. Fix that if necessary.
        Motor left = new Motor(hMap, 312.0, 2.0, 1.0, "front_left", "back_left");
        left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        Motor right = new Motor(hMap, 312.0, 2.0, 1.0, "front_right", "back_right");
        right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //Drive constants are in here, so if trajectory following / location is off, look here
        /*
        These constants are the kind that roadrunner uses, but we haven't gone through the process
        of tuning them. Don't expect these to be accurate. Track width is 18.0 inches (distance from left to right wheels).
        You can try measuring that if things aren't going well. You can also supply PID coefficients after the tank constraints
        if the trajectory following doesn't seem to be correcting itself too well, but the default values should be okay.
         */
        drive = new TankDrive(
                left, right, null,
                new TankConstraints(
                        left.getMaxRPM(),
                        18.0,
                        30.0,
                        30.0,
                        Math.toRadians(90.0),
                        Math.toRadians(100.0)
                )
        );
        spinner = new Spinner(new Motor(hMap, "spinner", 1620));
        intake = new Intake(new Motor(hMap, "intake", 1620));
        conveyor = new Conveyor(new Motor(hMap, "conveyor", 1620));
        camera = new Camera("webcam", this);

        register(lift, bucket, drive, spinner, intake, conveyor, gamepad);
    }

    public void initTeleOp() {
        schedule(Command.of(() -> {
            Vector2d stick = new Vector2d(
                    gamepad.p1.getInternal().left_stick_x,
                    gamepad.p1.getInternal().left_stick_y
            );
            telemetry.addData("stick", stick);
            Pose2d vel = new Pose2d(
                    stick.x, 0,
                    -stick.y
            ).times(2);
            telemetry.addData("power", TankKinematics.robotToWheelVelocities(vel, 1));
            drive.setDrivePower(vel);
        }).requires(drive).runUntil(false));

        map(gamepad.p1.a::justActivated, Command.select(() -> {
            int newLevel = lift.getLevel() + 1;
            if (newLevel > 3) newLevel = 1;
            return lift.setLevel(newLevel);
        }).requires(lift));
        map(() -> gamepad.p1.b.justActivated() && requiring(lift) == null, bucket::toggle);
        map(gamepad.p1.x::justActivated, () -> {
            intake.toggle();
            conveyor.toggle();
        });
        map(gamepad.p1.y::justActivated, spinner::toggle);
    }

    public void initAutoBlueWarehouse() {
        //Starting duck detection
        camera.start();
        //Setting pose estimate
        drive.setPoseEstimate(start);
        getPacket().addLine("This opmode is a test autonomous.");
        getPacket().addLine("The robot should be against the wall behind the barcode nearest to the " +
                "carousel and aligned on the nearest field tile. The camera should be facing the barcode");
        getPacket().addLine("The robot should drive to the carousel, activate the spinner (if it's the wrong direction, " +
                "reverse the spinner), drive to the shipping hub, drop the freight, and drive into the warehouse." +
                "There should be dashboard variables to tweak these values.");
        DashboardUtil.drawRobot(getPacket().fieldOverlay(), drive.getPoseEstimate());
        dashboard.sendTelemetryPacket(getPacket());

        //If spinner needs reversing:
//        spinner.reversed = true;

        //This should draw the robot's position on the field during autonomous.
        schedule(Command.of(() ->
                DashboardUtil.drawRobot(getPacket().fieldOverlay(), drive.getPoseEstimate()))
                .runUntil(false)
        );

        schedule(Command.of(() -> {
            //After op mode starts:
            camera.close();
            schedule(Command.select(() -> {
                //get duck position
                final int level;
                switch(camera.getLastPosition()) {
                    case TWO: level = 1;
                    break;
                    case THREE: level = 2;
                    break;
                    default: level = 3;
                    break;
                }
                getPacket().addLine("duck position: " + level);
                //Go to carousel
                return drive.followTrajectory(drive.trajectoryBuilder()
                        .splineTo(carouselSpot, Math.toRadians(180.0))
                        .build()
                )
                        //Spin spinner
                        .then(spinner::start)
                        .wait(spinnerSpinningDuration)
                        .then(spinner::stop)
                        //Go to shipping hub
                        .then(drive.followTrajectory(drive.trajectoryBuilder()
                                .back(26.322604207555816)
                                .splineTo(shippingHubSpot, Math.toRadians(-90.0))
                                .build()
                        ))
                        //Drop freight
                        .then(lift.setLevel(level))
                        .then(bucket::open)
                        .wait(bucketOpenDuration)
                        .then(bucket::close)
                        //Reset lift
                        .then(lift.setLevel(1))
                        //Go into warehouse
                        .then(drive.followTrajectory(drive.trajectoryBuilder()
                                .splineTo(warehouseSpot, Math.toRadians(0.0))
                                .build()
                        ));
            }));
        }));
    }
}
