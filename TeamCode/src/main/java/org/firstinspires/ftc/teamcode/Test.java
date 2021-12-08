package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.amarcolini.joos.command.CommandScheduler;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.hardware.MotorGroup;
import com.amarcolini.joos.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Debug")
public class Test extends LinearOpMode {
    private Motor intake;
    private Motor conveyor;
    private Motor spinner;
    private Motor lift;
    private Servo bucket;
    private MotorGroup drive;

    public static double intakeSpeed = 0.0;
    public static double conveyorSpeed = 0.0;
    public static double spinnerSpeed = 0.0;
    public static int liftPosition = 0;
    public static double liftSpeed = 0;
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(1.0);
    public static double bucketPosition = 1.0;
    public static double driveSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Motor(hardwareMap, "intake", 1620);
        conveyor = new Motor(hardwareMap, "conveyor", 1620);
        spinner = new Motor(hardwareMap, "spinner", 1620);
        lift = new Motor(hardwareMap, "lift", 312, 537.7);
        bucket = new Servo(hardwareMap, "bucket");
        drive = new MotorGroup(
                new Motor(hardwareMap, "front_left", 312),
                new Motor(hardwareMap, "back_left", 312),
                new Motor(hardwareMap, "front_right", 312),
                new Motor(hardwareMap, "back_right", 312)
        );
//        drive = new Motor(hardwareMap, 312, "front_left", "back_left", "front_right", "back_right");
        MultipleTelemetry telem = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        
        lift.setRunMode(Motor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(0);
        lift.setPositionCoefficients(liftCoefficients);
        lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            intake.setPower(intakeSpeed);
            conveyor.setPower(conveyorSpeed);
            spinner.setPower(spinnerSpeed);
            lift.setPositionCoefficients(liftCoefficients);
            lift.setTargetPosition(liftPosition);
            lift.setPower(liftSpeed);
            bucket.setPosition(bucketPosition);
            drive.setPower(driveSpeed);
            telem.addData("bucket position", bucket.getPosition());
            telem.addData("lift position", lift.getCurrentPosition());
            telem.update();
        }
    }
}
