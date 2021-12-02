package org.firstinspires.ftc.teamcode;

import com.amarcolini.joos.hardware.Imu;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Imu Test")
public class ImuTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Imu imu = new Imu(hardwareMap, "imu");

        waitForStart();

        while(opModeIsActive()) {
            final Orientation orientation = imu.getImu().getAngularOrientation();
            telemetry.addData("heading", imu.getHeading());
            telemetry.addData("velocity", imu.getHeadingVelocity());
            telemetry.addData("pose", imu.getLocalizer().getPoseEstimate());
            telemetry.addData("orientation", orientation);
            telemetry.addData("first", orientation.firstAngle);
            telemetry.addData("second", orientation.secondAngle);
            telemetry.addData("third", orientation.thirdAngle);
//            telemetry.update();
        }
    }
}
