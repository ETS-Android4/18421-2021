package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.amarcolini.joos.hardware.Imu;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Imu Test")
public class ImuTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Imu imu = new Imu(hardwareMap, "imu");
        MultipleTelemetry telem = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();

        while(opModeIsActive()) {
            final Orientation orientation = imu.getImu().getAngularOrientation();
            telem.addData("heading", imu.getHeading());
            telem.addData("velocity", imu.getHeadingVelocity());
            telem.addData("pose", imu.getLocalizer().getPoseEstimate());
            telem.addData("orientation", orientation);
            telem.addData("first", orientation.firstAngle);
            telem.addData("second", orientation.secondAngle);
            telem.addData("third", orientation.thirdAngle);
            telem.update();
        }
    }
}
