package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.amarcolini.joos.drive.Drive;
import com.amarcolini.joos.drive.DriveSignal;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.hardware.Imu;
import com.amarcolini.joos.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Imu Test")
public class ImuTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Imu imu = new Imu(hardwareMap, "imu");
        MultipleTelemetry telem = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();

        final Imu.Axis axis = imu.autoDetectUpAxis();
        /*
        Heading axis is Z for vertical hub with motor ports facing up
         */

        while(opModeIsActive()) {
            final Orientation orientation = imu.getImu().getAngularOrientation();
            final Orientation extrinsic = imu.getImu().getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telem.addData("heading", imu.getHeading());
            telem.addData("velocity", imu.getHeadingVelocity());
            telem.addData("pose", imu.getLocalizer().getPoseEstimate());
            telem.addData("orientation", orientation);
            telem.addData("extrinsic", extrinsic);
            telem.addData("predicted up axis", axis);
            telem.addData("first", orientation.firstAngle);
            telem.addData("second", orientation.secondAngle);
            telem.addData("third", orientation.thirdAngle);
            telem.update();
        }
    }
}
