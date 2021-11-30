package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Camera Test")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        telemetry.addData("", "loading camera...");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(new DuckDetector());
                webcam.startStreaming(800, 600);
                dashboard.startCameraStream(webcam, 60);
                telemetry.addData("", "camera opened!");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("", "camera failed to open.");
                telemetry.update();
            }
        });

        waitForStart();

        while(opModeIsActive()) {
            sleep(500);
        }
    }
}
