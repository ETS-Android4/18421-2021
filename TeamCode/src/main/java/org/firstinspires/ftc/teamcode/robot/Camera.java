package org.firstinspires.ftc.teamcode.robot;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.command.InstantCommand;
import com.amarcolini.joos.command.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DuckDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera implements Component {
    private final Robot robot;
    private final OpenCvWebcam webcam;
    private final DuckDetector detector = new DuckDetector();

    public Camera(String webcamName, Robot robot) {
        this.robot = robot;
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(
                robot.hMap.get(WebcamName.class, webcamName)
        );
    }

    public void start() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(detector);
                webcam.startStreaming(800, 600);
                robot.dashboard.startCameraStream(webcam, 60);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void stop() {
        webcam.stopStreaming();
    }

    public DuckDetector.Position getLastPosition() {
        return detector.getLastPosition();
    }

    public void close() {
        webcam.closeCameraDevice();
    }
}
