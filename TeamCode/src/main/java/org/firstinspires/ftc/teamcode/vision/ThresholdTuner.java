package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraRotation;

@TeleOp(name="ThresholdTunerOpenCV", group="TeleOp")
public class ThresholdTuner extends LinearOpMode {
    OpenCvCamera webcam;
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        waitForStart();
        ThresholdTunerPipeline pipeline = new ThresholdTunerPipeline(telemetry);
        webcam.setPipeline(pipeline);
        webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        webcam.resumeViewport();
        while(!isStopRequested()){
            telemetry.update();
        }
    }
}
