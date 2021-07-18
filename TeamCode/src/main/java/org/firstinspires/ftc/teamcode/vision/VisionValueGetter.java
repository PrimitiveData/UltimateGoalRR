package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraRotation;

@Disabled
public class VisionValueGetter extends LinearOpMode {
    OpenCvCamera webcam;
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        waitForStart();
        VisionValueGetterPipeline pipeline = new VisionValueGetterPipeline(510,230,530,250);
        webcam.setPipeline(pipeline);
        webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        webcam.resumeViewport();
        while(!isStopRequested()){
            pipeline.TRx-=gamepad1.left_stick_x*0.05;
            pipeline.TRy-=gamepad1.left_stick_y*0.05;
            pipeline.BLx-=gamepad1.right_stick_x*0.05;
            pipeline.BLy-=gamepad1.right_stick_y*0.05;
            telemetry.addLine("BLx: "+pipeline.BLx+", BLy: " + pipeline.BLy+", TRx: "+pipeline.TRx +", TRy: "+pipeline.TRy);
            telemetry.addLine("1: "+ pipeline.highlightedBox[0] + ", 2: "+pipeline.highlightedBox[1] + ", 3: "+pipeline.highlightedBox[2]);
            telemetry.addLine("range for 1: " + pipeline.rangeValue1[0] + " to " + pipeline.rangeValue1[1]);
            telemetry.addLine("range for 2: " + pipeline.rangeValue2[0] + " to " + pipeline.rangeValue2[1]);
            telemetry.addLine("range for 3: " + pipeline.rangeValue3[0] + " to " + pipeline.rangeValue3[1]);
            telemetry.update();
            sleep(10);
        }
    }
}
