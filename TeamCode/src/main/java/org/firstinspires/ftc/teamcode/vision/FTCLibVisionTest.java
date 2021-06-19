package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class FTCLibVisionTest extends LinearOpMode {
    UGContourRingDetector detector;
    UGContourRingPipeline.Height height;
    @Override
    public void runOpMode() throws InterruptedException {
        detector = new UGContourRingDetector(hardwareMap, "Webcam 1", telemetry, true);
        detector.init();
        height = detector.getHeight();
        while(!opModeIsActive()){
            height = detector.getHeight();
        }
        waitForStart();
        while(opModeIsActive()){
            switch (height){
                case ONE:
            }
        }
    }
}
