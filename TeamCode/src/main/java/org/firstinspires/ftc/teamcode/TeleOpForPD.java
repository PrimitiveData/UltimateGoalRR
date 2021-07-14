package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
@TeleOp
public class TeleOpForPD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        drive.setPoseEstimate(PoseStorageForPD.currentPose);

        while(!opModeIsActive()) {
            telemetry.addData("Current Pose: ", PoseStorageForPD.currentPose);
            telemetry.update();

            if(isStopRequested() || isStarted()) {
                break;
            }
        }

        while(opModeIsActive()) {
            drive.setWeightedDrivePower(new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            telemetry.addData("Current Pose: ", PoseStorageForPD.currentPose);
            telemetry.update();
        }
    }
}
