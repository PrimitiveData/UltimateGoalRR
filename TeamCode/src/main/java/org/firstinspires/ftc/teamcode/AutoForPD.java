package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Disabled
public class AutoForPD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineTo(new Vector2d(50, 5))
                .build();

        waitForStart();

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        drive.followTrajectoryAsync(trajectory);
        while (opModeIsActive() && !isStopRequested()){
            drive.update();
            PoseStorageForPD.currentPose = drive.getPoseEstimate();

            telemetry.addData("Current Pose: ", PoseStorageForPD.currentPose);
            telemetry.update();
        }
    }
}