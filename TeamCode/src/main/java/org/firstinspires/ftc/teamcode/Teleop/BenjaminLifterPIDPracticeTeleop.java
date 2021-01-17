package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.PID.BenjaminLifterPIDPractice;

public class BenjaminLifterPIDPracticeTeleop extends LinearOpMode {
    public DcMotor lifter;
    BenjaminLifterPIDPractice pid;
    ElapsedTime time;
    int encoderPos;
    int targetPos;

    public void waitForStart() {
        super.waitForStart();
    }
    public void runOpMode() {
        lifter = hardwareMap.get(DcMotor.class, "Lifter");
        time = new ElapsedTime();
        encoderPos = lifter.getCurrentPosition();
        pid = new BenjaminLifterPIDPractice(1,1,1,3,time);

        while(!isStopRequested()){
            encoderPos = lifter.getCurrentPosition();
            pid.setState(targetPos);
            lifter.setPower(pid.updateCurrentStateAndGetOutput(encoderPos));
        }
    }
}
