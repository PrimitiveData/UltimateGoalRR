package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name="6wdArcadeDrive", group="TeleOp")
public class SixWheelDriveBenjamin extends OpMode {
    Hardware hardware;
    public void init(){
        hardware = new Hardware(hardwareMap,telemetry);
    }
    public double logistic(double input, double constantB, double constantC){
        return constantB*(1/(1+ Math.pow(Math.E,-constantC*(input-0.6)))) - constantB/2+0.55;
    }
    public void loop(){
        double translationAbsValue = Math.abs(gamepad1.left_stick_y);
        double turnAbsValue = Math.abs(gamepad1.right_stick_x);
        double translation = logistic(translationAbsValue, 1,7.2) * -gamepad1.left_stick_y/translationAbsValue;
        double turn = logistic(turnAbsValue,1,7.2) * gamepad1.right_stick_x/turnAbsValue;
        double[] powers = setPowers(translation,turn);
        double leftPower = powers[0];
        double rightPower = powers[1];
        hardware.sixWheelDrive.LF.motor.setPower(leftPower);
        hardware.sixWheelDrive.LB.motor.setPower(leftPower);
        hardware.sixWheelDrive.RF.motor.setPower(rightPower);
        hardware.sixWheelDrive.RB.motor.setPower(rightPower);
    }
    /*public double[] setPowers(double forwardsBackPower, double turnPower) {
        double sum = Math.abs(forwardsBackPower) + Math.abs(turnPower);
        double leftTurn = 0;
        double rightTurn = 0;
        if (sum > 1 || sum < -1) {
            if (turnPower > 0)
                leftTurn = turnPower / sum;
            else if (turnPower < 0)
                rightTurn = turnPower / sum * -1;
            double leftForward = forwardsBackPower / sum;
            double rightForward = forwardsBackPower / sum;
            return new double[]{leftTurn-rightTurn+leftForward,rightTurn-leftTurn+rightForward};
        } else {
            if (turnPower > 0) {
                return new double[]{forwardsBackPower+turnPower,forwardsBackPower-turnPower};
            }
            else if (turnPower < 0) {
                return new double[]{forwardsBackPower-turnPower,forwardsBackPower+turnPower};
            }
            else{
                return new double[]{forwardsBackPower,forwardsBackPower};
            }
        }
    }*/
    public double[] setPowers(double forwardBackPower, double turnPower){
        return new double[]{forwardBackPower + turnPower, forwardBackPower - turnPower};
    }
}
