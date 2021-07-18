package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

@Disabled
public class MecanumDriveTeleop extends OpMode {
    HardwareMecanum hardware;
    public void init(){
        if(hardwareMap==null){

            telemetry.addLine("hwmap  null");
            telemetry.update();
        }
        else{
            telemetry.addLine("hwmap not null");
            telemetry.update();

        }

        hardware = new HardwareMecanum(hardwareMap,telemetry);

    }
    public void loop(){
        /*
        hardware.mecanumDrive.setPowers(-gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x);
        hardware.loop();

        telemetry.addLine("left position: " + hardware.previousPortReading + ", right position: " + hardware.previousStarboardReading + ", lateral position: " + hardware.previousLateralReading);
        telemetry.addLine("angle: "+hardware.angle + ", in degrees: "+ Math.toDegrees(hardware.angle) + ", from odo: "+ Math.toDegrees(hardware.angleOdo));
        telemetry.addLine("angle 1: "+ Math.toDegrees(hardware.angle1) + ", angle 2: "+ Math.toDegrees(hardware.angle2));
        telemetry.addLine("X: " + hardware.getX()+ ", Y: "+ hardware.getY() + ", angle: " + hardware.angle);
        telemetry.addLine("XAlt: " + hardware.xPosTicksAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAlt: "+hardware.yPosTicksAlt* Hardware.circumfrence/ Hardware.ticks_per_rotation);
        telemetry.addLine("XAlt2: " + hardware.xPosTicksAlt2 * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAlt2: "+hardware.yPosTicksAlt2* Hardware.circumfrence/ Hardware.ticks_per_rotation);
        telemetry.addLine("XAltAlt: "+ hardware.xPosTicksAltAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAltAlt: " + hardware.yPosTicksAltAlt * Hardware.circumfrence/ Hardware.ticks_per_rotation);
        telemetry.addLine("angularVeloTracker: "+hardware.integratedAngularVeloTracker);
        telemetry.addLine("loops/sec: " + (hardware.loops / ((hardware.time.milliseconds()-hardware.startTime)/1000)));
         */
    }
}
