package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class ThreeWheelTrackingLocalizerAnalogGyro extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.7134484467359772; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double FORWARD_OFFSET = -8.07464133858; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 0.97819119737; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.97819119737; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private SampleMecanumDrive drive;

    public ThreeWheelTrackingLocalizerAnalogGyro(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
            new Pose2d(0, 0, 0), // left and right
            new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        this.drive = drive;


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LF"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LB"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RF"));
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches((leftEncoder.getCurrentPosition()+rightEncoder.getCurrentPosition())/2)*X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition())*Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches((leftEncoder.getCorrectedVelocity()+rightEncoder.getCorrectedVelocity())/2)*X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())*Y_MULTIPLIER
        );
    }
}