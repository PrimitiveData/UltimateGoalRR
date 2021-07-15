package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 35.0/25.4/2.0; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15.756207022137104; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -8.07464133858; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 0.9991568; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.0048540; // Multiplier in the Y direction
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public double lastLeft, lastRight, lastFront = 0;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LF"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LB"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RF"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {

        double leftPos = leftEncoder.getCurrentPosition();
        double rightPos = rightEncoder.getCurrentPosition();
        double frontPos = frontEncoder.getCurrentPosition();


        if (leftPos == 0 && Math.abs(lastLeft) > 4000) {
            leftPos = lastLeft;
        }

        if (rightPos == 0 && Math.abs(lastRight) > 4000) {
            rightPos = lastRight;
        }

        if (frontPos == 0 && Math.abs(lastFront) > 4000) {
            frontPos = lastFront;
        }

        lastLeft = leftPos;
        lastRight = rightPos;
        lastFront = frontPos;

        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );

    }

//    @NonNull
//    @Override
//    public List<Double> getWheelPositions() {
//        long leftEncoderPos = leftEncoder.getCurrentPosition();
//        long rightEncoderPos = rightEncoder.getCurrentPosition();
//        long frontEncoderPos = frontEncoder.getCurrentPosition();
//        System.out.println("Encoder Pos:" + leftEncoderPos + "," + rightEncoderPos + "," + frontEncoderPos);
//
//
//
//        return Arrays.asList(
//                encoderTicksToInches(leftEncoderPos)*X_MULTIPLIER,
//                encoderTicksToInches(rightEncoderPos)*X_MULTIPLIER,
//                encoderTicksToInches(frontEncoderPos)*Y_MULTIPLIER
//        );
//    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity())*X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity())*X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())*Y_MULTIPLIER
        );
    }
}
