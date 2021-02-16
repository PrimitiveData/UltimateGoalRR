package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class MathFunctions {
    public static double constrainAngleRange(double angle){
        //takes radian difference in heading between global and local turret angle and constrains the local to a -pi, pi range
        angle = angle % (Math.PI*2);
        if(angle > Math.PI)
            angle -= Math.PI;
        if(angle < Math.PI)
            angle += Math.PI;
        return angle;
    }
    public static double correctedTargetWithinRange(double currentTurretAngle, double targetLocalTurretAngle, double maxNegative, double maxPositive){
        //takes currentangle, targetangle, max, min, and picks the most effecient angle without exceeding the range
        double correctedTarget;
        double delta = (targetLocalTurretAngle - maxNegative)/(Math.PI * 2);
        int deltaInt = (int) delta;

        if(delta < 0)
            deltaInt -= 1;
        correctedTarget = targetLocalTurretAngle - (Math.PI * 2 * deltaInt);

        double differenceAngle = Math.abs(currentTurretAngle - correctedTarget);
        double nextCorrectedTarget;
        while (true){
            nextCorrectedTarget = correctedTarget + (Math.PI * 2);
            if(nextCorrectedTarget > maxPositive)
                return correctedTarget;
            double difference = Math.abs(currentTurretAngle - nextCorrectedTarget);
            if(difference >= differenceAngle)
                return correctedTarget;
            correctedTarget = nextCorrectedTarget;
            differenceAngle = difference;
        }
    }
    public static double correctedTargetServoScale(double currentMagAngle, double targetAngle, double maxNegative, double maxPositive){
        double uncorrectedTarget = correctedTargetWithinRange(currentMagAngle, targetAngle, maxNegative, maxPositive); //returns target value between maxNegative and maxPositive
        double adjustRangeto0Min = uncorrectedTarget - maxNegative; //adjusts output into 0 to (maxNegative + maxPositive)
        double range = maxPositive - maxNegative; //total range
        double convertToServoRange = adjustRangeto0Min/range; //divides radian measure by range to get value between 0 and 1 for servo
        return convertToServoRange;
    }
    public static double keepAngleWithin180Degrees(double angle){
        while(angle > Math.toRadians(180)){
            angle -= Math.toRadians(360);
        }
        while(angle < -Math.toRadians(180)){
            angle += Math.toRadians(360);
        }
        return angle;
    }
    public static double keepAngleWithinSetRange(double rangeBottom, double rangeTop, double angle){
        while(angle > rangeTop){
            angle -= Math.toRadians(360);
        }
        while(angle < rangeBottom){
            angle += Math.toRadians(360);
        }
        return angle;
    }
    public static double[] findIntercept(double equation1Slope, double equation1YIntercept, double equation2Slope, double equation2YIntercept){
        double x = (equation2YIntercept-equation1YIntercept)/(equation1Slope-equation2Slope);
        double y = equation1Slope * x + equation1YIntercept;
        double[] point = new double[2];
        point[0] = x;
        point[1] = y;
        return point;
    }
    public static boolean isInBetween(double startNum, double endNum, double testPoint){
        if(startNum > endNum){
            double startNumStorage = startNum;
            startNum = endNum;
            endNum = startNumStorage;
        }
        return testPoint <= endNum && testPoint >= startNum;
    }
    public static double[] lineToPointDistanceAndClosestPoint(double startPointX, double startPointY, double endPointX, double endPointY, double setPointX, double setPointY){
        if(endPointY==startPointY){
            if(isInBetween(startPointX,endPointX, setPointX)){
                double[] outputData = new double[3];
                outputData[0] = Math.abs(startPointY-setPointY);
                outputData[1] = setPointX;
                outputData[2] = startPointY;
                return outputData;
            }
            else{
                double startPointDist = Math.hypot((startPointX-setPointX), (startPointY-setPointY));
                double endPointDist = Math.hypot((endPointX-setPointX), (endPointY-setPointY));
                if(startPointDist < endPointDist){
                    double[] outputData = new double[3];
                    outputData[0] = startPointDist;
                    outputData[1] = startPointX;
                    outputData[2] = startPointY;
                    return outputData;
                }
                else{
                    double[] outputData = new double[3];
                    outputData[0] = endPointDist;
                    outputData[1] = endPointX;
                    outputData[2] = endPointY;
                    return  outputData;
                }
            }
        }
        if(endPointX==startPointX){
            if(isInBetween(startPointY,endPointY, setPointY)){
                double[] outputData = new double[3];
                outputData[0] = Math.abs(startPointX-setPointX);
                outputData[1] = startPointX;
                outputData[2] = setPointY;
                return outputData;
            }
            else{
                double startPointDist = Math.hypot((startPointX-setPointX), (startPointY-setPointY));
                double endPointDist = Math.hypot((endPointX-setPointX), (endPointY-setPointY));
                if(startPointDist < endPointDist){
                    double[] outputData = new double[3];
                    outputData[0] = startPointDist;
                    outputData[1] = startPointX;
                    outputData[2] = startPointY;
                    return outputData;
                }
                else{
                    double[] outputData = new double[3];
                    outputData[0] = endPointDist;
                    outputData[1] = endPointX;
                    outputData[2] = endPointY;
                    return  outputData;
                }
            }
        }
        double slope = (endPointY-startPointY)/(endPointX-startPointX);
        double yIntercept = startPointY - slope * startPointX;
        double inverseSlope = -1/slope;
        double inverseYIntercept = setPointY - inverseSlope * setPointX;
        double[] intercept = findIntercept(slope, yIntercept, inverseSlope, inverseYIntercept);
        if(isInBetween(startPointX, endPointX, intercept[0])){
            double[] outputData = new double[3];
            outputData[0] = Math.hypot((intercept[0] - setPointX), (intercept[1] - setPointY));
            outputData[1] = intercept[0];
            outputData[2] = intercept[1];
            return outputData;
        }
        else{
            double startPointDist = Math.hypot((startPointX-setPointX), (startPointY-setPointY));
            double endPointDist = Math.hypot((endPointX-setPointX), (endPointY-setPointY));
            if(startPointDist < endPointDist){
                double[] outputData = new double[3];
                outputData[0] = startPointDist;
                outputData[1] = startPointX;
                outputData[2] = startPointY;
                return outputData;
            }
            else{
                double[] outputData = new double[3];
                outputData[0] = endPointDist;
                outputData[1] = endPointX;
                outputData[2] = endPointY;
                return  outputData;
            }
        }
    }
    public static double findPercentageOfCurrentLineTravelled(double startPointX, double startPointY, double endPointX, double endPointY, double setPointX, double setPointY){
        return Math.hypot(setPointX - startPointX, setPointY-startPointY)/ Math.hypot(endPointX-startPointX,endPointY-startPointY);
    }

    public static double velocityCommand(double desiredVelocity, double headingError, double xError, double constantK){
        return desiredVelocity* Math.cos(headingError) + constantK * xError;
    }
    public static double getConstantK(double desiredVelocity, double desiredAngularVelocity, double constantB, double constantC){
        return 2*constantC* Math.sqrt(Math.pow(desiredAngularVelocity,2) + constantB* Math.pow(desiredVelocity,2));
    }
    public static double getAngularVelocityCommand(double desiredVelocity, double desiredAngularVelocity, double headingError, double yError, double constantB, double constantK){
        if(Math.abs(headingError)>0.1) {
            return desiredAngularVelocity + constantK * headingError + constantB * desiredVelocity * Math.sin(headingError) / headingError * yError;
        }
        else{
            return desiredAngularVelocity + constantK * headingError + constantB * desiredVelocity * yError;
        }
    }

    public static double angleFormatting(double angle) {
        if(angle > Math.toRadians(180)) {
            angle -= Math.toRadians(360);
        }
        if(angle < Math.toRadians(-180)) {
            angle += Math.toRadians(360);
        }
        return angle;
    }

    public static double[] transposeCoordinate(double originalX, double originalY, double transposeDistance, double heading){
        return new double[]{originalX + transposeDistance* Math.cos(heading),originalY+transposeDistance* Math.sin(heading)};
    }
    public static double[] transposeCoordinate(double originalX, double originalY, double localXFromOriginalToTarget, double localYFromOriginalToTarget, double heading){
        return new double[]{originalX + localXFromOriginalToTarget * Math.cos(heading)-localYFromOriginalToTarget* Math.sin(heading),originalY + localXFromOriginalToTarget* Math.sin(heading) + localYFromOriginalToTarget* Math.cos(heading) };
    }
}
