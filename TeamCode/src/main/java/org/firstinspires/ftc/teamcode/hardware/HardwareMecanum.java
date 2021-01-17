package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.vision.T265;

import java.util.List;

public class HardwareMecanum {
    HardwareMap hardwareMap;
    static HardwareMecanum hw;
    public BNO055IMU imu;
    public BNO055IMU imu2;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public double angle;
    public double angleOdo;
    public double previousAngleOdoReading;
    public List<LynxModule> allHubs;
    private double centerWheelOffset;
    private double centerWheelOffsetBase = 5.23;
    private double centerWheelOffsetChange = 0;
    public static double trackWidth = 13.781;
    private static double portOffset = 6.532; // inches
    private static double starboardOffset = 6.985; //inches
    public static double odoWidth =portOffset+starboardOffset;
    public double xPosTicks = 0;
    public double yPosTicks = 0;
    public double xPosInches = 0;
    public double yPosInches = 0;
    public double yPosTicksAlt=0;
    public double xPosTicksAlt=0;
    public double yPosTicksAlt2=0;
    public double xPosTicksAlt2=0;
    public double xPosTicksAltAlt = 0;
    public double yPosTicksAltAlt = 0;
    public int previousPortReading;
    public int previousStarboardReading;
    public int previousLateralReading;
    public double previousAngleReading;
    public double localYVelocity;
    private double prevW;
    private double prevLocalXVelo;
    private double prevLocalYVelo;
    public static double ticks_per_rotation = 8192;
    //38mm diameter, converted to inches = 1.4960630
    public static double driveWheelRadius = 1.41732;
    public static double odoWheelRadius = 0.6889764;
    public static double circumfrence = odoWheelRadius * 2 * Math.PI / (92.7070970616/96);
    public int ticker = 0;
    public Motor[] hub1Motors;
    public Motor[] hub2Motors;
    public RegServo[] servos;
    public MecanumDrive mecanumDrive;
    public ElapsedTime time;
    public int loops = 0;
    public boolean firstLoop = true;
    public double startTime;
    public double prevTime;
    String TAG = "odo";
    public boolean updatePID;
    public double prevLocalY;
    public double banglePrev=0;
    double canglePrev=0;
    public double danglePrev=0;
    public double localY;
    public double localX;
    public double integratedAngularVeloTracker;
    public double deltaTime;
    public boolean currentlyForwardDirection = true;
    public double batteryVoltage;
    public static Telemetry telemetry;
    public double angle1 = 0;
    public double angle2 = 0;
    public HardwareMecanum(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        hw = this;
        HardwareMecanum.telemetry = telemetry;
        telemetry.addLine("batteryvoltage: "+batteryVoltage);
        telemetry.update();
        updatePID = false;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu2 = hardwareMap.get(BNO055IMU.class,"imu2");
        imu2.initialize(parameters);
        angle = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        previousAngleReading = angle;
        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        hub1Motors = new Motor[4];//initialize here
        hub1Motors[0] = new Motor( hardwareMap.get(DcMotorEx.class,"LF"));
        hub1Motors[0].motor.setDirection(DcMotorEx.Direction.REVERSE);
        hub1Motors[1] = new Motor(hardwareMap.get(DcMotorEx.class,"LB"));
        hub1Motors[1].motor.setDirection(DcMotorEx.Direction.REVERSE);
        hub1Motors[2] = new Motor(hardwareMap.get(DcMotorEx.class,"RF"));
        hub1Motors[2].motor.setDirection(DcMotorEx.Direction.REVERSE);
        hub1Motors[3] = new Motor(hardwareMap.get(DcMotorEx.class,"RB"));
        hub1Motors[3].motor.setDirection(DcMotorEx.Direction.FORWARD);
        time = new ElapsedTime();
        hub2Motors = new Motor[4];//initialize here
        servos = new RegServo[12];//initialize here
        setForward();
        mecanumDrive = new MecanumDrive(hub1Motors[0],hub1Motors[1],hub1Motors[2],hub1Motors[3]);

    }
    public void setForward(){
        centerWheelOffset = centerWheelOffsetBase+centerWheelOffsetChange;
        if(!currentlyForwardDirection){
            xPosTicks+=2*centerWheelOffsetChange*ticks_per_rotation/circumfrence* Math.cos(angle);
            yPosTicks+=2*centerWheelOffsetChange*ticks_per_rotation/circumfrence* Math.sin(angle);
        }
        currentlyForwardDirection = true;
    }
    public void setBackwards(){
        centerWheelOffset = centerWheelOffsetBase-centerWheelOffsetChange;
        if(currentlyForwardDirection){
            xPosTicks-=2*centerWheelOffsetChange*ticks_per_rotation/circumfrence* Math.cos(angle);
            yPosTicks-=2*centerWheelOffsetChange*ticks_per_rotation/circumfrence* Math.sin(angle);
        }
        currentlyForwardDirection = false;
    }
    public void loop(){

        loops++;
        double deltaAngle=0;
        double deltaAngleOdo=0;
        if(ticker % 8 == 0) {
            double bangle = MathFunctions.keepAngleWithin180Degrees(Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
            double dangle = MathFunctions.keepAngleWithin180Degrees(Math.toRadians(imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
            double deltaAngle1 = 360/358.638192308*360/360.58242287*360/356.851325758*360/359.83265011* MathFunctions.keepAngleWithin180Degrees(bangle - banglePrev);
            double deltaAngle2 = 360/360.4173125*360/358.811653846*360/359.75* MathFunctions.keepAngleWithin180Degrees(dangle - danglePrev);
            banglePrev = bangle;
            danglePrev = dangle;
            angle1+=deltaAngle1;
            angle2+=deltaAngle2;
            angle = (deltaAngle1+deltaAngle2)/2 + canglePrev;
            deltaAngle = angle-previousAngleReading;
            canglePrev = angle;
            ticker = 1;
        }
        hub1Motors[0].readRequested = true;
        hub1Motors[1].readRequested = true;
        hub1Motors[3].readRequested = true;
        boolean hub1ReadNeeded = true;
        for(Motor motor: hub1Motors){
            if(motor != null && motor.readRequested){
                hub1ReadNeeded = true;
            }
        }
        allHubs.get(0).clearBulkCache();
        for(Motor motor: hub1Motors){
            if(motor != null && motor.readRequested){
                motor.currentPosition = motor.motor.getCurrentPosition();
                motor.currentVelocity = motor.motor.getVelocity(AngleUnit.RADIANS);
            }
        }
        boolean hub2ReadNeeded = false;
        for(Motor motor: hub2Motors){
            if(motor != null && motor.readRequested)
                hub2ReadNeeded = true;
        }
        allHubs.get(1).clearBulkCache(); // depends on which one is not the odo hub
        for(Motor motor: hub2Motors){
            if(motor != null && motor.readRequested){
                motor.currentPosition = motor.motor.getCurrentPosition();
                motor.currentVelocity = motor.motor.getVelocity(AngleUnit.RADIANS);
            }
        }

            int portReading=-hub1Motors[3].getCurrentPosition();
            int starboardReading=hub1Motors[1].getCurrentPosition();
            int lateralReading=-hub1Motors[0].getCurrentPosition();
        if(firstLoop){
            startTime = time.milliseconds();
            prevTime = startTime;
            previousLateralReading = lateralReading;
            previousStarboardReading = starboardReading;
            previousPortReading = portReading;
            previousAngleReading = angle;
            previousAngleOdoReading = angle;
            prevLocalY = 0;
            firstLoop = false;
            T265.slamra.start();
        }
        double currentTime = time.milliseconds();
        deltaTime = currentTime-prevTime;
       int PortChange = portReading - previousPortReading;
            int StarboardChange = starboardReading - previousStarboardReading;
        if(ticker%8!=0) {
            deltaAngleOdo  =  360/363.090869374*360/361.49102385*360/358.47252*360/361.801642105*360/358.753786593*(StarboardChange - PortChange) / (odoWidth * ticks_per_rotation / circumfrence);
            angle += deltaAngleOdo;
            deltaAngle = deltaAngleOdo;
            ticker++;
        }
            double deltaAngleBodo  =  360/356.933974359*360/363.090869374*360/361.49102385*360/358.47252*360/361.801642105*360/358.753786593*(StarboardChange - PortChange) / (odoWidth * ticks_per_rotation / circumfrence);
            angleOdo +=deltaAngleBodo;
            double localXAlt = lateralReading - previousLateralReading;//-(deltaAngle * centerWheelOffset * (ticks_per_rotation/circumfrence));
             localX =  lateralReading - previousLateralReading-(deltaAngle * centerWheelOffset * (ticks_per_rotation/circumfrence));

        /*double localY = (PortChange + (deltaAngle * portOffset * (ticks_per_rotation/circumfrence)) + StarboardChange - (deltaAngle * starboardOffset * (ticks_per_rotation/circumfrence)))/2.0;
 */         double localYAlt = StarboardChange; //
            double localYAlt2 = PortChange;
             localY = (PortChange+ (deltaAngle * portOffset * (ticks_per_rotation/circumfrence)) + StarboardChange - (deltaAngle * starboardOffset * (ticks_per_rotation/circumfrence)))/2;
            RobotLog.dd(TAG, "lateralReading: " + lateralReading);
            RobotLog.dd(TAG, "prevlateral: " + previousLateralReading);
            RobotLog.dd(TAG, "portchange: " + PortChange);
            RobotLog.dd(TAG, "deltaAngle: " + deltaAngle+", prevAngle: " +previousAngleReading);
            RobotLog.dd(TAG, "localX: "+localX);
            RobotLog.dd(TAG,"localY: "+localY);
            if(localX == 0 && localY == 0){

            }
            else {
                yPosTicks += (localY) * Math.sin(previousAngleReading + deltaAngle * 0.5) - (localX) * Math.cos(previousAngleReading + deltaAngle * 0.5);//updates Y position
                xPosTicks += (localY) * Math.cos(previousAngleReading + deltaAngle * 0.5) + (localX) * Math.sin(previousAngleReading + deltaAngle * 0.5);//updates X position
                yPosTicksAlt += (localYAlt) * Math.sin(previousAngleReading + deltaAngle * 0.5) - (localXAlt) * Math.cos(previousAngleReading + deltaAngle * 0.5);//updates Y position
                xPosTicksAlt += (localYAlt) * Math.cos(previousAngleReading + deltaAngle * 0.5) + (localXAlt) * Math.sin(previousAngleReading + deltaAngle * 0.5);//updates X position
                yPosTicksAlt2 += (localYAlt2) * Math.sin(previousAngleReading + deltaAngle * 0.5) - (localXAlt) * Math.cos(previousAngleReading + deltaAngle * 0.5);//updates Y position
                xPosTicksAlt2 += (localYAlt2) * Math.cos(previousAngleReading + deltaAngle * 0.5) + (localXAlt) * Math.sin(previousAngleReading + deltaAngle * 0.5);//updates X position
                yPosTicksAltAlt += (localY) * Math.sin(previousAngleOdoReading + deltaAngleOdo * 0.5) - (localX) * Math.cos(previousAngleOdoReading + deltaAngleOdo * 0.5);//updates Y position
                xPosTicksAltAlt += (localY) * Math.cos(previousAngleOdoReading + deltaAngleOdo * 0.5) + (localX) * Math.sin(previousAngleOdoReading + deltaAngleOdo * 0.5);//updates X position
                RobotLog.dd(TAG, "yPosTicks increment: " + ((localY) * Math.sin(previousAngleReading + deltaAngle * 0.5) - (localX) * Math.cos(previousAngleReading + deltaAngle * 0.5)));//updates Y position)
                RobotLog.dd(TAG, "xPosTicks increment: " + ((localY) * Math.cos(previousAngleReading + deltaAngle * 0.5) + (localX) * Math.sin(previousAngleReading + deltaAngle * 0.5)));//updates X position))
                RobotLog.dd(TAG, "xPosTicks: " + xPosTicks);
                RobotLog.dd(TAG, "yPosTicks: " + yPosTicks);
            }
            /*
            double portVelocity=0;
            double starboardVelocity=0;
            double lateralVelocity =0;
            double w = (starboardVelocity-portVelocity)/odoWidth;
            double localXVelocity = lateralVelocity - (w * centerWheelOffset * (ticks_per_rotation/circumfrence));
            double localYVelocity = (portVelocity + (w*portOffset * (ticks_per_rotation/circumfrence)) + starboardVelocity - (w * starboardOffset * (ticks_per_rotation/circumfrence)))/2.0;
            double Twy = 2*(localY)/(prevLocalYVelo+localYVelocity);
            double Twx = 2*(localX)/(prevLocalXVelo+localXVelocity);
            double Tg = 2*(deltaAngle)/(w + prevW);
            double angularAccel = (w - prevW)/Tg;
            long ClocalYf = OdoMath.C(0.56419*(Twy * prevW + Tg * angularAccel*Tg)/(Twy*Math.sqrt(angularAccel)));
            long Ci = OdoMath.C(0.56419*(prevW)/(Math.sqrt(angularAccel)));
            long SlocalYf = OdoMath.S(0.56419*(Twy * prevW + Tg * angularAccel*Tg)/(Twy*Math.sqrt(angularAccel)));
            long Si = OdoMath.S(0.56419*(prevW)/(Math.sqrt(angularAccel)));
            long ClocalXf = OdoMath.C(0.56419*(Twx * prevW + Tg * angularAccel*Tg)/(Twx*Math.sqrt(angularAccel)));
            long SlocalXf = OdoMath.S(0.56419*(Twx * prevW + Tg * angularAccel*Tg)/(Twx*Math.sqrt(angularAccel)));
            double globalYchangeForLocalYChange = prevLocalYVelo*1.77245*Twy/(Tg * Math.sqrt(angularAccel))*((Math.cos(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading)*ClocalYf + Math.sin(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading) * SlocalYf) -(Math.cos(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading)*Ci + Math.sin(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading) * Si));
            double globalXchangeForLocalYChange = prevLocalYVelo*1.77245*Twy/(Tg * Math.sqrt(angularAccel))*((Math.cos(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading)*SlocalYf - Math.sin(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading) * ClocalYf) -(Math.cos(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading)*Si - Math.sin(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading) * Ci));
            double globalYchangeForLocalXChange = prevLocalXVelo*1.77245*Twx/(Tg * Math.sqrt(angularAccel))*((Math.cos(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90)))*ClocalXf + Math.sin(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90))) * SlocalXf) -(Math.cos(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90)))*Ci + Math.sin(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90))) * Si));
            double globalXchangeForLocalXChange = prevLocalXVelo*1.77245*Twx/(Tg * Math.sqrt(angularAccel))*((Math.cos(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90)))*SlocalXf - Math.sin(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90))) * ClocalXf) -(Math.cos(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90)))*Si - Math.sin(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90))) * Ci));
            prevW = w;
            prevLocalXVelo = localXVelocity;
            prevLocalYVelo = localYVelocity;
            previousPortReading = portReading;
            previousStarboardReading = starboardReading;
            previousLateralReading = (int)localX;
            previousAngleReading = angle;

             */
        double localXCameraVelo = (-PortChange*circumfrence/ticks_per_rotation-8.5*deltaAngle - StarboardChange*circumfrence/ticks_per_rotation + 4.875*deltaAngle)/2/deltaTime *0.0254;
        double localYCameraVelo = ((lateralReading-previousLateralReading)*circumfrence/ticks_per_rotation-1.75*deltaAngle)/deltaTime*0.0254;
        //T265.slamra.sendOdometry(localXCameraVelo,localYCameraVelo);
        for(Motor motor: hub1Motors){
                if(motor!=null&&motor.setTargetPosRequested){
                    motor.motor.setTargetPosition(motor.targetPosition);
                    motor.setTargetPosRequested = false;

                }
            }
            for(Motor motor: hub1Motors){
                if(motor!=null&&motor.writePowerRequested){
                    motor.motor.setPower(motor.power);
                    motor.writePowerRequested = false;
                }
                if(motor!=null&&motor.writeVelocityRequested){
                    motor.motor.setVelocity(motor.velocity);
                    motor.writeVelocityRequested = false;
                }
            }
            for(Motor motor: hub2Motors){
                if(motor!=null&&motor.setTargetPosRequested){
                    motor.motor.setTargetPosition(motor.targetPosition);
                    motor.setTargetPosRequested = false;
                }
            }
            for(Motor motor: hub2Motors){
                if(motor!=null&&motor.writePowerRequested){
                    motor.motor.setPower(motor.power);
                    motor.writePowerRequested = false;
                }
                if(motor!=null&&motor.writeVelocityRequested){
                    motor.motor.setVelocity(motor.velocity);
                    motor.writeVelocityRequested = false;
                }
            }
            for(RegServo servo: servos){
                if(servo!=null&&servo.writeRequested){
                    servo.servo.setPosition(servo.position);
                    servo.writeRequested = false;
                }
            }
        previousLateralReading = lateralReading;
            previousAngleReading = angle;
            previousPortReading = portReading;
            previousStarboardReading = starboardReading;
            prevTime = currentTime;
            prevLocalY = localYVelocity;
            previousAngleOdoReading = angleOdo;
    }
    public double getX(){
        xPosInches = (double)xPosTicks * circumfrence / ticks_per_rotation;
        return xPosInches;
    }
    public double getY(){
        yPosInches = (double)yPosTicks * circumfrence / ticks_per_rotation;
        return yPosInches;
    }
    public double getXMeters(){
        return getX() /39.3701;
    }
    public double getYMeters(){
        return getY()/39.3701;
    }
    public static HardwareMecanum getInstance(){
        return hw;
    }
}
