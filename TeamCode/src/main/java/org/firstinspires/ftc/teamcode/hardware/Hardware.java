package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Intake;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Shooter;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Turret;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.WobblerArm;

import java.util.List;

public class Hardware {
    public int GYRO_UPDATE_FREQ = 10;
    public static double yPosTicksClassVariable = 0;
    public static double xPosTicksClassVariable = 0;
    public static double angleClassVariable = 0;
    public HardwareMap hardwareMap;
    static Hardware hw;
    public BNO055IMU imu;
    public BNO055IMU imu2;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public double angle;
    public double angleOdo;
    public double previousAngleOdoReading;
    public List<LynxModule> allHubs;
    private double centerWheelOffset = 4.91142;

    private double centerWheelOffsetChange = 1.5;
    public static double trackWidth = 16.037;
    private static double portOffset = 7.674; // inches
    private static double starboardOffset = 8.103; //inches
    public static double odoWidth =portOffset+starboardOffset;
    public double xPosTicks = 0;
    public double yPosTicks = 0;
    public double xPosInches = 0;
    public double yPosInches = 0;
    public double yPosTicksAlt=0;
    public double xPosTicksAlt=0;
    public double xPosTicksAltAlt = 0;
    public double yPosTicksAltAlt = 0;
    private int previousPortReading;
    private int previousStarboardReading;
    private int previousLateralReading;
    private double previousAngleReading;
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
    public ContRotServo[] CRservos;
    public MecanumDrive mecanumDrive;
    public SixWheelDrive sixWheelDrive;
    public ElapsedTime time;
    public int loops = 0;
    public boolean firstLoop = true;
    public double startTime;
    public double prevTime;
    String TAG = "odo";
    public boolean updatePID;
    public double prevLocalY;
    public double banglePrev=0;
    public double canglePrev=0;
    public double danglePrev=0;
    public double localY;
    public double localX;
    public double localXToReportToRamseteDesiredKinematics;
    public double integratedAngularVeloTracker;
    public double deltaTime;
    public boolean currentlyForwardDirection = true;
    public double batteryVoltage;
    public static Telemetry telemetry;
    public double angle1 = 0;
    public double angle2 = 0;
    public Shooter shooter;
    public Turret turret;
    public Intake intake;
    public Mag mag;
    public WobblerArm wobbler;
    public Hardware(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        hw = this;
        Hardware.telemetry = telemetry;
        batteryVoltage = 8.62;
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
        hub1Motors[0].motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hub1Motors[0].motor.setDirection(DcMotorEx.Direction.REVERSE);
        hub1Motors[1] = new Motor(hardwareMap.get(DcMotorEx.class,"LB"));
        hub1Motors[1].motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hub1Motors[1].motor.setDirection(DcMotorEx.Direction.FORWARD);
        hub1Motors[2] = new Motor(hardwareMap.get(DcMotorEx.class,"RF"));
        hub1Motors[2].motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hub1Motors[2].motor.setDirection(DcMotorEx.Direction.FORWARD);
        hub1Motors[3] = new Motor(hardwareMap.get(DcMotorEx.class,"RB"));
        hub1Motors[3].motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hub1Motors[3].motor.setDirection(DcMotorEx.Direction.REVERSE);
        time = new ElapsedTime();
        sixWheelDrive = new SixWheelDrive(hub1Motors[0],hub1Motors[1],hub1Motors[2],hub1Motors[3],time);
        hub2Motors = new Motor[4];//initialize here
        servos = new RegServo[12];//initialize here
        CRservos = new ContRotServo[12];

        setForward();
        CRservos[0] = new ContRotServo(hardwareMap.get(CRServo.class,"turretServo1"));
        CRservos[1] = new ContRotServo(hardwareMap.get(CRServo.class,"turretServo2"));
        hub2Motors[0] = new Motor(hardwareMap.get(DcMotorEx.class,"shooterMotor1"));
        hub2Motors[1] = new Motor(hardwareMap.get(DcMotorEx.class,"shooterMotor2"));
        hub2Motors[2] = new Motor(hardwareMap.get(DcMotorEx.class,"intakeMotor1"));
        hub2Motors[3] = new Motor(hardwareMap.get(DcMotorEx.class,"intakeMotor2"));
        servos[0] = new RegServo(hardwareMap.get(Servo.class,"shootAngleController"));
        servos[1] = new RegServo(hardwareMap.get(Servo.class,"intakeDropperGuard"));
        servos[2] = new RegServo(hardwareMap.get(Servo.class,"magServo"));
        servos[4] = new RegServo(hardwareMap.get(Servo.class,"wobblerClaw"));
        servos[5] = new RegServo(hardwareMap.get(Servo.class,"wobblerArm"));
        servos[6] = new RegServo(hardwareMap.get(Servo.class,"ringPusher"));
        servos[7] = new RegServo(hardwareMap.get(Servo.class, "magRotationServo"));
        //shooter = new Shooter(hub2Motors[0],hub2Motors[1],servos[0],this);
        //turret = new Turret(new ContRotServo[]{CRservos[0],CRservos[1]}, servos[7], hub2Motors[0], this);
        //intake = new Intake(hub2Motors[2],servos[1]);
        mag = new Mag(servos[2],servos[6]);
        //wobbler = new WobblerArm(servos[5],servos[4]);
    }
    /*public Hardware(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        hw = this;
        batteryVoltage = VelocityPIDDrivetrain.getBatteryVoltage()-2;
        updatePID = false;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //imu2 = hardwareMap.get(BNO055IMU.class,"imu2");
        //imu2.initialize(parameters);
        angle = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        previousAngleReading = angle;
        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        hub1Motors = new Motor[4];//initialize here
        hub1Motors[0] = new Motor( hardwareMap.get(DcMotorEx.class,"LF"));
        hub1Motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        hub1Motors[1] = new Motor(hardwareMap.get(DcMotorEx.class,"LB"));
        hub1Motors[1].motor.setDirection(DcMotorEx.Direction.REVERSE);
        hub1Motors[2] = new Motor(hardwareMap.get(DcMotorEx.class,"RF"));
        hub1Motors[2].motor.setDirection(DcMotorEx.Direction.REVERSE);
        hub1Motors[3] = new Motor(hardwareMap.get(DcMotorEx.class,"RB"));
        hub1Motors[3].motor.setDirection(DcMotorEx.Direction.FORWARD);
        time = new ElapsedTime();
        sixWheelDrive = new SixWheelDrive(hub1Motors[0],hub1Motors[1],hub1Motors[2],hub1Motors[3],time);
        hub2Motors = new Motor[4];//initialize here
        servos = new RegServo[12];//initialize here
        setForward();
    }*/
    public void setForward(){
        currentlyForwardDirection = true;
    }
    public void setBackwards(){
        currentlyForwardDirection = false;
    }
    public void loop(){

        loops++;
        double deltaAngle=0;
        double deltaAngleOdo=0;
        if(ticker % GYRO_UPDATE_FREQ == 0) {
            double bangle = MathFunctions.keepAngleWithin180Degrees(Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
            double dangle = MathFunctions.keepAngleWithin180Degrees(Math.toRadians(imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
            double deltaAngle1 = 360/359.7*(360/357.0446428571429)* MathFunctions.keepAngleWithin180Degrees(bangle - banglePrev);
            double deltaAngle2 = 360/359.7*(360/359.9)*(360/358.872)*(360/359.7276785714286)* MathFunctions.keepAngleWithin180Degrees(dangle - danglePrev);
            angle1 += deltaAngle1;
            angle2 += deltaAngle2;
            banglePrev = bangle;
            danglePrev = dangle;
            angle = (deltaAngle2+deltaAngle1)/2 + canglePrev;
            deltaAngle = angle-previousAngleReading;
            canglePrev = angle;
            ticker = 0;
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
        if(hub1ReadNeeded) {
            allHubs.get(0).clearBulkCache();
        }
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
        if(hub2ReadNeeded) {
            allHubs.get(1).clearBulkCache();
        }
        for(int i = 0; i <hub2Motors.length;i++){
            Motor motor = hub2Motors[i];
            if(motor != null && motor.readRequested){
                motor.currentPosition = motor.motor.getCurrentPosition();
                motor.currentVelocity = motor.motor.getVelocity(AngleUnit.RADIANS);
            }
        }

            int portReading=-hub1Motors[0].getCurrentPosition();
            int starboardReading=-hub1Motors[3].getCurrentPosition();
            int lateralReading=hub1Motors[1].getCurrentPosition();
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
        }
        double currentTime = time.milliseconds();
        deltaTime = currentTime-prevTime;
       int PortChange = portReading - previousPortReading;
            int StarboardChange = starboardReading - previousStarboardReading;
        if(ticker%8!=GYRO_UPDATE_FREQ) {
            deltaAngleOdo  =  360/371.952509091*360/355.696047619*360/362.400832143*360.0/361*360/355.123*360/362.5*360/360.74258*(StarboardChange - PortChange) / (odoWidth * ticks_per_rotation / circumfrence);
            angle += deltaAngleOdo;
            deltaAngle = deltaAngleOdo;
            ticker++;
        }
            double deltaAngleBodo  =  360/371.952509091*360/355.696047619*360/362.400832143*360.0/361*360/355.123*360/362.5*360/360.74258*(StarboardChange - PortChange) / (odoWidth * ticks_per_rotation / circumfrence);
            angleOdo +=deltaAngleBodo;
            double localXAlt = lateralReading - previousLateralReading;//-(deltaAngle * centerWheelOffset * (ticks_per_rotation/circumfrence));
             localX =  lateralReading - previousLateralReading-(deltaAngle * centerWheelOffset * (ticks_per_rotation/circumfrence));
            localXToReportToRamseteDesiredKinematics = lateralReading - previousLateralReading-(deltaAngle * (centerWheelOffset-centerWheelOffsetChange) * (ticks_per_rotation/circumfrence));
        /*double localY = (PortChange + (deltaAngle * portOffset * (ticks_per_rotation/circumfrence)) + StarboardChange - (deltaAngle * starboardOffset * (ticks_per_rotation/circumfrence)))/2.0;
 */         double localYAlt = PortChange; //
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
                yPosTicks += (localY) * (Math.cos(previousAngleReading)- Math.cos(previousAngleReading + deltaAngle))/(deltaAngle) - (localX) * (Math.sin(previousAngleReading + deltaAngle)- Math.sin(previousAngleReading))/(deltaAngle);//updates Y position
                xPosTicks += (localY) * (Math.sin(previousAngleReading + deltaAngle)- Math.sin(previousAngleReading))/(deltaAngle) + (localX) * (Math.cos(previousAngleReading)- Math.cos(previousAngleReading + deltaAngle))/(deltaAngle);//updates X position
                yPosTicksAlt += (localYAlt) * Math.sin(previousAngleReading + deltaAngle * 0.5) - (localXAlt) * Math.cos(previousAngleReading + deltaAngle * 0.5);//updates Y position
                xPosTicksAlt += (localYAlt) * Math.cos(previousAngleReading + deltaAngle * 0.5) + (localXAlt) * Math.sin(previousAngleReading + deltaAngle * 0.5);//updates X position
                double[] AltAltNewPosition = MathFunctions.transposeCoordinate(xPosTicksAltAlt,yPosTicksAltAlt,localY,-localX,previousAngleReading+deltaAngle*0.5);
                xPosTicksAltAlt = AltAltNewPosition[0];
                yPosTicksAltAlt = AltAltNewPosition[1];
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
            double w = deltaAngleBodo/((currentTime-prevTime)/1000);//inches
        integratedAngularVeloTracker+=w*(currentTime-prevTime)/1000;
        //double localYVelocity = (portVelo + (w*portOffset) + starboardVelo - (w * starboardOffset))/2.0;

         localYVelocity = localY*circumfrence/ticks_per_rotation/((currentTime - prevTime)/1000);
        if(updatePID) {
            sixWheelDrive.updatePID(localYVelocity - w * trackWidth / 2, localYVelocity + w * trackWidth / 2);
        }
        if(shooter.updatePID) {
            shooter.updateShooterPIDF(deltaTime / 1000);
        }
        if(turret.updatePID){
            turret.updateTurretPID();
        }
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
        for(ContRotServo CRservo: CRservos){
            if(CRservo!=null&&CRservo.writeRequested) {
                CRservo.servo.setPower(CRservo.power);
                CRservo.writeRequested = false;
            }
        }
        RobotLog.dd("MOTORDEBUG", "Left: "+sixWheelDrive.LF.power+ ", Right: "+sixWheelDrive.RF.power);
            xPosTicksClassVariable = xPosTicks;
            yPosTicksClassVariable = yPosTicks;
            angleClassVariable = angle;
        previousLateralReading = lateralReading;
            previousAngleReading = angle;
            previousPortReading = portReading;
            previousStarboardReading = starboardReading;
            prevTime = currentTime;
            prevLocalY = localYVelocity;
            previousAngleOdoReading = angleOdo;
    }
    public double getX(){
        double xPosInches = (double)xPosTicks * circumfrence / ticks_per_rotation;
        double yPosInches = (double)yPosTicks * circumfrence / ticks_per_rotation;
        if(currentlyForwardDirection) {
            this.xPosInches = MathFunctions.transposeCoordinate(xPosInches, yPosInches, centerWheelOffsetChange, angle)[0];
        }
        else{
            this.xPosInches = MathFunctions.transposeCoordinate(xPosInches, yPosInches, -centerWheelOffsetChange, angle)[0];
        }
        return this.xPosInches;
    }
    public double getY(){
        double xPosInches = (double)xPosTicks * circumfrence / ticks_per_rotation;
        double yPosInches = (double)yPosTicks * circumfrence / ticks_per_rotation;
        if(currentlyForwardDirection) {
            this.yPosInches = MathFunctions.transposeCoordinate(xPosInches, yPosInches, centerWheelOffsetChange, angle)[1];
        }
        else{
            this.yPosInches = MathFunctions.transposeCoordinate(xPosInches, yPosInches, -centerWheelOffsetChange, angle)[1];
        }
        return this.yPosInches;
    }
    public double getXAbsoluteCenter(){
        return (double)xPosTicks * circumfrence / ticks_per_rotation;
    }
    public double getYAbsoluteCenter(){
        return (double)yPosTicks * circumfrence / ticks_per_rotation;
    }
    public double getXMeters(){
        return getX() /39.3701;
    }
    public double getYMeters(){
        return getY()/39.3701;
    }
    public static Hardware getInstance(){
        return hw;
    }
}
