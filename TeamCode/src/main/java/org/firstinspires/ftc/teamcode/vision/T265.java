package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.MathFunctions;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class T265 extends OpMode
{
        // We treat this like a singleton because there should only ever be one object per camera
        public static T265Camera slamra = null;
        public static double localXOffsetCameraToCenter=7.00591;//-0.6;//+3.68125;
        public static double localYOffsetCameraToCenter=-1.5427;//-0.5;//-3.3599;
        public static double dilationFactor =1;//1.06;//1.065;
        public static Rotation2d rotation2d = new Rotation2d();
        public static double ODOMETRY_COVARIANCE=0.5;
        @Override
        public void init() {
                if (slamra == null) {

                        T265.slamra = new T265Camera(new Transform2d(), T265.ODOMETRY_COVARIANCE, hardwareMap.appContext);                }
        }

        @Override
        public void init_loop() {
        }

        @Override
        public void start() {
                slamra.start();
        }

        public static double[] getCameraPosition(T265Camera.CameraUpdate up){
                double xAtCamera = -up.pose.getTranslation().getX() / 0.0254/dilationFactor;
                double yAtCamera = -up.pose.getTranslation().getY() / 0.0254/dilationFactor;
                double heading = up.pose.getHeading();
                double[] centerPosition = MathFunctions.transposeCoordinate(xAtCamera,yAtCamera,localXOffsetCameraToCenter,localYOffsetCameraToCenter,heading);
                centerPosition[0] -= localXOffsetCameraToCenter;
                centerPosition[1] -= localYOffsetCameraToCenter;
                for(int i = 0; i < 2;i++){
                        centerPosition[i] = centerPosition[i]/=dilationFactor;
                }
                return new double[]{centerPosition[0], centerPosition[1]};
        }
        @Override
        public void loop() {
                final int robotRadius = 9; // inches


                T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
                if (up == null) return;

                // We divide by 0.0254 to convert meters to inches
                Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
                Rotation2d rotation = up.pose.getRotation();

                double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
                double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
                double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
                telemetry.addLine("x: "+up.pose.getTranslation().getX()/0.0254+", y: "+up.pose.getTranslation().getY()/0.0254 +", heading: "+ Math.toDegrees(up.pose.getHeading()));
                telemetry.update();
        }

        @Override
        public void stop() {
                slamra.stop();
        }

}