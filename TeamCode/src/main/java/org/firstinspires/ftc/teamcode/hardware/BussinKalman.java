package org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BussinKalman {
        protected double x;
        protected double p;
        protected double x_previous;
        protected double p_previous;
        protected double currentOdomAngle;
        protected double previousOdomAngle;
        protected double EPSILON = 0.000001;

        /**
         * kalman parameters
         * <p>
         * Q
         */
        protected double Q = 0.41504936835155863;
        protected double R = .904936835155863;

        protected double A = 1;
        protected double C = 1;
        protected double H = 1;
        protected double I = 1;
        protected double kalmanGain;


        public BussinKalman(double initialCondition) {
            this.x = initialCondition;
            this.x_previous = initialCondition;
            this.p = 0;
            this.p_previous = 0;
            this.currentOdomAngle = initialCondition;
            this.previousOdomAngle = initialCondition;
        }

        /**
         * update the odometry measurement from our sensor1 and gyro angle
         *
         * @param sensor1 primary angle sensor
         * @param sensor2 most stable angle sensor that we are converging on over time
         * @return estimated angle from kalman filter
         */
        public double updateKalmanEstimate(double sensor1, double sensor2) {

            //calculateCovariances(sensor1,sensor2);

            currentOdomAngle = sensor1;

            double u = currentOdomAngle - previousOdomAngle;

            x = x_previous + u;
            p = p_previous + Q;

            kalmanGain = p * H * (H * p * H + R);

            x = AngleUnit.normalizeRadians(x + kalmanGain * AngleUnit.normalizeRadians(sensor2 - H * x));

            x_previous = x;

            previousOdomAngle = currentOdomAngle;

            p = (I - kalmanGain * H) * p;

            p_previous = p;

            return x;
        }


        /**
         * compute the deviation for each sensor, this is the corresponding Q and R values
         * @param sensor1 first sensor
         * @param sensor2 second sensor
         */
        private void calculateCovariances(double sensor1, double sensor2) {

            double mean = (sensor1 + sensor2) / 2;
            double deviation1 = Math.pow(sensor1 - mean,2);
            double deviation2 = Math.pow(sensor2 - mean,2);
            deviation1 = Math.sqrt(deviation1);
            deviation2 = Math.sqrt(deviation2);

            Q = validateDeviation(deviation1);

            R = validateDeviation(deviation2);

        }

        /**
         * ensure that our deviations are valid and that it wont cause the robot to literally die and spin in circles
         * @param deviation the deviation we are assessing
         * @return the (hopefully safe) deviation
         */
        public double validateDeviation(double deviation) {

            // scary things happen if any of these are true so we just make it less scary!

            // one of these three conditions occurs occasionally but the robot dies so quickly that I cannot debug it
            // as a result i just check all three values each time.  In efficient? yes. Safer? I sure hope so lmfao
            if (deviation == 0 || Double.isNaN(deviation) || Double.isNaN(deviation)) {
                return EPSILON;
            }


            return deviation;
        }

}
