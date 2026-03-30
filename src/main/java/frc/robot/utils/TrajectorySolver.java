package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class TrajectorySolver {
    public static class TrajectoryParameters {
        public double velocity;
        public double theta_yaw;
        public double theta_pitch;
        public double time;
    }

    public static class TrajectoryConditions {
        public Translation3d launch;
        public Translation3d target;
        public Translation2d chassis_velocity;
        public double theta_pitch;
    }

    // constants and conveniences
    private static final double GRAVITY = 9.81;
    private static final double VELOCITY_THRESHOLD = 1e-2;
    private static final Translation3d UP = new Translation3d(0,0,1);
    private static final Translation3d ZERO = Translation3d.kZero;

    /**
     * Calculate the launch parameters of a trajectory that will hit the target.
     * 
     * @param conditions Conditions that the trajectory must meet.
     * @param type Which control variable to use from conditions, theta or velocity.
     * while firing from the specified conditions.
     */
    public static TrajectoryParameters solveTrajectory(TrajectoryConditions conditions) {
        /*
         * praise be to Game Dev Stack Exchange (and @DMGregory)!
         * i spent hours doing the algebra for this and it worked but came out unreadable;
         * this is a better version that labels individual steps of the process.
         * 
         * https://gamedev.stackexchange.com/questions/174261/how-do-i-launch-a-ballistic-projectile-to-hit-a-moving-target-given-launch-pos
         * 
         * variable names here match the names of variables in the answer, so
         * you can follow along there for an explanation.
         */

        // defining our variables
        double t = conditions.theta_pitch;
        Translation3d r = conditions.target.minus(conditions.launch);
        Translation3d v = new Translation3d(conditions.chassis_velocity.unaryMinus());

        double r_v = r.dot(UP);
        Translation2d r_h = r.toTranslation2d();
        
        double v_v = v.dot(UP);
        Translation2d v_h = v.toTranslation2d();

        // quartic terms
        double tan2 = Math.pow(Math.tan(t),2);

        double a = (GRAVITY * GRAVITY)/4.0;
        double b = v_v*GRAVITY;
        double c = r_v*GRAVITY + v_v*v_v - tan2*v_h.dot(v_h);
        double d = 2*(r_v*v_v - tan2*r_h.dot(v_h));
        double e = r_v*r_v - tan2*r_h.dot(r_h);

        // solve for maximum time (highest arc)
        double time = 0;
        if (v.getDistance(ZERO) > VELOCITY_THRESHOLD) {
            double[] roots = solveRealQuarticRoots(a, b, c, d, e);
            time = -Double.MAX_VALUE;
            for (int i = 0; i < roots.length; i++) {
                System.out.println(time);
                if (roots[i] > time) time = roots[i];
            }
        } else {
            // when velocity is 0, b and d terms disappear; we can still
            // solve for t in this case by solving for t^2 via the quadratic
            // 0 = at^4 + ct^2 + e
            double time2_max = (-c + Math.sqrt(c*c - 4*a*e))/(2*a);
            time = Math.sqrt(time2_max);
        }

        // plug time into the launch velocity equations
        Translation2d l_h = r_h.plus(v_h).div(time);
        double l_hm = Math.hypot(l_h.getX(), l_h.getY());
        double l_v = Math.tan(t) * l_hm;

        Translation3d l = new Translation3d(l_h.getX(), l_h.getY(), l_v);

        // finally
        TrajectoryParameters params = new TrajectoryParameters();
        params.velocity = l.getDistance(ZERO);
        params.theta_yaw = l_h.getAngle().getRadians();
        params.theta_pitch = t;
        params.time = time;

        return params;
    }

    public static double[] solveRealQuarticRoots(double a, double b, double c, double d, double e) {
        double s1 = 2 * c * c * c - 9 * b * c * d + 27 * (a * d * d + b * b * e) - 72 * a * c * e,
            q1 = c * c - 3 * b * d + 12 * a * e,
            s2 = s1 + Math.sqrt(-4 * q1 * q1 * q1 + s1 * s1),
            q2 = Math.cbrt(s2 / 2),
            s3 = q1 / (3 * a * q2) + q2 / (3 * a),
            s4 = Math.sqrt((b * b) / (4 * a * a) - (2 * c) / (3 * a) + s3),
            s5 = (b * b) / (2 * a * a) - (4 * c) / (3 * a) - s3,
            s6 = (-(b * b * b) / (a * a * a) + (4 * b * c) / (a * a) - (8 * d) / a) / (4 * s4);

        double[] roots = new double[4];
        for (int i = 0; i < 3; i++)
            roots[i] = -b / (4 * a) + (i > 1 ? -1 : 1) * (s4 / 2) - (i % 2 == 0 ? -1 : 1) * (Math.sqrt(s5 + (i > 1 ? -1 : 1) * s6) / 2);

        return roots;
    }
}