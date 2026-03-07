package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;

public class TrajectorySolver {
    public static class TrajectoryParameters {
        public double velocity;
        public double theta_yaw;
        public double theta_pitch;

        public double time;
    }

    public static class TrajectoryConditions {
        // required
        public Pose3d start;
        public Pose3d target;

        // control variables, only one needed
        public double theta;
        public double velocity;
    }

    public enum SolveType {
        CONTROL_THETA,
        CONTROL_VELOCITY
    }

    /**
     * Calculate the launch parameters of a trajectory that will hit the target.
     * 
     * @param conditions Conditions that the trajectory must meet.
     * @param type Which control variable to use from conditions, theta or velocity.
     * while firing from the specified conditions.
     */
    public static TrajectoryParameters solveTrajectory(TrajectoryConditions conditions, SolveType type) {
        double dx = conditions.start.toPose2d().getTranslation()
                .getDistance(conditions.target.toPose2d().getTranslation());
        double dy = conditions.target.getZ() - conditions.start.getZ();

        double theta_yaw = conditions.start.toPose2d().getTranslation()
                .minus(conditions.target.toPose2d().getTranslation())
                .getAngle().getRadians();

        switch (type) {
            case CONTROL_THETA: {
                /*
                 * Ready for some math?

                 * 	vx = vcos(theta)
                 *  vy = vsin(theta)
                 *  v = vx/cos(theta)
                 *  v = vy/sin(theta)

                 *  dx = vxt
                 *  vx = dx/t

                 *  dy = vyt + 1/2att
                 *  vyt = dy - att/2Math.sqrt((2*dy*Math.cos(th) - 2*dx*Math.sin(th))/(-9.8*Math.cos(th)));
                 *  vy = (dy - att/2)/t

                 *  v = (dx)/tcos(theta)
                 *  v = (dy - att/2)/tsin(theta)

                 *  dx/tcos(theta) = (dy - 1/2att)/tsin(theta)
                 *  (dy - att/2)tcos(theta) = dxtsin(theta)
                 *  (dy - att/2)cos(theta) = dxsin(theta)
                 *  dycos(theta) - attcos(theta)/2 = dxsin(theta)
                 *  attcos(theta)/2 = dycos(theta) - dxsin(theta)
                 *  attcos(theta) = 2dycos(theta) - 2dxsin(theta)
                 *  t = sqrt((2dycos(theta) - 2dxsin(theta))/(acos(theta)))

                 *  vx = dx/t
                 *  v = vx/cos(theta)
                 *  v = dx/tcos(theta)
                 *  v = dx/sqrt((2dycos(theta) - 2dxsin(theta))/(acos(theta)))cos(theta)
                 */
                double th = conditions.theta;

                double t = Math.sqrt((2*dy*Math.cos(th) - 2*dx*Math.sin(th))/(-9.8*Math.cos(th)));Math.sqrt((2*dy*Math.cos(th) - 2*dx*Math.sin(th))/(-9.8*Math.cos(th)));
                double vx = dx/t;
                double v = vx/Math.cos(th);

                TrajectoryParameters params = new TrajectoryParameters();
                params.velocity = v;
                params.theta_pitch = th;
                params.theta_yaw = theta_yaw;
                params.time = t;

                return params;
            } case CONTROL_VELOCITY: {
                // vx = vcos(theta)
                // theta = acos(vx/v)
                /*double theta_pitch = Math.acos(vx/conditions.velocity);

                TrajectoryParameters parameters = new TrajectoryParameters();
                parameters.velocity = conditions.velocity;
                parameters.theta_yaw = theta_yaw;
                parameters.theta_pitch = theta_pitch;
                return parameters;*/
                return null;
            } default: {
                return null;
            }
        }
    }    
}


/*

	CONTROLS:
	ax,ay
	bx,by
	cx,cy

	ay = a(ax)^2 + b(ax) + c
	by = a(bx)^2 + b(bx) + c
	cy = a(cx)^2 + b(cx) + c

	0 = a(ax)^2 + b(ax) + c - ay
	0 = a(bx)^2 + b(bx) + c - by
	0 = a(cx)^2 + b(cx) + c - cy

	a(ax)^2 + b(ax) + c - ay = a(bx)^2 + b(bx) + c - by = a(cx)^2 + b(cx) + c - cy



 */