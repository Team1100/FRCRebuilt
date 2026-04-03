package frc.robot.utils.trajectory;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class VelocityMapping implements Sendable {
    public double velocity1;
    public double velocity2;
    public double rpm1;
    public double rpm2;

    public VelocityMapping(double velocity1, double velocity2, double rpm1, double rpm2) {
        this.velocity1 = velocity1;
        this.velocity2 = velocity2;
        this.rpm1 = rpm1;
        this.rpm2 = rpm2;
    }

    public VelocityMapping() {
        this.velocity1 = 0;
        this.velocity2 = 0;
        this.rpm1 = 0;
        this.rpm2 = 0;
    }

    public double getVelocity(double rpm) {
        double t = (rpm - rpm1)/(rpm2 - rpm1);
        return velocity1 + (velocity2 - velocity1) * t;
    }

    public double getRPM(double velocity) {
        double t = (velocity - velocity1)/(velocity2 - velocity1);
        return rpm1 + (rpm2 - rpm1) * t;
    }

    public static VelocityMapping interpolate(VelocityMapping a, VelocityMapping b, double t) {
        double blendedVelocity1 = a.velocity1 + (b.velocity1 - a.velocity1) * t;
        double blendedVelocity2 = a.velocity2 + (b.velocity2 - a.velocity2) * t;
        double blendedRPM1 = a.rpm1 + (b.rpm1 - a.rpm1) * t;
        double blendedRPM2 = a.rpm2 + (b.rpm2 - a.rpm2) * t;

        return new VelocityMapping(
            blendedVelocity1, blendedVelocity2,
            blendedRPM1, blendedRPM2
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("VelocityMapping");
        builder.addDoubleProperty("Velocity 1", ()->this.velocity1, (double a)->{this.velocity1 = a;});
        builder.addDoubleProperty("Velocity 2", ()->this.velocity2, (double a)->{this.velocity2 = a;});
        builder.addDoubleProperty("RPM 1", ()->this.rpm1, (double a)->{this.rpm1 = a;});
        builder.addDoubleProperty("RPM 2", ()->this.rpm2, (double a)->{this.rpm2 = a;});
    }
}