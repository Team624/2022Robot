package frc.robot.utility;

public class Gains {

    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public final double iZone;

    public Gains(double p, double i, double d, double f, double zone) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        iZone = i;
    }

}