package frc.robot.utility;

public class PathPoint {
    private double x, y, vx, vy, heading, omega, vision;
    public PathPoint(double x, double y, double vx, double vy, double heading, double omega, double vision) {
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.heading = heading;
        this.omega = omega;
        this.vision = vision;
    }

    public void setX(double x){this.x = x;}
    public void setY(double y){this.y = y;}
    public void setVx(double vx){this.vx = vx;}
    public void setVy(double vy){this.vy = vy;}
    public void setHeading(double h){this.heading = h;}
    public void setOmega(double o){this.omega = o;}
    public void setVision(double v){this.vision = v;}

    public double getX() {return x;}
    public double getY() {return y;}
    public double getVx() {return vx;}
    public double getVy() {return vy;}
    public double getHeading() {return heading;}
    public double getOmega() {return omega;}
    public double getVision() {return vision;}

}