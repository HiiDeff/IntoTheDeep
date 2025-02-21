package org.firstinspires.ftc.teamcode.util.limelight;

public class Coordinates {

    public double x, y;
    public double angle;
    public Coordinates(double x, double y){
        this.x = x;
        this.y = y;
        this.angle = 0.0;
    }
    public Coordinates(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
}
