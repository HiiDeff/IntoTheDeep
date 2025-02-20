package org.firstinspires.ftc.teamcode.util.limelight;


public class LimelightConfig {
    // all measurements are in inches and degrees
    public int resolutionX, resolutionY;
    public double horizontalFOV, verticalFOV;
    public double angleWithHorizontal;
    public double xOffset, yOffset, zOffset;
    // xOffset: right is positive, left is negative
    // yOffset: forwards is positive, backwards is negative
    // zOffset: up is positive, down is negative

    public LimelightConfig(int resolutionX, int resolutionY,
                           double angleWithHorizontalDegrees,
                           double horizontalFOV, double verticalFOV,
                           double xOffsetInch, double yOffsetInch, double zOffsetInch) {
        this.resolutionX = resolutionX;
        this.resolutionY = resolutionY;
        this.horizontalFOV = horizontalFOV;
        this.verticalFOV = verticalFOV;
        this.angleWithHorizontal = angleWithHorizontalDegrees;
        this.xOffset = xOffsetInch;
        this.yOffset = yOffsetInch;
        this.zOffset = zOffsetInch;
    }
}