package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@Config
public class IntakeSampleState {

    public static double MIN_DIST_WITH_SAMPLE = 1.2;
    private V2RobotBase robot;
    private PixelColor[] pixelColors = {PixelColor.COLOR_UNSPECIFIED, PixelColor.COLOR_UNSPECIFIED};
    private int numOfPixels = 0;
    private boolean leftSensorDetected;

    private void setStripLightColors() {
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

        if(pixelColors[0] != PixelColor.COLOR_UNSPECIFIED && pixelColors[1] != PixelColor.COLOR_UNSPECIFIED && pixelColors[0] != pixelColors[1]){
            pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        }
        else if (numOfPixels > 0) {
            int idx = leftSensorDetected? 0:1;
            switch (pixelColors[idx]) {
                case COLOR_YELLOW:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    break;
                case COLOR_BLUE:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                    break;
                case COLOR_RED:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    break;
                case COLOR_UNSPECIFIED:
                default:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                    break;
            }
        }
        robot.setLightColor(pattern);
    }
    public String getStripLightColors() {
        return pixelColors[0].name()+" "+pixelColors[1].name();
    }

    private boolean colorIsNotSet(PixelColor color) {
        return color == PixelColor.COLOR_UNSPECIFIED;
    }

    public IntakeSampleState(V2RobotBase robot) {
        this.robot = robot;
    }

    public void update() {
        numOfPixels = 0;
        double leftDist = robot.leftIntakeColorSensor.getDistance(DistanceUnit.INCH);
        double rightDist = robot.rightIntakeColorSensor.getDistance(DistanceUnit.INCH);
        pixelColors[0] = PixelColor.COLOR_UNSPECIFIED;
        pixelColors[1] = PixelColor.COLOR_UNSPECIFIED;
        leftSensorDetected = false;
        if (leftDist < MIN_DIST_WITH_SAMPLE) {
            numOfPixels++;
            pixelColors[0] = toHDColor(robot.leftIntakeColorSensor.getNormalizedColors());
            leftSensorDetected = true;
        }
        if (rightDist < MIN_DIST_WITH_SAMPLE) {
            numOfPixels++;
            pixelColors[1] = toHDColor(robot.rightIntakeColorSensor.getNormalizedColors());
        }
        setStripLightColors();
    }

    public int getNumOfPixels() {
        return numOfPixels;
    }

    public PixelColor getPixelColor(int index) {
        if (index < 0 || index > 1) return PixelColor.COLOR_UNSPECIFIED;
        return pixelColors[index];
    }

    private PixelColor toHDColor(NormalizedRGBA color) {
        HSV hsv = ImageProcessor.ColorToHsv(color);
        if (ImageProcessor.isYellow(hsv)) {
            return IntakeSampleState.PixelColor.COLOR_YELLOW;
        }
        if (ImageProcessor.isRed(hsv)) {
            return IntakeSampleState.PixelColor.COLOR_RED;
        }
        if (ImageProcessor.isBlue(hsv)) {
            return IntakeSampleState.PixelColor.COLOR_BLUE;
        }
        return IntakeSampleState.PixelColor.COLOR_UNSPECIFIED;
    }

    public enum PixelColor {
        COLOR_UNSPECIFIED,
        COLOR_YELLOW,
        COLOR_BLUE,
        COLOR_RED
    }
}
