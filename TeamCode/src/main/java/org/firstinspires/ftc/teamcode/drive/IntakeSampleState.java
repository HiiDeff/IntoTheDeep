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
    private RobotBase robot;
    private SampleColor[] sampleColors = {SampleColor.COLOR_UNSPECIFIED, SampleColor.COLOR_UNSPECIFIED};
    private int numOfSamples = 0;
    private boolean leftSensorDetected;

    private void setStripLightColors() {
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

        if(sampleColors[0] != SampleColor.COLOR_UNSPECIFIED && sampleColors[1] != SampleColor.COLOR_UNSPECIFIED && sampleColors[0] != sampleColors[1]){
            pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        }
        else if (numOfSamples > 0) {
            int idx = leftSensorDetected? 0:1;
            switch (sampleColors[idx]) {
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
        return sampleColors[0].name()+" "+sampleColors[1].name();
    }

    private boolean colorIsNotSet(SampleColor color) {
        return color == SampleColor.COLOR_UNSPECIFIED;
    }

    public IntakeSampleState(RobotBase robot) {
        this.robot = robot;
    }

    public boolean IsSampleIn()
    {
//        double leftDist = robot.leftIntakeColorSensor.getDistance(DistanceUnit.INCH);
//        double rightDist = robot.rightIntakeColorSensor.getDistance(DistanceUnit.INCH);

//        return leftDist < MIN_DIST_WITH_SAMPLE || rightDist < MIN_DIST_WITH_SAMPLE;
        return numOfSamples > 0;
    }

    public void update() {
        numOfSamples = 0;
        double leftDist = robot.leftIntakeColorSensor.getDistance(DistanceUnit.INCH);
        double rightDist = robot.rightIntakeColorSensor.getDistance(DistanceUnit.INCH);
        sampleColors[0] = SampleColor.COLOR_UNSPECIFIED;
        sampleColors[1] = SampleColor.COLOR_UNSPECIFIED;
        leftSensorDetected = false;
        if (leftDist < MIN_DIST_WITH_SAMPLE) {
            numOfSamples++;
            sampleColors[0] = toHDColor(robot.leftIntakeColorSensor.getNormalizedColors());
            leftSensorDetected = true;
        }
        if (rightDist < MIN_DIST_WITH_SAMPLE) {
            numOfSamples++;
            sampleColors[1] = toHDColor(robot.rightIntakeColorSensor.getNormalizedColors());
        }
        setStripLightColors();
    }

    public int getNumOfPixels() {
        return numOfSamples;
    }

    public SampleColor getPixelColor(int index) {
        if (index < 0 || index > 1) return SampleColor.COLOR_UNSPECIFIED;
        return sampleColors[index];
    }

    private SampleColor toHDColor(NormalizedRGBA color) {
        HSV hsv = ImageProcessor.ColorToHsv(color);
        if (ImageProcessor.isYellow(hsv)) {
            return IntakeSampleState.SampleColor.COLOR_YELLOW;
        }
        if (ImageProcessor.isRed(hsv)) {
            return IntakeSampleState.SampleColor.COLOR_RED;
        }
        if (ImageProcessor.isBlue(hsv)) {
            return IntakeSampleState.SampleColor.COLOR_BLUE;
        }
        return IntakeSampleState.SampleColor.COLOR_UNSPECIFIED;
    }

    public enum SampleColor {
        COLOR_UNSPECIFIED,
        COLOR_YELLOW,
        COLOR_BLUE,
        COLOR_RED
    }
}
