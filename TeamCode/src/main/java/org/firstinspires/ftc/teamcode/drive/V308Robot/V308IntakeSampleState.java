package org.firstinspires.ftc.teamcode.drive.V308Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@Config
public class V308IntakeSampleState {

    public static double MIN_DIST_WITH_SAMPLE = 1.2;
    private V308Robot robot;
    private SampleColor[] sampleColors = {SampleColor.COLOR_UNSPECIFIED, SampleColor.COLOR_UNSPECIFIED};
    private int numOfSamples = 0;
    private boolean leftSensorDetected;

    private void setStripLightColors() {
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

        if(sampleColors[0] != SampleColor.COLOR_UNSPECIFIED && sampleColors[1] != SampleColor.COLOR_UNSPECIFIED && sampleColors[0] != sampleColors[1]){
            pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
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
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
//                    pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                    break;
            }
        }
        robot.setLightColor(pattern);
    }
    public synchronized String getIntakeColors() {
        return sampleColors[0].name()+" "+sampleColors[1].name();
    }

    private boolean colorIsNotSet(SampleColor color) {
        return color == SampleColor.COLOR_UNSPECIFIED;
    }

    public V308IntakeSampleState(V308Robot robot) {
        this.robot = robot;
    }

    public synchronized boolean IsSampleIn() {
//        double leftDist = robot.leftIntakeColorSensor.getDistance(DistanceUnit.INCH);
//        double rightDist = robot.rightIntakeColorSensor.getDistance(DistanceUnit.INCH);

//        return leftDist < MIN_DIST_WITH_SAMPLE || rightDist < MIN_DIST_WITH_SAMPLE;
        return numOfSamples > 0;
    }

    public void update() {

        int tempNumOfSamples = 0;
        SampleColor[] tempSampleColors = {SampleColor.COLOR_UNSPECIFIED, SampleColor.COLOR_UNSPECIFIED};
        boolean tempLeftSensorDetected = false;

        double leftDist = robot.leftIntakeColorSensor.getDistance(DistanceUnit.INCH);
        if(leftDist < MIN_DIST_WITH_SAMPLE){
            tempNumOfSamples++;
            tempSampleColors[0] = toHDColor(robot.leftIntakeColorSensor.getNormalizedColors());
            tempLeftSensorDetected = true;
        }
        double rightDist = robot.rightIntakeColorSensor.getDistance(DistanceUnit.INCH);
        if (rightDist < MIN_DIST_WITH_SAMPLE) {
            tempNumOfSamples++;
            tempSampleColors[1] = toHDColor(robot.rightIntakeColorSensor.getNormalizedColors());
        }

//        sampleColors[0] = SampleColor.COLOR_UNSPECIFIED;
//        sampleColors[1] = SampleColor.COLOR_UNSPECIFIED;
//        leftSensorDetected = false;
//        if (leftDist < MIN_DIST_WITH_SAMPLE) {
//            numOfSamples++;
//            sampleColors[0] = toHDColor(robot.leftIntakeColorSensor.getNormalizedColors());
//            leftSensorDetected = true;
//        }
//        if (rightDist < MIN_DIST_WITH_SAMPLE) {
//            numOfSamples++;
//            sampleColors[1] = toHDColor(robot.rightIntakeColorSensor.getNormalizedColors());
//        }
        updateInternal(tempNumOfSamples, tempSampleColors, tempLeftSensorDetected);
    }
    private synchronized void updateInternal(int tempNumOfSamples, SampleColor[] tempSampleColors, boolean tempLeftSensorDetected) {
        numOfSamples = tempNumOfSamples;
        sampleColors[0] = tempSampleColors[0];
        sampleColors[1] = tempSampleColors[1];
        leftSensorDetected = tempLeftSensorDetected;
        setStripLightColors();
    }

    public synchronized int getNumOfSamples() {
        return numOfSamples;
    }

    public synchronized SampleColor getSampleColor(int index) {
        if (index < 0 || index > 1) return SampleColor.COLOR_UNSPECIFIED;
        return sampleColors[index];
    }

    private SampleColor toHDColor(NormalizedRGBA color) {
        HSV hsv = ImageProcessor.ColorToHsv(color);
        if (ImageProcessor.isYellow(hsv)) {
            return V308IntakeSampleState.SampleColor.COLOR_YELLOW;
        }
        if (ImageProcessor.isRed(hsv)) {
            return V308IntakeSampleState.SampleColor.COLOR_RED;
        }
        if (ImageProcessor.isBlue(hsv)) {
            return V308IntakeSampleState.SampleColor.COLOR_BLUE;
        }
        return V308IntakeSampleState.SampleColor.COLOR_UNSPECIFIED;
    }

    public enum SampleColor {
        COLOR_UNSPECIFIED,
        COLOR_YELLOW,
        COLOR_BLUE,
        COLOR_RED
    }
}
