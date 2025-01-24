package org.firstinspires.ftc.teamcode.drive.V4Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@Config
public class V4IntakeSampleState {

    public static double MIN_DIST_WITH_SAMPLE = 1.2;
    private V4Robot robot;
    private SampleColor sampleColor = SampleColor.COLOR_UNSPECIFIED;
    private int numOfSamples = 0;

    private void setStripLightColors() {
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;

        if (numOfSamples > 0) {
            switch (sampleColor) {
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
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE;
                    break;
            }
        }
        robot.setLightColor(pattern);
    }
    public String getStripLightColors() {
        return sampleColor.name();
    }

    private boolean colorIsNotSet(SampleColor color) {
        return color == SampleColor.COLOR_UNSPECIFIED;
    }

    public V4IntakeSampleState(V4Robot robot) {
        this.robot = robot;
    }

    public boolean IsSampleIn() {
        return numOfSamples > 0;
    }

    public void update() {
        numOfSamples = 0;
        double dist = robot.intakeColorSensor.getDistance(DistanceUnit.INCH);
        sampleColor = SampleColor.COLOR_UNSPECIFIED;
        if (dist < MIN_DIST_WITH_SAMPLE) {
            numOfSamples = 1;
            sampleColor = toHDColor(robot.intakeColorSensor.getNormalizedColors());
        }
        setStripLightColors();
    }

    public int getNumOfSamples() {
        return numOfSamples;
    }

    public SampleColor getSampleColor() {
        return sampleColor;
    }

    private SampleColor toHDColor(NormalizedRGBA color) {
        HSV hsv = ImageProcessor.ColorToHsv(color);
        if (ImageProcessor.isYellow(hsv)) {
            return V4IntakeSampleState.SampleColor.COLOR_YELLOW;
        }
        if (ImageProcessor.isRed(hsv)) {
            return V4IntakeSampleState.SampleColor.COLOR_RED;
        }
        if (ImageProcessor.isBlue(hsv)) {
            return V4IntakeSampleState.SampleColor.COLOR_BLUE;
        }
        return V4IntakeSampleState.SampleColor.COLOR_UNSPECIFIED;
    }

    public enum SampleColor {
        COLOR_UNSPECIFIED,
        COLOR_YELLOW,
        COLOR_BLUE,
        COLOR_RED
    }
}
