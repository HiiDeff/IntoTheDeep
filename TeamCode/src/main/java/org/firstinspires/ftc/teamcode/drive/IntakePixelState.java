package org.firstinspires.ftc.teamcode.drive;//package org.firstinspires.ftc.teamcode.drive;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
//import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;
//
//@Config
//public class IntakePixelState {
//    public static double MIN_DIST_WITH_PIXEL = 0.5;
//
//    private WorldRobotBase robot;
//    private PixelColor[] pixelColors = {PixelColor.COLOR_UNSPECIFIED, PixelColor.COLOR_UNSPECIFIED};
//    private int numOfPixels = 0;
//
//    private void setStripLightColors() {
//        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
//
//        if (numOfPixels == 2) {
//            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
//        } else if (numOfPixels > 0) {
//            switch (pixelColors[numOfPixels - 1]) {
//                case COLOR_PURPLE:
//                    pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
//                    break;
//                case COLOR_GREEN:
//                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
//                    break;
//                case COLOR_YELLOW:
//                    pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
//                    break;
//                case COLOR_WHITE:
//                    pattern = RevBlinkinLedDriver.BlinkinPattern.GRAY;
//                    break;
//                case COLOR_WHITE_OR_PURPLE:
//                    pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
//                    break;
//                case COLOR_UNSPECIFIED:
//                default:
//                    pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
//                    break;
//            }
//        }
//        robot.setLeftLightColor(pattern);
//        robot.setRightLightColor(pattern);
//    }
//
//    private boolean colorIsNotSet(PixelColor color) {
//        return color == PixelColor.COLOR_UNSPECIFIED || color == PixelColor.COLOR_WHITE_OR_PURPLE;
//    }
//
//    public IntakePixelState(WorldRobotBase robot) {
//        this.robot = robot;
//    }
//
//    public void update() {
//        numOfPixels = 0;
//        double dist1 = robot.firstIntake.getDistance(DistanceUnit.INCH);
//        double dist2 = robot.secondIntake.getDistance(DistanceUnit.INCH);
//        if (dist1 < MIN_DIST_WITH_PIXEL) {
//            numOfPixels++;
//            pixelColors[0] = toHDColor(robot.firstIntake.getNormalizedColors());
//        }
//        if (dist2 < MIN_DIST_WITH_PIXEL) {
//            numOfPixels++;
//            pixelColors[1] = toHDColor(robot.secondIntake.getNormalizedColors());
//        }
//        setStripLightColors();
//    }
//
//    public int getNumOfPixels() {
//        return numOfPixels;
//    }
//
//    public PixelColor getPixelColor(int index) {
//        if (index < 0 || index > 1) return PixelColor.COLOR_UNSPECIFIED;
//        return pixelColors[index];
//    }
//
//    private PixelColor toHDColor(NormalizedRGBA color) {
//        HSV hsv = ImageProcessor.ColorToHsv(color);
//        if (ImageProcessor.isGreen(hsv)) {
//            return PixelColor.COLOR_GREEN;
//        }
//        if (ImageProcessor.isYellow(hsv)) {
//            return PixelColor.COLOR_YELLOW;
//        }
//        if (ImageProcessor.isPurple(hsv)) {
//            return PixelColor.COLOR_PURPLE;
//        }
//        if (ImageProcessor.isWhite(hsv)) {
//            return PixelColor.COLOR_WHITE;
//        }
//        if (ImageProcessor.couldBeWhiteOrPurple(hsv)) {
//            return PixelColor.COLOR_WHITE_OR_PURPLE;
//        }
//        return PixelColor.COLOR_UNSPECIFIED;
//    }
//
//    public enum PixelColor {
//        COLOR_UNSPECIFIED,
//        COLOR_WHITE,
//        COLOR_PURPLE,
//        COLOR_GREEN,
//        COLOR_YELLOW,
//        COLOR_WHITE_OR_PURPLE,
//        COLOR_BLACK
//    }
//}
