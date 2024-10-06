package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Pose2d;

public class AutoStates {

    private Pose2d prevEndPos;
    private int cycleNumber;
    private int initSign;
    private int sign;
    private boolean isRed;
    private int deliveryZone;
    private Pose2d pixelPos;
    private int numberOfPixels;

    private int twoPixelCount;
    private int lessPixelCount;

    public Pose2d getPrevEndPos() {
        return prevEndPos;
    }

    public void setEndPos(Pose2d pos) {
        this.prevEndPos = pos;
    }

    public int getCycleNumber() {
        return cycleNumber;
    }

    public void setCycleNumber(int cycleNumber) {
        this.cycleNumber = cycleNumber;
    }

    public int getInitSign() {
        return initSign;
    }

    public void setInitSign(int initSign) {
        this.initSign = initSign;
    }

    public boolean isRed() {
        return isRed;
    }

    public void setAlianceColor(boolean isRed) {
        this.isRed = isRed;
    }

    public int getSign() {
        return sign;
    }

    public void setSign(int sign) {
        this.sign = sign;
    }

    public int getDeliveryZone() {
        return deliveryZone;
    }

    public void setDeliveryZone(int deliveryZone) {
        this.deliveryZone = deliveryZone;
    }

    public Pose2d getPixelPos() {
        return pixelPos;
    }

    public void setPixelPos(Pose2d pixelPos) {
        this.pixelPos = pixelPos;
    }

    public void setNumberOfPixels(int num) {
        numberOfPixels = num;
    }

    public int getNumberOfPixels() {
        return numberOfPixels;
    }


    public void incrementTwoPixelCount() {
        twoPixelCount += 1;
    }

    public void incrementLessPixelCount() {
        lessPixelCount += 1;
    }

    public int getTwoPixelCount() {
        return twoPixelCount;
    }

    public int getLessPixelCount() {
        return lessPixelCount;
    }
}
