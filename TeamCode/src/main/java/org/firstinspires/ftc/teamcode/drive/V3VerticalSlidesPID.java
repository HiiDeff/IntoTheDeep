package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.util.pid.PIDModel;

public class V3VerticalSlidesPID extends PIDModel {

    private final V3Robot robot;
    private int targetPosition;

    public V3VerticalSlidesPID(V3Robot robot, double minPower, double maxPower, double pK, double iK, double dK) {
        super(minPower, maxPower, pK, iK, dK);
        this.robot = robot;
    }

    public void setTargetPosition(int targetPositionTicks) {
        targetPosition = targetPositionTicks;
    }

    @Override
    public void cancel() {
        //what to do if cancel? probably just holding power but don't use cancel()
    }

    @Override
    public double getError() {
        return targetPosition - robot.getVerticalSlidesHeightTicks(); //return error of position
    }

    @Override
    protected double getStopError() {
        return 5;
    } //minimum error to stop adjusting

    @Override
    protected double getStopSpeed() {
        return 0.05;
    } //minimum speed to stop adjusting
}
