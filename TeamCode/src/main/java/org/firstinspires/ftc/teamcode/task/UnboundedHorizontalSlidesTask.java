package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class UnboundedHorizontalSlidesTask implements Task, UnboundedTask {

    private final RobotBase robot;
    private final double distInInches;
    private final double power;
    private boolean started;

    public UnboundedHorizontalSlidesTask(RobotBase robot, double distInInches, double power) {
        this.robot = robot;
        this.distInInches = distInInches;
        this.power = power;
    }
    @Override
    public boolean perform() {
        if(!started) {
            robot.setHorizontalSlideDist(this.distInInches, power);
            started = true;
        }
        return false;
    }
    @Override
    public void cancel() {
        robot.setHorizontalSlidesPower(0.0);
    }
}
