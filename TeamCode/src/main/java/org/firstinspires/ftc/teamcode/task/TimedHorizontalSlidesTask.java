package org.firstinspires.ftc.teamcode.task;


import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class TimedHorizontalSlidesTask extends TimedTask{

    private RobotBase robot;
    private final double distInInches;
    private final double power;
    private boolean started = false;

    public TimedHorizontalSlidesTask(RobotBase robot, double distInInches, double power,
                                int finishTimeMillis) {
        this.robot = robot;
        this.distInInches = distInInches;
        this.power = power;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            started = true;
            robot.setHorizontalSlideDist(this.distInInches, power);
        }
        return false;
    }

    @Override
    public void cancel() {
        robot.setHorizontalSlidesPower(0.0);
    }
}
