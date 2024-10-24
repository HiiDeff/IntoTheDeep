package org.firstinspires.ftc.teamcode.task;


import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class HorizontalSlidesTask extends TimedTask{

    private V2RobotBase robot;
    private final double distInInches;
    private final double power;
    private boolean started = false;
    public HorizontalSlidesTask(V2RobotBase robot, double distInInches, double power,
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
        if (Math.abs(robot.getHorizontalSlideDist() - distInInches) < 0.5) {
            cancel();
            return true;
        }
        return false;
    }

    @Override
    public void cancel() {
        robot.setHorizontalSlidesPower(0.0);
    }


}
