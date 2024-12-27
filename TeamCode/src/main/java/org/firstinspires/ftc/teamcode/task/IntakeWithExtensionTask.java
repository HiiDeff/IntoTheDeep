package org.firstinspires.ftc.teamcode.task;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class IntakeWithExtensionTask extends TimedTask {

    private RobotBase robot;
    private final double distInInches;
    private boolean started = false;
    private ElapsedTime timer;

    public IntakeWithExtensionTask(RobotBase robot, double distInInches, int finishTimeMillis) {
        this.robot = robot;
        this.distInInches = distInInches;
        setFinishTimeMillis((finishTimeMillis));
    }

    @Override
    protected boolean performInternal() {
        if(!started) {
            robot.runIntakeIn();
            robot.setHorizontalSlideDist(distInInches, 1.0);
            started = true;
        } else{
            if (robot.intakeSampleState.IsSampleIn()) {
                if (timer == null) {
                    timer = new ElapsedTime();
                } else {
                    if (timer.milliseconds() > 100) {
                        robot.setHorizontalSlidesPower(0.2);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    @Override
    public void cancel() {

        robot.stopIntake();
    }
}
