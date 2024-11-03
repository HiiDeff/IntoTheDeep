package org.firstinspires.ftc.teamcode.task;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class IntakeWithExtensionTask extends TimedTask {

    private V2RobotBase robot;
    private double length;
    private boolean started = false;
    protected ElapsedTime timer;

    public IntakeWithExtensionTask(V2RobotBase robot, double extensionLength, int finishTimeMillis) {
        this.robot = robot;
        length = extensionLength;
        setFinishTimeMillis((finishTimeMillis));
    }

    @Override
    public void cancel() {
        robot.stopIntake();
    }

    @Override
    protected boolean performInternal() {
        if(!started) {
            started = true;
            robot.runIntakeIn();
            robot.setHorizontalSlideDist(length, 1.0);
        }
        else{
            if (robot.intakeSampleState.IsSampleIn())
            {
                if (timer == null)
                {
                    timer = new ElapsedTime();
                }
                else
                {
                    if (timer.milliseconds() > 100)
                    {
                        robot.setHorizontalSlidesPower(0.2);
                        return true;
                    }
                }
            }
        }

        return false;
    }
}
