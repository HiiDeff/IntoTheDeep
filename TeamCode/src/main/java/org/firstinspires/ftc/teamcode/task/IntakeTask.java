package org.firstinspires.ftc.teamcode.task;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class IntakeTask extends TimedTask {

    private V2RobotBase robot;
    private boolean started = false;
    protected ElapsedTime timer;

    public IntakeTask(V2RobotBase robot, int finishTimeMillis) {
        this.robot = robot;
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
        }
 /*       else{
            if (robot.intakeSampleState.IsSampleIn())
            {
                if (timer == null)
                {
                    timer = new ElapsedTime();
                }
                else
                {
                    if (timer.milliseconds() > 200)
                    {
                        robot.stopIntake();
                        return true;
                    }
                }
            }
        }

  */
        return false;
    }
}
