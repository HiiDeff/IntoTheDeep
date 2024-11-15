package org.firstinspires.ftc.teamcode.task;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class IntakeReverseTask extends TimedTask {

    private V2RobotBase robot;
    private boolean started = false;

    public IntakeReverseTask(V2RobotBase robot, int finishTimeMillis) {
        this.robot = robot;
        setFinishTimeMillis((finishTimeMillis));
    }

    @Override
    protected boolean performInternal() {
        if(!started) {
            started = true;
            robot.runIntakeOut();
        }
        return false;
    }

    @Override
    public void cancel() {
        robot.stopIntake();
    }
}
