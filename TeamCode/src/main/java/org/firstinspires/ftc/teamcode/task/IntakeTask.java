package org.firstinspires.ftc.teamcode.task;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class IntakeTask extends TimedTask {

    private V2RobotBase robot;
    private boolean started = false;

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
        return false;
    }
}
