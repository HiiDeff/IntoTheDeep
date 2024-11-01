package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class UnboundedIntakeTask implements Task, UnboundedTask{

    private final V2RobotBase robot;

    public UnboundedIntakeTask(V2RobotBase robot) {
        this.robot = robot;
    }
    @Override
    public boolean perform() {
        robot.runIntakeIn();
        return false;
    }
    @Override
    public void cancel() {
        robot.stopIntake();
    }
}
