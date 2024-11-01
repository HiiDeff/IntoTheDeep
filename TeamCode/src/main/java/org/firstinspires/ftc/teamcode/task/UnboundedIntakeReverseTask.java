package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class UnboundedIntakeReverseTask implements Task, UnboundedTask{

    private final V2RobotBase robot;

    public UnboundedIntakeReverseTask(V2RobotBase robot) {
        this.robot = robot;
    }
    @Override
    public boolean perform() {
        robot.runIntakeOut();
        return false;
    }
    @Override
    public void cancel() {
        robot.stopIntake();
    }
}
