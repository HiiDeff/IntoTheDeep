package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class UnboundedIntakeReverseTask implements Task, UnboundedTask{

    private final RobotBase robot;

    public UnboundedIntakeReverseTask(RobotBase robot) {
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
