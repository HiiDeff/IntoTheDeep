package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class UnboundedIntakeTask implements Task, UnboundedTask{

    private final RobotBase robot;

    public UnboundedIntakeTask(RobotBase robot) {
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
