package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

public class UnboundedIntakeTask implements Task, UnboundedTask{

    private final V2RobotBase robot;
    private final double intakeSpeed;

    public UnboundedIntakeTask(V2RobotBase robot, double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        this.robot = robot;
    }
    @Override
    public boolean perform() {
        robot.runIntakeWithPower(intakeSpeed);
        return false;
    }
    @Override
    public void cancel() {
        robot.stopIntake();
    }
}
