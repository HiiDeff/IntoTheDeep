package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class DiffyIntakeTask implements Task{

    private RobotBase robot;

    public DiffyIntakeTask(RobotBase robot) {

    }
    @Override
    public boolean perform() {
        return false;
    }

    @Override
    public void cancel() {
        Task.super.cancel();
    }

    public enum DiffyPosition {
        DOWN_CENTER,
        DOWN_LEFT,
        DOWN_RIGHT,
        FORWARD,
        UP
    }
}
