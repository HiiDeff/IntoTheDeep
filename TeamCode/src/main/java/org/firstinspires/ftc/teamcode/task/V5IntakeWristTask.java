package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.V5Robot.V5Robot;
import org.firstinspires.ftc.teamcode.util.Utils;

public class V5IntakeWristTask extends TimedTask {

    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;

    private final V5Robot robot;
    private boolean started;
    private final WristPosition position;

    public V5IntakeWristTask(V5Robot robot, WristPosition position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal() {
        if (!started) {
            double currPos = robot.intakeWrist.getPosition();
            double targetPos = robot.getWristPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            robot.intakeWrist.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum WristPosition {
        VERTICAL,
        HORIZONTAL
    }
}
