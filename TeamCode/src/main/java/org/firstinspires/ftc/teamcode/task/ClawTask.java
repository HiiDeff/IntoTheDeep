package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class ClawTask extends TimedTask {

    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;

    private final RobotBase robot;
    private boolean started;
    private final ClawPosition position;

    public ClawTask(RobotBase robot, ClawPosition position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal() {
        if (!started) {
            double currPos = robot.claw.getPosition();
            double targetPos = robot.getClawPosition(position);
            // Only consider the half of normal time since we don't have to wait for the servo to
            // finish.
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos) / 2);
            robot.claw.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum ClawPosition {
        OPEN,
        CLOSE
    }
}
