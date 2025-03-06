package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.V5Robot.V5Robot;
import org.firstinspires.ftc.teamcode.util.Utils;

public class V5IntakeClawTask extends TimedTask {

    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;

    private final V5Robot robot;
    private boolean started;
    private final ClawPosition position;

    public V5IntakeClawTask(V5Robot robot, ClawPosition position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal() {
        if (!started) {
            double currPos = robot.claw.getPosition();
            double targetPos = robot.getClawPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
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
