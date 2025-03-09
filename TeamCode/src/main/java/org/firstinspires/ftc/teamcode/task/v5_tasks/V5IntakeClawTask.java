package org.firstinspires.ftc.teamcode.task.v5_tasks;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.V5Robot.V5Robot;
import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.util.Utils;

public class V5IntakeClawTask extends TimedTask {

    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;

    private final V5Robot robot;
    private boolean started;
    private final Position position;

    public V5IntakeClawTask(V5Robot robot, Position position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal() {
        if (!started) {
            double currPos = robot.intakeClaw.getPosition();
            double targetPos = robot.getIntakeClawPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            robot.intakeClaw.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum Position {
        OPEN,
        CLOSE
    }
}
