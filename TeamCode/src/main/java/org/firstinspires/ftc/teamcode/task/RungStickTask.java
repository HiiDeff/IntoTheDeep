package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public class RungStickTask extends TimedTask {

    public static double UP_POSITION = 0,
            DOWN_POSITION = 0.535;


    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;
    private V2Robot robot;
    private Position position;
    private boolean started;

    public static void setPosition(V2Robot robot, Position position) {
        robot.rungStick.setPosition(getTargetPos(position));
    }

    public RungStickTask(V2Robot robot, Position position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            double currPos = robot.rungStick.getPosition();
            double targetPos = getTargetPos(position);
            // Only consider the half of normal time since we don't have to wait for the servo to
            // finish.
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos) / 2);
            robot.rungStick.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private static double getTargetPos(Position position) {
        switch (position) {
            case UP:
                return UP_POSITION;
            case DOWN:
            default:
                return DOWN_POSITION;
        }
    }

    public enum Position {
        UP,
        DOWN
    }
}



