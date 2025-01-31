package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.V3Robot.V3Robot;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public class SwipeStickTask extends TimedTask {

    public static double OUT_POSITION = 0.65,
            IN_POSITION = 0.12;


    private static final int RPM = 30;
    private static final int MAX_ROTATION_DEGREE = 300;
    private RobotBase robot;
    private Position position;
    private boolean started;

    public static void setPosition(RobotBase robot, Position position) {
        robot.swipeStick.setPosition(getTargetPos(position));
    }

    public SwipeStickTask(RobotBase robot, Position position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            double currPos = robot.swipeStick.getPosition();
            double targetPos = getTargetPos(position);
            // Only consider the half of normal time since we don't have to wait for the servo to
            // finish.
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos) / 2);
            robot.swipeStick.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private static double getTargetPos(Position position) {
        switch (position) {
            case OUT:
                return OUT_POSITION;
            case IN:
            default:
                return IN_POSITION;
        }
    }

    public enum Position {
        OUT,
        IN
    }
}



