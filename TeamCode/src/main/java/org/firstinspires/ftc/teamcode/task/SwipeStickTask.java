package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public class SwipeStickTask extends TimedTask {

    public static double OUT_POSITION = 0.03,
            IN_POSITION = 1.0;


    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 200;
    private V2RobotBase robot;
    private Position position;
    private boolean started;

    public static void setPosition(V2RobotBase robot, Position position) {
        robot.swipeStick.setPosition(getTargetPos(position));
    }

    public SwipeStickTask(V2RobotBase robot, Position position) {
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



