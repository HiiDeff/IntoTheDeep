package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Dual mode speed servo has 90 RPM under 4.8 v and 300 degree max rotation.
 */
@Config
public class RungStickTask extends TimedTask {


    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;
    public static double RUNGSTICK_UP_POSITION = 0;
    public static double RUNGSTICK_INIT_POSITION = 0.8;
    private V2RobotBase robot;
    private Position position;
    private boolean started;

    public static void setPosition(V2RobotBase robot, Position position) {
        robot.rungStick.setPosition(getTargetPos(position));
    }

    public RungStickTask(V2RobotBase robot, Position position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = robot.rungStick;
            double currPos = servo.getPosition();
            double targetPos = getTargetPos(position);
            // Only consider the half of normal time since we don't have to wait for the servo to
            // finish.
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos) / 2);
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private static double getTargetPos(Position position) {
        switch (position) {
            case UP:
                return RUNGSTICK_UP_POSITION;
            case INIT:
            case DOWN:
            default:
                return RUNGSTICK_INIT_POSITION;
        }
    }



    public enum Position {
        INIT,
        UP,
        DOWN
    }
}



