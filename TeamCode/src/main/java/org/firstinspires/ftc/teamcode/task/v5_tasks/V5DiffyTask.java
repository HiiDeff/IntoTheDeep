package org.firstinspires.ftc.teamcode.task.v5_tasks;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotBase2;
import org.firstinspires.ftc.teamcode.drive.V5Robot.V5Robot;
import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * 5-turn torque servo has 40 RPM under 4.8 v and 4.5 turn max rotation.
 */
@Config
public class V5DiffyTask extends TimedTask {
    public enum WhichDiffy {
        LEFT,
        RIGHT
    }


    private static final int RPM = 20; // 40 RPM without load
    private static final int MAX_ROTATION_DEGREE = 1620; // 4.5 turn


    private final RobotBase2 robot;
    private final Position position;
    private final WhichDiffy hang;
    private boolean started;

    public static void setHangPosition(RobotBase2 robot,
                                       WhichDiffy placer, Position position) {
        getDiffyServo(robot, placer).setPosition(robot.getDiffyTargetPos(placer, position));
    }

    public V5DiffyTask(RobotBase2 robot, WhichDiffy hang, Position position) {
        this.robot = robot;
        this.hang = hang;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = getDiffyServo(robot, hang);
            double currPos = servo.getPosition();
            double targetPos = robot.getDiffyTargetPos(hang, position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private static Servo getDiffyServo(RobotBase2 robot, WhichDiffy hang) {
        return hang == WhichDiffy.LEFT ? robot.diffyLeft : robot.diffyRight;
    }

    public enum Position {
        TRANSFER,
        FLAT,
        UP,
        CLIP,
        DELIVER
    }
}



