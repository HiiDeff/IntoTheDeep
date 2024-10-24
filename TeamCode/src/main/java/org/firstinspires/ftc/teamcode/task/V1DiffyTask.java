package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class V1DiffyTask extends TimedTask {

    public enum WhichDiffy {
        LEFT,
        RIGHT
    }

    private static final int RPM = 115;
    private static final int MAX_ROTATION_DEGREE = 1800;
    private final RobotBase robot;
    private final DiffyPosition pos;
    private final WhichDiffy diffy;
    private boolean started;

    public static void setDiffyPosition(RobotBase robot,
                                        V1DiffyTask.WhichDiffy servo, V1DiffyTask.DiffyPosition position){
        getDiffyServo(robot, servo).setPosition(robot.getDiffyTargetPos(servo, position));
    }

    private static Servo getDiffyServo(RobotBase robot, WhichDiffy diffy) {
        return diffy == WhichDiffy.LEFT ? robot.diffyLeft : robot.diffyRight;
    }

    public V1DiffyTask(RobotBase robot, WhichDiffy diffy, DiffyPosition pos) {
        this.robot = robot;
        this.diffy = diffy;
        this.pos = pos;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = getDiffyServo(robot, diffy);
            double currPos = servo.getPosition();
            double targetPos = robot.getDiffyTargetPos(diffy, pos);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum DiffyPosition {
        DOWN_CENTER,
        DOWN_LEFT,
        DOWN_RIGHT,
        FORWARD,
        UP
    }
}
