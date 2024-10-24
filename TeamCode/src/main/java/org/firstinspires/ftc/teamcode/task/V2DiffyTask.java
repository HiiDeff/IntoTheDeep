package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class V2DiffyTask extends TimedTask {

    public enum WhichDiffy {
        LEFT,
        RIGHT
    }

    private static final int RPM = 115;
    private static final int MAX_ROTATION_DEGREE = 1800;
    private final V2RobotBase robot;
    private final DiffyPosition pos;
    private final WhichDiffy diffy;
    private boolean started;

    public static void setDiffyPosition(V2RobotBase robot,
            V2DiffyTask.WhichDiffy servo, V2DiffyTask.DiffyPosition position){
        getDiffyServo(robot, servo).setPosition(robot.getDiffyTargetPos(servo, position));
    }

    private static Servo getDiffyServo(V2RobotBase robot, WhichDiffy diffy) {
        return diffy == WhichDiffy.LEFT ? robot.diffyLeft : robot.diffyRight;
    }

    public V2DiffyTask(V2RobotBase robot, WhichDiffy diffy, DiffyPosition pos) {
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
        DELIVER,
        CARRY,
        TRANSFER
    }
}
