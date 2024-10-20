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
public class IntakeLiftTask extends TimedTask {


    public static double LEFT_UP_POS = 0.16;
    public static double LEFT_DOWN_POS = 0.69;
    public static double RIGHT_UP_POS = 0.82;
    public static double RIGHT_DOWN_POS = 0.36;
    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;

    private V2RobotBase robot;
    private Position position;
    private boolean started;

    public static void setPlacerPosition(V2RobotBase robot, Position position) {
        robot.intakeArm.setPosition(getTargetPos(position));
    }

    public IntakeLiftTask(V2RobotBase robot, Position position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = robot.intakeArm;
            double currPos = servo.getPosition();
            double targetPos = getTargetPos(position);
            // Only consider the half of normal time since we don't have to wait for the servo to
            // finish.
            setFinishTimeMillis(Utils.getEstimatedServoTime(V2Robot.INTAKE_ARM_RPM, V2Robot.INTAKE_ARM_RANGE,
                    targetPos - currPos) / 2);
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    private static double getTargetPos(Position position) {
        switch (position) {
            case TRANSFER:
                return V2Robot.INTAKE_ARM_TRANSFER;
            case CLEAR:
                return V2Robot.INTAKE_ARM_CLEAR;
            case DOWN:
            default:
                return V2Robot.INTAKE_ARM_DOWN;
        }
    }



    public enum Position {
        TRANSFER,
        DOWN,
        CLEAR
    }
}



