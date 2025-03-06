package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.V5Robot.V5Robot;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Dual mode speed servo has 90 RPM under 4.8 v and 300 degree max rotation.
 */
@Config
public class V5IntakeArmTask extends TimedTask {

    private static final int RPM = 50;
    private static final int MAX_ROTATION_DEGREE = 300;

    private V5Robot robot;
    private Position position;
    private boolean started;

    public V5IntakeArmTask(V5Robot robot, Position position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    protected boolean performInternal() {
        if (!started) {
            Servo servo = robot.intakeArm;
            double currPos = servo.getPosition();
            double targetPos = robot.getIntakeArmPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            servo.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum Position {
        TRANSFER,
        DOWN
    }
}



