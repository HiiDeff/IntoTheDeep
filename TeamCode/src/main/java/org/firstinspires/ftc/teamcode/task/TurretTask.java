package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class TurretTask extends TimedTask{

    private RobotBase robot;
    private boolean started = false;

    // TODO: Andrew please tune me
    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;
    private TurretPosition position;

    public TurretTask(RobotBase robot, TurretPosition position){
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if (!started){
            double currPos = robot.turret.getPosition();
            double targetPos = robot.getTurretPosition(position);

            // no idea how estimating servo time works but it is very important
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE, targetPos -currPos));

            robot.turret.setPosition(targetPos);

            started = true;
        }
        return false;
    }

    public enum TurretPosition{
        DEGREE_45,
        DEGREES_NEGATIVE_45,
        CENTER
    }

}
