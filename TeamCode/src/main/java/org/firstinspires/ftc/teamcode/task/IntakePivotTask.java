package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class IntakePivotTask extends TimedTask{

    private RobotBase robot;
    private boolean started = false;

    // TODO: Andrew please tune me
    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 355;

    // TODO: Ethan please help
    private double angle;

    public IntakePivotTask(RobotBase robot, double angle){
        this.robot = robot;
        this.angle = angle;
    }

    @Override
    public boolean performInternal(){
        if (!started){
            double currPos = robot.intakePivot.getPosition();
            // TODO: help no idea if this is how servos work
            double targetPos = getIntakePivotPosition(angle);

            // no idea how estimating servo time works but it is very important
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE, targetPos -currPos));

            robot.intakePivot.setPosition(targetPos);

            started = true;
        }
        return false;
    }

    public double getIntakePivotPosition(double angleDegrees){
        return Utils.clamp(angleDegrees, 0, MAX_ROTATION_DEGREE)/360 * MAX_ROTATION_DEGREE;
    }

}
