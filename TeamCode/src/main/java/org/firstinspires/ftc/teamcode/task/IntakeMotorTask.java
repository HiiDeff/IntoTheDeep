package org.firstinspires.ftc.teamcode.task;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
public class IntakeMotorTask implements Task{
    private static double INTAKE_IN_POWER = 1;
    private static double INTAKE_OUT_POWER = -1;
    private V2RobotBase robot;
    private Direction direction;
    //private double power;
    private boolean canceled = false;
    public IntakeMotorTask(V2RobotBase robot, Direction direction) {
        this.robot = robot;
        this.direction = direction;
        //this.power = power;
    }

    @Override
    public boolean perform() {
        if (!canceled) {
            switch (direction) {
                case REVERSE:
                    //robot.intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.intakeRoller.setPower(INTAKE_OUT_POWER);
                    break;
                case FORWARD:
                default:
                    robot.intakeRoller.setPower(INTAKE_IN_POWER);
                    break;
            }
        }
        return canceled;
    }
    @Override
    public void cancel() {
        robot.stopIntake();
        canceled = true;
    }
    public enum Direction {FORWARD, REVERSE};
}
