package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.IntakeMotorTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Intake Test", group = "Test")
@Config
public class IntakeTest extends LinearOpMode {



    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public MultipleTelemetry multipleTelemetry;
    //Servo servo;
    GamePad gp1;
    public static double LengthInInches = 1;
    V2RobotBase robot;
    @Override
    public void runOpMode() {
        robot = (V2RobotBase) RobotFactory.createRobot(hardwareMap);
        multipleTelemetry = new MultipleTelemetry(telemetry,
                dashboard.getTelemetry());

        TimedTask liftTask = null;
        Task rollerTask = null;
        gp1 = new GamePad(gamepad1);

        int runToPos = 0;

        boolean rollerRunning = false;
        waitForStart();
        telemetry.addData("started", 0);
        telemetry.update();
        while (opModeIsActive()) {
            if (liftTask != null)
            {
                if (liftTask.perform()) {
                    liftTask = null;
                }
            }

            else if (gp1.onceB()) {
                telemetry.addData("clicked B", 1);
                telemetry.update();


                liftTask = new IntakeArmTask(robot, IntakeArmTask.Position.DOWN);
            }
            else if (gp1.onceA()) {
                telemetry.addData("clicked A", 1);
                telemetry.update();

                liftTask = new IntakeArmTask(robot, IntakeArmTask.Position.TRANSFER);
            }
            else if (gp1.onceX()) {
                telemetry.addData("clicked X", 1);
                telemetry.update();

                liftTask = new IntakeArmTask(robot, IntakeArmTask.Position.CLEAR);
            }
            if (rollerTask != null)
            {
                if (rollerTask.perform()) {
                    //rollerTask.cancel();
                    rollerTask = null;
                    rollerRunning = false;
                }
            }
            else if (gp1.leftBumper()) {
                if (rollerRunning)
                {
                    robot.stopIntake();
                }
                rollerTask = new IntakeMotorTask(robot, IntakeMotorTask.Direction.REVERSE);
                rollerRunning = true;
            }
            else if (gp1.rightBumper()) {
                if (rollerRunning)
                {
                    robot.stopIntake();
                }
                rollerTask = new IntakeMotorTask(robot, IntakeMotorTask.Direction.FORWARD);
                rollerRunning = true;
            }
            else {

            }
            gp1.update();
        }
    }
}
