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
import org.firstinspires.ftc.teamcode.task.HorizontalSlidesTask;
import org.firstinspires.ftc.teamcode.task.IntakeMotorTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.TimedTask;
import org.firstinspires.ftc.teamcode.task.VerticalSlidesTask;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.Utils;

@TeleOp(name = "Slide Task Test", group = "Test")
@Config
public class SlideTaskTest extends LinearOpMode {



    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public MultipleTelemetry multipleTelemetry;
    //Servo servo;
    GamePad gp1;
    public static double VERTICAL_HEIGHT = 1;
    public static double HORIZONTAL_DISTANCE = 1;
    public static double POWER = 0.5;
    V2RobotBase robot;
    @Override
    public void runOpMode() {
        robot = (V2RobotBase) RobotFactory.createRobot(hardwareMap);
        multipleTelemetry = new MultipleTelemetry(telemetry,
                dashboard.getTelemetry());

        TimedTask verticalSlideTask = null;
        TimedTask horizontalSlideTask = null;
        gp1 = new GamePad(gamepad1);

        waitForStart();
        telemetry.addData("started", 0);
        telemetry.update();
        while (opModeIsActive()) {
            if (verticalSlideTask != null)
            {
                if (verticalSlideTask.perform()) {
                    verticalSlideTask = null;
                }
            }

            if (horizontalSlideTask != null)
            {
                if (horizontalSlideTask.perform()) {
                    horizontalSlideTask = null;
                }
            }

            if (gp1.onceB()) {
                multipleTelemetry.addData("ticks", (Utils.clamp(VERTICAL_HEIGHT, 0, 52)
                        * 32.91 * 1.4));
                multipleTelemetry.update();
                verticalSlideTask = new VerticalSlidesTask(robot, VERTICAL_HEIGHT, POWER);
            }
            else if (gp1.onceA()) {
                horizontalSlideTask = new HorizontalSlidesTask(robot, HORIZONTAL_DISTANCE, POWER, 2000);
            }
            gp1.update();
        }
    }
}
