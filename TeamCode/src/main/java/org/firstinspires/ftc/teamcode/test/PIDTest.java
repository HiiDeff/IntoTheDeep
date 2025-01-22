package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V3Robot.V3Robot;
import org.firstinspires.ftc.teamcode.drive.V4Robot.V4Robot;

@TeleOp(name = "PID Test", group = "Test")
@Config

public class PIDTest extends LinearOpMode {


    public MultipleTelemetry multipleTelemetry;

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private LM1Robot robot;
//    private GamePad gp1;

    private V4Robot robot;
    public static double targetPos = 10.0;

    @Override
    public void runOpMode() throws InterruptedException {

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = (V4Robot) RobotFactory.createRobot(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {

            robot.setVerticalSlidesHeight(targetPos);


            robot.updateEverything();
            multipleTelemetry.addData("current position", robot.getVerticalSlidesHeightInch());
            multipleTelemetry.addData("PID target position", targetPos);
            multipleTelemetry.update();

        }

    }
}
