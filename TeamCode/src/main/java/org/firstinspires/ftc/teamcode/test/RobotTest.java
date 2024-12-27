package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.IntakeSampleState;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V3Robot;
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.util.profile.ServoProfiler;

@TeleOp(name = "Robot Test", group = "Test")
@Config

public class RobotTest extends LinearOpMode {


    public MultipleTelemetry multipleTelemetry;

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private LM1Robot robot;
//    private GamePad gp1;

    private V3Robot robot;
    public static double targetPos = 10.0;

    @Override
    public void runOpMode() throws InterruptedException {

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = (V3Robot) RobotFactory.createRobot(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {

            robot.setVerticalSlidesHeight(targetPos, 1.0);


            robot.updateEverything();
            multipleTelemetry.addData("pid power", robot.verticalSlidesPID.getPower());
            multipleTelemetry.update();

        }

    }
}
