package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.LM1Robot;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.Utils;

@TeleOp(name = "Servo Tester", group = "Test")
@Config

public class ServoTest extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private LM1Robot robot;
    private GamePad gp1;

    Servo leftDiffy;
    Servo rightDiffy;
    public static double leftPos = 0.0, rightPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        //robot = new LM1Robot(hardwareMap);


        leftDiffy = hardwareMap.get(Servo.class, "intakeArm");
        //rightDiffy = hardwareMap.get(Servo.class, "diffyRight");


        waitForStart();

        while (opModeIsActive()) {
            leftDiffy.setPosition(leftPos);
            //rightDiffy.setPosition(rightPos);
        }
    }
}
