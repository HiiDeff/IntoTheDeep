package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Servo Tester", group = "Test")
@Config

public class ServoTest extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private V2Robot robot;
    private GamePad gp1;

    Servo claw;
    public static double pos = 0.78;
    //public static double leftPos = 0.58, rightPos = 0.41;

    @Override
    public void runOpMode() throws InterruptedException {

        claw = hardwareMap.get(Servo.class, "intakeArm");

        waitForStart();

        while (opModeIsActive()) {
            claw.setPosition(pos);
        }
    }
}
