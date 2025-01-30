package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.V3Robot.V3Robot;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Servo Tester", group = "Test")
@Config

public class ServoTest extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private V3Robot robot;
//    private GamePad gp1;

    Servo servo;
    public static double pos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "test");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(pos);
        }
    }
}
