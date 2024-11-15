package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tester", group = "Test")
@Config

public class Test extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private LM1Robot robot;
//    private GamePad gp1;

    DcMotorEx intakeMotor;
    Servo servo;
    public static boolean servoDown = true;
    public static double servoPos = 0.95, motorPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        //robot = new LM1Robot(hardwareMap);

        servo = hardwareMap.get(Servo.class, "servo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
//            if(servoDown) servo.setPosition(0.95);
            servo.setPosition(servoPos);
            intakeMotor.setPower(motorPower);
        }
    }
}
