package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Tester", group = "Test")
//@Disabled
@Config

public class Test extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private LM1Robot robot;
//    private GamePad gp1;

    DcMotorEx intakeMotor;

    Servo deliveryServo;
    Servo intakeServo;
    Servo servo;
    private GamePad gp1;

    public static double POWER = -0.7;
    public static double deliveryServoPos = 0.95, intakeServoPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        gp1 = new GamePad(gamepad1);
        //intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deliveryServo = hardwareMap.get(Servo.class, "deliveryServo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
;
        waitForStart();

        while (opModeIsActive()) {
            /*if(gp1.onceA()) {
                intakeMotor.setPower(POWER);
                telemetry.addData("A Pressed", "power: " + POWER);
                telemetry.update();
            } else if (gp1.onceB()) {
                intakeMotor.setPower(-POWER);
                telemetry.addData("B Pressed", "power: " + POWER);
                telemetry.update();
            } else if(gp1.onceX()){
                intakeMotor.setPower(0.0);
                telemetry.addData("Stopped Motor", "power: " + POWER);
                telemetry.update();
            }*/
            gp1.update();

//            telemetry.update();
        }
    }
}
