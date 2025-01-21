package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Servo Tester", group = "Test")
//@Disabled
@Config

public class ServoTest extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private V2Robot robot;
    private GamePad gp1;

    Servo intakeArm;
    Servo deliveryArm;

    DcMotorEx intakeMotor;
    //public static double pos = 0.78;
    public static double intakeUpPos = 0.34, intakeDownPos = 0.92;
    public static double deliveryUpPos = 0.87, deliveryDownPos = 0.32;
    public static double POWER = 0.7;
    //0.34 intake transfer pos
    //0.92 intake intake pos
    //0.32 delivery transfer pos
    //0.87 delivery intake(from wall) pos

    @Override
    public void runOpMode() throws InterruptedException {
        gp1 = new GamePad(gamepad1);
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        deliveryArm = hardwareMap.get(Servo.class, "deliveryArm");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        waitForStart();

        while (opModeIsActive()) {
            if(gp1.onceA()) {
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
            } else if(gp1.dpadUp()){
                intakeArm.setPosition(intakeUpPos);
            } else if(gp1.dpadDown()){
                intakeArm.setPosition(intakeDownPos);
            } else if(gp1.rightBumper()){
                deliveryArm.setPosition(deliveryUpPos);
            }else if (gp1.leftBumper()){
                deliveryArm.setPosition(deliveryDownPos);
            }
            gp1.update();;
        }
    }
}
