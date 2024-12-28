package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "FirstTest", group = "R2BTest")
@Config

public class Robot2BTest extends LinearOpMode {

    public static double HANG_LEFT_INIT_POS = 0.315;
    public static double HANG_LEFT_RELEASE_POS = 0.15;
    public static double HANG_LEFT_HANG_POS = 0.83;
    public static double HANG_RIGHT_INIT_POS = 0.63;
    public static double HANG_RIGHT_RELEASE_POS = 0.78;
    public static double HANG_RIGHT_HANG_POS = 0.13;
    public static double HORIZONTAL_SLIDES_POWER = 0.5;
    public static double VERTICAL_SLIDES_POWER = 0.5;
    public static double INTAKE_POWER = 0.5;

    public static double INTAKE_ROTATE_POS = 0.2;
    public static double CLAW_POS = 0.5;
    public static double BUCKET_SERVO_POS = 0.5;


    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private GamePad gp1;

    DcMotorEx horizontalSlides;
    DcMotorEx verticalSlidesRight;
    DcMotorEx verticalSlidesLeft;

    DcMotorEx intakeMotor;
    Servo hangLeft;
    Servo hangRight;
    Servo intakeRotate;
    Servo claw;
    Servo bucketServo;


    RevColorSensorV3 leftColor;
    RevColorSensorV3 rightColor;

    @Override
    public void runOpMode() throws InterruptedException {

        gp1 = new GamePad(gamepad1);

        horizontalSlides = hardwareMap.get(DcMotorEx.class, "horizontalSlides");
        verticalSlidesLeft = hardwareMap.get(DcMotorEx.class, "vSlidesL");
        verticalSlidesRight = hardwareMap.get(DcMotorEx.class, "vSlidesR");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeRoller");

        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        intakeRotate = hardwareMap.get(Servo.class, "intakeArm");
        intakeRotate.setPosition(INTAKE_ROTATE_POS);

        claw = hardwareMap.get(Servo.class, "claw");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        claw.setPosition(CLAW_POS);

        leftColor = hardwareMap.get(RevColorSensorV3.class, "intakeColorLeft");
        rightColor = hardwareMap.get(RevColorSensorV3.class, "intakeColorRight");
        hangLeft = hardwareMap.get(Servo.class, "leftHang");
        hangRight = hardwareMap.get(Servo.class, "rightHang");

        waitForStart();

        while (opModeIsActive()) {
            if (gp1.back()) {
                if (gp1.onceA()){
                    hangRight.setPosition(HANG_RIGHT_INIT_POS);
                    hangLeft.setPosition(HANG_LEFT_INIT_POS);
                } else if (gp1.onceB()) {
                    hangRight.setPosition(HANG_RIGHT_HANG_POS);
                    hangLeft.setPosition(HANG_LEFT_HANG_POS);
                } else if (gp1.onceX()) {
                    hangRight.setPosition(HANG_RIGHT_RELEASE_POS);
                    hangLeft.setPosition(HANG_LEFT_RELEASE_POS);
                }
            }

            if (gp1.onceY()) {
                intakeRotate.setPosition(INTAKE_ROTATE_POS);
            } else if (gp1.onceLeftBumper()) {
                claw.setPosition(CLAW_POS);
                bucketServo.setPosition(BUCKET_SERVO_POS);
            }

            double horizontalPower = clampRange(gp1.leftStickY(), HORIZONTAL_SLIDES_POWER);
            horizontalSlides.setPower(horizontalPower);
            double verticalPower = clampRange(gp1.rightStickY(), VERTICAL_SLIDES_POWER);
            verticalSlidesRight.setPower(verticalPower);
            verticalSlidesLeft.setPower(verticalPower);
            double intakePower = 0.0;
            if (gp1.dpadUp()) intakePower = INTAKE_POWER;
            if (gp1.dpadDown()) intakePower = -INTAKE_POWER;
            intakeMotor.setPower(intakePower);

            telemetry.addData("right y: %f", verticalPower);
            telemetry.addData("left y: %f", horizontalPower);
            telemetry.addData("left color:%f", leftColor.getDistance(DistanceUnit.INCH));
            telemetry.addData("right color: %f", rightColor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            gp1.update();
        }
    }

    private double clampRange(double val, double limit) {
        return Math.min(Math.max(val, -1 * limit), limit);
    }
}
