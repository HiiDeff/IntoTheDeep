package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "FirstTest", group = "R2BTest")
@Config

public class Robot2BTest extends LinearOpMode {

    public static double HANG_LEFT_POS = 0.0;
    public static double HANG_RIGHT_POS = 1.0;
    public static double HORIZONTAL_SLIDES_POWER = 0.5;
    public static double VERTICAL_SLIDES_POWER = 0.5;

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private GamePad gp1;

    DcMotorEx horizontalSlides;
    DcMotorEx verticalSlidesRight;
    DcMotorEx verticalSlidesLeft;
    Servo hangLeft;
    Servo hangRight;

    @Override
    public void runOpMode() throws InterruptedException {

        gp1 = new GamePad(gamepad1);

        hangLeft = hardwareMap.get(Servo.class, "hangLeft");
        hangRight = hardwareMap.get(Servo.class, "hangRight");
        horizontalSlides = hardwareMap.get(DcMotorEx.class, "horizontalSlides");
        verticalSlidesLeft = hardwareMap.get(DcMotorEx.class, "verticalSlidesLeft");
        verticalSlidesRight = hardwareMap.get(DcMotorEx.class, "verticalSlidesRight");
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gp1.onceA()) {
                Log.d("lswu", "A pressed");
                hangRight.setPosition(HANG_RIGHT_POS);
            } else if (gp1.onceB()) {
                Log.d("lswu", "B pressed");
                hangLeft.setPosition(HANG_LEFT_POS);
            }

            double horizontalPower = clampRange(gp1.leftStickY(), HORIZONTAL_SLIDES_POWER);
            horizontalSlides.setPower(horizontalPower);
            double verticalPower = clampRange(gp1.rightStickY(), VERTICAL_SLIDES_POWER);
            verticalSlidesRight.setPower(verticalPower);
            verticalSlidesLeft.setPower(verticalPower);

            telemetry.addData("right y: %f", verticalPower);
            telemetry.addData("left y: %f", horizontalPower);
            telemetry.update();
            gp1.update();
        }
    }

    private double clampRange(double val, double limit) {
        return Math.min(Math.max(val, -1 * limit), limit);
    }
}
