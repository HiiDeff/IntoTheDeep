package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Motor Tester", group = "Test")
@Disabled
@Config

public class MotorTest extends LinearOpMode {

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private V2Robot robot;
    private GamePad gp1;

    //DcMotorEx testMotor;
    public DcMotorEx horizontalSlides, verticalSlides;

    //public static motorMode mode = motorMode.RUN_WITHOUT_ENCODER;

    public static double RUNTO_POSITION = 0.0;
    public static double POWER = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        //robot = new LM1Robot(hardwareMap);

        horizontalSlides = hardwareMap.get(DcMotorEx.class, "horizontalSlides");
        horizontalSlides.setDirection(DcMotorEx.Direction.REVERSE);
        horizontalSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        verticalSlides = hardwareMap.get(DcMotorEx.class, "verticalSlides");
        verticalSlides.setDirection(DcMotorEx.Direction.REVERSE);
        verticalSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (opModeIsActive()) {

            verticalSlides.setPower(POWER);

            telemetry.addData("h pos", horizontalSlides.getCurrentPosition());
            telemetry.addData("v pos", verticalSlides.getCurrentPosition());
            telemetry.update();

        }

//        while(opModeIsActive()) {
//            switch (mode) {
//                case RUN_WITHOUT_ENCODER:
//                    setPower();
//                case RUN_TO_POSITION:
//                    runToPos();
//                default:
//                    setPower();
//            }
//        }

    }
//    private void setPower() {
//        if(testMotor.getMode() != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
//            testMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        }
//        testMotor.setPower(Utils.clamp(POWER, -1.0, 1.0));
//    }
//    private void runToPos() {
//        testMotor.setTargetPosition((int) RUNTO_POSITION);
//        if(testMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
//            testMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            testMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        }
//        testMotor.setPower(Utils.clamp(POWER, -1.0, 1.0));
//    }
    private enum motorMode {
        RUN_WITHOUT_ENCODER,
        RUN_TO_POSITION;
    }
}
