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
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.util.profile.ServoProfiler;

@TeleOp(name = "Tester", group = "Test")
@Config

public class Test extends LinearOpMode {


    public MultipleTelemetry multipleTelemetry;

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private LM1Robot robot;
//    private GamePad gp1;

    public DcMotorEx verticalSlidesLeft, verticalSlidesRight;
    public DcMotorEx horizontalSlides;
    public DcMotorEx intakeRoller;
    public DcMotorEx leftBack, leftFront, rightBack, rightFront;

    public static double VSpower = 0.0, hSPower = 0.0, iPower = 0.0,
                        LBPower = 0.0, LFPower = 0.0, RBPower = 0.0, RFPower = 0.0;

    // Servos
    public Servo claw;
    public Servo intakeArm;
    public Servo bucketServo;
    public static double maxVelo = 0;

    public static double clawPos = 0.0, intakeArmPos = 0.0, bucketServoPos = 0.0;

    // Sensors
    public RevColorSensorV3 leftIntakeColorSensor, rightIntakeColorSensor;

    public GoBildaPinpointDriverRR pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Motors:
        verticalSlidesLeft = hardwareMap.get(DcMotorEx.class, "vSlidesL");
        verticalSlidesLeft.setDirection(DcMotor.Direction.REVERSE);
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalSlidesRight = hardwareMap.get(DcMotorEx.class, "vSlidesR");
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizontalSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        horizontalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlides.setDirection(DcMotor.Direction.REVERSE);
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeRoller = hardwareMap.get(DcMotorEx.class, "iRoller");
        intakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servos:
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        claw = hardwareMap.get(Servo.class, "claw");
        intakeArm = hardwareMap.get(Servo.class, "iArm");

        // Sensors:
        leftIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorLeft");
        rightIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorRight");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2d(0, 0, 0));
        /*
        public static double VSpower = 0.0, hSPower = 0.0, iPower = 0.0,
                LBPower = 0.0, LFPower = 0.0, RBPower = 0.0, RFPower = 0.0;

        public static double clawPos = 0.0, intakeArmPos = 0.0, bucketServoPos = 0.0;
        */

        waitForStart();
        while(opModeIsActive()) {
            verticalSlidesLeft.setPower(VSpower);
            verticalSlidesRight.setPower(VSpower);
            horizontalSlides.setPower(hSPower);
            intakeRoller.setPower(iPower);
            leftBack.setPower(LBPower);
            leftFront.setPower(LFPower);
            rightBack.setPower(RBPower);
            rightFront.setPower(RFPower);
            claw.setPosition(clawPos);
            intakeArm.setPosition(intakeArmPos);
            bucketServo.setPosition(bucketServoPos);

            maxVelo = Math.max(maxVelo, verticalSlidesLeft.getVelocity());

            multipleTelemetry.addData("vslidemaxvelo", maxVelo);

            multipleTelemetry.addData("rightBack ", rightBack.getCurrentPosition());
            multipleTelemetry.addData("leftBack ", leftBack.getCurrentPosition());
            multipleTelemetry.addData("rightFront ", rightFront.getCurrentPosition());
            multipleTelemetry.addData("leftFront ", leftFront.getCurrentPosition());

            multipleTelemetry.addData("v slide pos left ", verticalSlidesLeft.getCurrentPosition());
            multipleTelemetry.addData("v slide pos right ", verticalSlidesRight.getCurrentPosition());
            multipleTelemetry.addData("h slide pos ", horizontalSlides.getCurrentPosition());
            multipleTelemetry.addData("color sensor 1 ", leftIntakeColorSensor.getDistance(DistanceUnit.INCH));
            multipleTelemetry.addData("color sensor 2 ", rightIntakeColorSensor.getDistance(DistanceUnit.INCH));
            multipleTelemetry.addData("pinpoint", pinpoint.getPositionRR());
            multipleTelemetry.update();

        }

    }
}
