package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "Intake Tester", group = "Test")
@Config
@Disabled
public class IntakeTest extends LinearOpMode {


    public MultipleTelemetry multipleTelemetry;

    //private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private LM1Robot robot;
//    private GamePad gp1;

    public DcMotorEx verticalSlidesLeft, verticalSlidesRight;
    public DcMotorEx horizontalSlides;
    public DcMotorEx intakeRoller;
    public DcMotorEx leftBack, leftFront, rightBack, rightFront;

    public static double VSpower = 0.0, hSPower = 0.0, iPower = 1.0,
                        LBPower = 0.0, LFPower = 0.0, RBPower = 0.0, RFPower = 0.0;

    // Servos
    public Servo claw;
    public Servo intakeArm;
    public Servo deliveryArmServo;
    public static double maxVelo = 0;

    public static double clawPos = 0.0, intakeArmPos = 0.0, deliveryArmServoPos = 0.0;
    public static double INTAKE_ARM_INTAKE = 0.87, INTAKE_ARM_NEUTURE = 0.6, INTAKE_ARM_TRANSFER = 0.36;
    public static double DELIVERY_ARM_DELIVERY = 0.7, DELIVERY_ARM_TRANSFER = 0.27;
    public static double CLAW_OPEN = 0.25, CLAW_CLOSE = 0.1;

    private GamePad gp1;
    // Sensors

    public GoBildaPinpointDriverRR pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        gp1 = new GamePad(gamepad1);
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
        deliveryArmServo = hardwareMap.get(Servo.class, "dArm");
        claw = hardwareMap.get(Servo.class, "claw");
        intakeArm = hardwareMap.get(Servo.class, "iArm");


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
            if (gp1.onceA()) {
                intakeRoller.setPower(iPower);
            } else if (gp1.onceB()) {
                intakeRoller.setPower(-iPower);
            } else if (gp1.onceX()) {
                intakeRoller.setPower(0);
            } else if (gp1.onceDpadUp()) {
                deliveryArmServo.setPosition(DELIVERY_ARM_TRANSFER);
                claw.setPosition(CLAW_OPEN);
                intakeArm.setPosition(INTAKE_ARM_TRANSFER);
            } else if (gp1.onceDpadDown()) {
                intakeArm.setPosition(INTAKE_ARM_INTAKE);
            } else if (gp1.onceDpadLeft()) {
                intakeArm.setPosition(INTAKE_ARM_NEUTURE);
            } else if (gp1.onceLeftTrigger()) {
                deliveryArmServo.setPosition(DELIVERY_ARM_DELIVERY);
            } else if (gp1.onceRightTrigger()) {
                deliveryArmServo.setPosition(DELIVERY_ARM_TRANSFER);
            } else if (gp1.onceLeftBumper()) {
                claw.setPosition(CLAW_CLOSE);
            } else if (gp1.onceRightBumper()) {
                claw.setPosition(CLAW_OPEN);
            } else if (gp1.onceY()) {

            }

            gp1.update();
            multipleTelemetry.update();

        }

    }
}
