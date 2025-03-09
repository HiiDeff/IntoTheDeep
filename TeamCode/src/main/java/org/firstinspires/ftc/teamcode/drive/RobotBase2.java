package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.V5Robot.V5VerticalSlidesPID;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryArmTask;
import org.firstinspires.ftc.teamcode.task.HangTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.SwipeStickTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5DeliveryClawTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5DiffyTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5IntakeClawTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5IntakePivotTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5IntakeWristTask;
import org.firstinspires.ftc.teamcode.task.v5_tasks.V5TurretTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightSampleDetector;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.util.profile.ServoProfiler;

@Config
public abstract class RobotBase2 extends PinpointDrive {


    ///////////////////* CONSTANTS *///////////////////

    // Vertical Slides Constants
    public static double VERTICAL_EXTEND_MIN_IN = -0.2, VERTICAL_EXTEND_MAX_IN;
    public static double VERTICAL_TICKS_PER_INCH;
    public static double VERTICAL_SLIDE_DOWN_POWER_TELEOP = -0.5, VERTICAL_SLIDE_DOWN_POWER_AUTO = -0.7;

    // Horizontal Slides Constants
    public static double HORIZONTAL_EXTEND_MIN_IN = -1.0, HORIZONTAL_EXTEND_MAX_IN = 22.0;
    public static double HORIZONTAL_TICKS_PER_INCH = 31.8181818;
    public static double HORIZONTAL_SLIDE_TRANSFER_POS = 64 / HORIZONTAL_TICKS_PER_INCH;

    // Intake Constants


    ///////////////////* HARDWARE *///////////////////


    // Common
    protected final HardwareMap hardwareMap;

    // Motors
    public final DcMotorEx horizontalSlides;
    public final DcMotorEx verticalSlidesLeft, verticalSlidesRight;
    // Servos
    public final Servo diffyLeft, diffyRight, deliveryClaw, turret, intakeArm, intakePivot, intakeWrist, intakeClaw;
    public final Servo swipeStick;

    // Motion PIDs + Profilers
    public static PIDCoefficients VERTICAL_SLIDES_PID_COEFFICIENTS; // = new PIDCoefficients(0.3, 1.0, 0.015, 0.0, 0.0,0.22); // new PIDCoefficients(0.3, 1.0, 0.006, 0.0, 0.0,0.16); //0.07 min ff, 0.37 = max ff, so do
    public final V5VerticalSlidesPID verticalSlidesPID;

    // Lights
    private final RevBlinkinLedDriver light;

    // Vision
    public final Limelight3A limelight;
    public static LimelightConfig LLConfig = new LimelightConfig(640, 480,
            -30, 54,41,
            7.5,6.0,14.00); //ROBOT 5 MEASUREMENTS
    private final LimelightSampleDetector limelightSampleDetector;

    // Cached Encoder Values
    private int verticalSlidesPos = 0;
    private int horizontalSlidesPos = 0;
    private int lastHorizontalSlidesPos = 0;

    // States
    private boolean verticalSlidesRunToPosition = false;

    public boolean hangUp = false;


    public RobotBase2(HardwareMap hardwareMap) {

        super(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Motors:
        horizontalSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        horizontalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalSlidesLeft = hardwareMap.get(DcMotorEx.class, "vSlidesL");
        verticalSlidesLeft.setDirection(DcMotor.Direction.REVERSE);
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight = hardwareMap.get(DcMotorEx.class, "vSlidesR");
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        VERTICAL_SLIDES_PID_COEFFICIENTS = getVerticalSlidesPidCoefficients();
        verticalSlidesPID = new V5VerticalSlidesPID(this, VERTICAL_SLIDES_PID_COEFFICIENTS);

        // Servos:
        swipeStick = hardwareMap.get(Servo.class, "swipeStick");
        diffyLeft = hardwareMap.get(Servo.class, "diffyL");
        diffyRight = hardwareMap.get(Servo.class, "diffyR");
        deliveryClaw = hardwareMap.get(Servo.class, "dClaw");
        turret = hardwareMap.get(Servo.class, "turret");
        intakeArm = hardwareMap.get(Servo.class, "iArm");
        intakePivot = hardwareMap.get(Servo.class, "iPivot");
        intakeWrist = hardwareMap.get(Servo.class, "iWrist");
        intakeClaw = hardwareMap.get(Servo.class, "iClaw");

        // Lights:
        light = hardwareMap.get(RevBlinkinLedDriver.class, "light");

        // Limelight:
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightSampleDetector = new LimelightSampleDetector(limelight, LLConfig);


        // Update Constants

        VERTICAL_TICKS_PER_INCH = getVerticalSlidesTicksPerInch();
        VERTICAL_EXTEND_MAX_IN = getVerticalSlidesMaxIn();

    }



    ///////////////////* INIT *///////////////////

    public void teleOpInit() {
        setIntakeArmPos(V5IntakeArmTask.Position.BACK);
        setIntakeWristPos(V5IntakeWristTask.Position.TRANSFER);
        setIntakeClawPos(V5IntakeClawTask.Position.OPEN);
        setIntakePivotPos(V5IntakePivotTask.Position.VERTICAL);
        setDeliveryClawPos(V5DeliveryClawTask.Position.OPEN);
        setDiffyPos(V5DiffyTask.Position.FLAT);
    }
    public void autoInit() {
//        setIntakeArmPos(IntakeArmTask.Position.DOWN);
////        deliveryArm.setPosition(getDeliveryArmServoPosition(DeliveryArmTask.Position.CARRY));
//        setClawPosition(ClawTask.ClawPosition.OPEN);
        swipeStick.setPosition(SwipeStickTask.getTargetPos(SwipeStickTask.Position.IN));
    }

    public void autoClipInit() {
//        setIntakeArmPos(IntakeArmTask.Position.TRANSFER);
////        deliveryArm.setPosition(getDeliveryArmServoPosition(DeliveryArmTask.Position.FLAT));
//        setClawPosition(ClawTask.ClawPosition.CLOSE);
        swipeStick.setPosition(SwipeStickTask.getTargetPos(SwipeStickTask.Position.IN));
    }

    ///////////////////* UPDATES *///////////////////

    public void updateEverything() {
        updateEncoders();
//        updatePoseEstimate();
        updateProfilers();
        updatePIDs();
//        updateIntakeSampleState();
    }
    public void updatePIDs() {
        if(verticalSlidesRunToPosition) {
            setVerticalSlidePowerForPID(VERTICAL_SLIDES_PID_COEFFICIENTS.feedForward + verticalSlidesPID.getPower());
        }
    }
    public void updateEncoders() {
        verticalSlidesPos = verticalSlidesLeft.getCurrentPosition();
        horizontalSlidesPos = horizontalSlides.getCurrentPosition();
    }
    public void updateLastHorizontalEncoder() {
        lastHorizontalSlidesPos = horizontalSlidesPos;
    }
    public void updateProfilers() {

    }
    public abstract void updateIntakeSampleState();


    ///////////////////* GENERAL UTILS *///////////////////

    public void resetHorizontalAndVerticalSlidesEncoderPositions() {
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    ///////////////////* VERTICAL SLIDES UTILS *///////////////////

    public abstract double getVerticalSlidesTicksPerInch();
    public abstract double getVerticalSlidesMaxIn();
    public abstract PIDCoefficients getVerticalSlidesPidCoefficients();
    public void setVerticalSlidePowerForPID(double power) {
        verticalSlidesLeft.setPower(Utils.clamp(power, VERTICAL_SLIDE_DOWN_POWER_AUTO, 1.0));
        verticalSlidesRight.setPower(Utils.clamp(power, VERTICAL_SLIDE_DOWN_POWER_AUTO, 1.0));
    }
    public void setVerticalSlidePowerWithoutEncoder(double power) {
        verticalSlidesRunToPosition = false;
        verticalSlidesLeft.setPower(Utils.clamp(power, VERTICAL_SLIDE_DOWN_POWER_TELEOP, 1.0));
        verticalSlidesRight.setPower(Utils.clamp(power, VERTICAL_SLIDE_DOWN_POWER_TELEOP, 1.0));
    }
    public void setVerticalSlidesHeight(double posInches) {
        verticalSlidesRunToPosition = true;
        verticalSlidesPID.setTargetPosition((int) (posInches * VERTICAL_TICKS_PER_INCH));
    }
    public double getVerticalSlidesHeightInch() {
        return verticalSlidesPos/VERTICAL_TICKS_PER_INCH;
    }
    public int getVerticalSlidesHeightTicks() {
        return verticalSlidesLeft.getCurrentPosition();
    }


    ///////////////////* HORIZONTAL SLIDES UTILS *///////////////////

    public void setHorizontalSlidesPower(double power) {
        if(horizontalSlides.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            horizontalSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        horizontalSlides.setPower(power);
    }
    public void setHorizontalSlidesPowerWithoutEncoder(double power) {
        if(horizontalSlides.getMode() != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            horizontalSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        horizontalSlides.setPower(power);
    }
    public void brakeHorizontalSlides(double brakePower) {
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlides.setTargetPosition(lastHorizontalSlidesPos);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlides.setPower(brakePower);
    }
    public void setHorizontalSlideDist(double posInches, double power) {
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlides.setTargetPosition((int) (
                (Utils.clamp(posInches, HORIZONTAL_EXTEND_MIN_IN, HORIZONTAL_EXTEND_MAX_IN)
                        * HORIZONTAL_TICKS_PER_INCH)));
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlides.setPower(power);
    }
    public int getLastHorizontalSlideDist() {
        return lastHorizontalSlidesPos;
    }
    public double getHorizontalSlideDistInch() {
        return ((double) horizontalSlides.getCurrentPosition()) / HORIZONTAL_TICKS_PER_INCH;
    }
    public double getHorizontalSlideTransferPos() {
        return HORIZONTAL_SLIDE_TRANSFER_POS;
    }


    ///////////////////* DELIVERY UTILS *///////////////////




    ///////////////////* CLAW UTILS *///////////////////


    ///////////////////* INTAKE UTILS *///////////////////

    public abstract double getIntakeArmPosition(V5IntakeArmTask.Position position);
    public void setIntakeArmPos(V5IntakeArmTask.Position position) {
        intakeArm.setPosition(getIntakeArmPosition(position));
    }
    public abstract double getIntakePivotPosition(V5IntakePivotTask.Position position);
    public void setIntakePivotPos(V5IntakePivotTask.Position position) {
        intakePivot.setPosition(getIntakePivotPosition(position));
    }
    public abstract double getTurretPosition(V5TurretTask.Position position);
    public void setTurretPos(V5TurretTask.Position position) {
        turret.setPosition(getTurretPosition(position));
    }
    public abstract double getIntakeClawPosition(V5IntakeClawTask.Position position);
    public void setIntakeClawPos(V5IntakeClawTask.Position position) {
        intakeClaw.setPosition(getIntakeClawPosition(position));
    }
    public abstract double getIntakeWristPosition(V5IntakeWristTask.Position position);
    public void setIntakeWristPos(V5IntakeWristTask.Position position) {
        intakeWrist.setPosition(getIntakeWristPosition(position));
    }

    public abstract double getDeliveryClawPosition(V5DeliveryClawTask.Position position);
    public void setDeliveryClawPos(V5DeliveryClawTask.Position position) {
        deliveryClaw.setPosition(getDeliveryClawPosition(position));
    }

    ///////////////////* DIFFY UTILS *///////////////////

    public abstract double getDiffyTargetPos(V5DiffyTask.WhichDiffy placer, V5DiffyTask.Position position);
    public void setDiffyPos(V5DiffyTask.Position position) {
        V5DiffyTask.setHangPosition(this, V5DiffyTask.WhichDiffy.LEFT, position);
        V5DiffyTask.setHangPosition(this, V5DiffyTask.WhichDiffy.RIGHT, position);
    }

    ///////////////////* COLOR SENSOR + LIGHTS UTILS *///////////////////

//    public abstract boolean isSampleIn();
//    public void setLightColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
//        light.setPattern(pattern);
//    }
}
