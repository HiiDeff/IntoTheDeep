package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryArmTask;
import org.firstinspires.ftc.teamcode.task.HangTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.SwipeStickTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.util.profile.ServoProfiler;

@Config
public abstract class RobotBase extends PinpointDrive {


    ///////////////////* CONSTANTS *///////////////////

    // Vertical Slides Constants
    public static double VERTICAL_EXTEND_MIN_IN = -0.2, VERTICAL_EXTEND_MAX_IN;
    public static double VERTICAL_TICKS_PER_INCH;
    public static double VERTICAL_SLIDE_DOWN_POWER_TELEOP = -0.5, VERTICAL_SLIDE_DOWN_POWER_AUTO = -0.7;

    // Horizontal Slides Constants
    public static double HORIZONTAL_EXTEND_MIN_IN = -0.05, HORIZONTAL_EXTEND_MAX_IN = 22.0;
    public static double HORIZONTAL_TICKS_PER_INCH = 31.8181818;
    public static double HORIZONTAL_SLIDE_TRANSFER_POS = 64 / HORIZONTAL_TICKS_PER_INCH;

    // Intake Constants
    public static double INTAKE_POWER = 1.0, OUTTAKE_POWER = -0.6;



    ///////////////////* HARDWARE *///////////////////


    // Common
    protected final HardwareMap hardwareMap;

    // Motors
    public final DcMotorEx horizontalSlides;
    public final DcMotorEx verticalSlidesLeft, verticalSlidesRight;
    public final DcMotorEx intakeRoller;

    // Servos
    public final Servo claw;
    public final Servo intakeArm;
    public final Servo deliveryArm;
    public final Servo leftHang, rightHang;
    public final Servo swipeStick;

    // Motion PIDs + Profilers
    public static PIDCoefficients VERTICAL_SLIDES_PID_COEFFICIENTS; // = new PIDCoefficients(0.3, 1.0, 0.015, 0.0, 0.0,0.22); // new PIDCoefficients(0.3, 1.0, 0.006, 0.0, 0.0,0.16); //0.07 min ff, 0.37 = max ff, so do
    public final VerticalSlidesPID verticalSlidesPID;
    public final ServoProfiler deliveryArmProfiler;

    // Lights
    private final RevBlinkinLedDriver light;

    // Cached Encoder Values
    private int verticalSlidesPos = 0;
    private int horizontalSlidesPos = 0;
    private int lastHorizontalSlidesPos = 0;

    // States
    private boolean verticalSlidesRunToPosition = false;

    public boolean hangUp = false;


    public RobotBase(HardwareMap hardwareMap) {

        super(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Motors:
        horizontalSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        horizontalSlides.setDirection(DcMotor.Direction.REVERSE);
        horizontalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeRoller = hardwareMap.get(DcMotorEx.class, "iRoller");
        intakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalSlidesLeft = hardwareMap.get(DcMotorEx.class, "vSlidesL");
        verticalSlidesLeft.setDirection(DcMotor.Direction.REVERSE);
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight = hardwareMap.get(DcMotorEx.class, "vSlidesR");
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        VERTICAL_SLIDES_PID_COEFFICIENTS = getVerticalSlidesPidCoefficients();
        verticalSlidesPID = new VerticalSlidesPID(this, VERTICAL_SLIDES_PID_COEFFICIENTS);

        // Servos:
        swipeStick = hardwareMap.get(Servo.class, "swipeStick");
        deliveryArm = hardwareMap.get(Servo.class, "dArm");
        claw = hardwareMap.get(Servo.class, "claw");
        intakeArm = hardwareMap.get(Servo.class, "iArm");
        leftHang = hardwareMap.get(Servo.class, "lHang");
        rightHang = hardwareMap.get(Servo.class, "rHang");

        deliveryArmProfiler = new ServoProfiler(deliveryArm, getDeliveryArmServoProfileConstraints());

        // Lights:
        light = hardwareMap.get(RevBlinkinLedDriver.class, "light");

        // Update Constants

        VERTICAL_TICKS_PER_INCH = getVerticalSlidesTicksPerInch();
        VERTICAL_EXTEND_MAX_IN = getVerticalSlidesMaxIn();

    }



    ///////////////////* INIT *///////////////////

    public void teleOpInit() {
        setIntakeArmPos(IntakeArmTask.Position.DOWN);
        deliveryArm.setPosition(getDeliveryArmServoPosition(DeliveryArmTask.Position.CARRY));
//        initHang();
        setClawPosition(ClawTask.ClawPosition.OPEN);
        swipeStick.setPosition(SwipeStickTask.getTargetPos(SwipeStickTask.Position.IN));
    }
    public void autoInit() {
        setIntakeArmPos(IntakeArmTask.Position.DOWN);
        deliveryArm.setPosition(getDeliveryArmServoPosition(DeliveryArmTask.Position.CARRY));
        lowerHang();
        setClawPosition(ClawTask.ClawPosition.OPEN);
        swipeStick.setPosition(SwipeStickTask.getTargetPos(SwipeStickTask.Position.IN));
    }

    public void autoClipInit() {
        setIntakeArmPos(IntakeArmTask.Position.TRANSFER);
        deliveryArm.setPosition(getDeliveryArmServoPosition(DeliveryArmTask.Position.FLAT));
        lowerHang();
        setClawPosition(ClawTask.ClawPosition.CLOSE);
        swipeStick.setPosition(SwipeStickTask.getTargetPos(SwipeStickTask.Position.IN));
    }

    ///////////////////* UPDATES *///////////////////

    public void updateEverything() {
        updateEncoders();
        updatePoseEstimate();
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
        deliveryArmProfiler.updatePosition();
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

    public abstract ProfileConstraints getDeliveryArmServoProfileConstraints();
    public abstract double getDeliveryArmServoPosition(DeliveryArmTask.Position position);
    public void setDeliveryArmServoPosition(DeliveryArmTask.Position position) {
        deliveryArmProfiler.setTargetPosition(getDeliveryArmServoPosition(position));
    }


    ///////////////////* CLAW UTILS *///////////////////

    public abstract double getClawPosition(ClawTask.ClawPosition position);
    public void setClawPosition(ClawTask.ClawPosition position) {
        claw.setPosition(getClawPosition(position));
    }


    ///////////////////* INTAKE UTILS *///////////////////

    public abstract double getIntakeArmPosition(IntakeArmTask.Position position);
    public void setIntakeArmPos(IntakeArmTask.Position position) {
        intakeArm.setPosition(getIntakeArmPosition(position));
    }
    public void runIntakeIn() {
        intakeRoller.setPower(INTAKE_POWER);
    }
    public void runIntakeWithPower(double power) { intakeRoller.setPower(power);}
    public void runIntakeOut() {
        intakeRoller.setPower(OUTTAKE_POWER);
    }
    public void stopIntake() {
        intakeRoller.setPower(0.0);
    }


    ///////////////////* COLOR SENSOR + LIGHTS UTILS *///////////////////

    public abstract boolean isSampleIn();
    public void setLightColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
        light.setPattern(pattern);
    }

    ///////////////////* HANG UTILS *///////////////////

    public abstract double getHangTargetPos(HangTask.WhichHang placer, HangTask.Position position);
    public void initHang() {
        HangTask.setHangPosition(this, HangTask.WhichHang.LEFT, HangTask.Position.INIT);
        HangTask.setHangPosition(this, HangTask.WhichHang.RIGHT, HangTask.Position.INIT);
        hangUp = false;
    }
    public void releaseHang() {
        HangTask.setHangPosition(this, HangTask.WhichHang.LEFT, HangTask.Position.RELEASE);
        HangTask.setHangPosition(this, HangTask.WhichHang.RIGHT, HangTask.Position.RELEASE);
        hangUp = true;
    }
    public void lowerHang() {
        HangTask.setHangPosition(this, HangTask.WhichHang.LEFT, HangTask.Position.HANG);
        HangTask.setHangPosition(this, HangTask.WhichHang.RIGHT, HangTask.Position.HANG);
        hangUp = false;
    }
    public boolean isHangUp(){
        return hangUp;
    }
}
