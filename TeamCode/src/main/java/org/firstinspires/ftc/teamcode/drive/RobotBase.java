package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.task.BucketTask;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.RungStickTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.util.profile.ServoProfiler;

@Config
public abstract class RobotBase extends PinpointDrive {

    // Horizontal Slides Constants
    public static double HORIZONTAL_EXTEND_MIN_IN = -0.05;
    private double HORIZONTAL_EXTEND_MAX_IN,
            HORIZONTAL_TICKS_PER_INCH,
            HORIZONTAL_SLIDE_TRANSFER_POS;

    // Intake Constants
    public static double INTAKE_POWER = 1.0, OUTTAKE_POWER = -0.9;

    // States


    // Common
    protected final HardwareMap hardwareMap;

    // Motors
    public final DcMotorEx horizontalSlides;
    public final DcMotorEx intakeRoller;


    // Servos
    public final Servo claw;
    public final Servo intakeArm;
    public final Servo bucketServo;
    public final ServoProfiler bucketProfiler;

    // Sensors
    public final RevColorSensorV3 leftIntakeColorSensor, rightIntakeColorSensor;
    public final IntakeSampleState intakeSampleState;

    // Lights
    private final RevBlinkinLedDriver light;

    // Cached Encoder Values
    private int verticalSlidesPos = 0;
    private int horizontalSlidesPos = 0;


    public RobotBase(HardwareMap hardwareMap) {

        super(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Motors:
        horizontalSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        if(horizontalSlidesIsReversed()) horizontalSlides.setDirection(DcMotor.Direction.REVERSE);
        horizontalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeRoller = hardwareMap.get(DcMotorEx.class, "iRoller");
        intakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servos:
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        claw = hardwareMap.get(Servo.class, "claw");
        intakeArm = hardwareMap.get(Servo.class, "iArm");

        bucketProfiler = new ServoProfiler(bucketServo, getBucketServoProfileConstraints());

        // Sensors:
        leftIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorLeft");
        rightIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorRight");

        // Lights:
        light = hardwareMap.get(RevBlinkinLedDriver.class, "light");

        // States:
        intakeSampleState = new IntakeSampleState(this);

        HORIZONTAL_EXTEND_MAX_IN = getHorizontalExtendMaxInConstants();
        HORIZONTAL_TICKS_PER_INCH = getHorizontalTicksPerInchConstants();
        HORIZONTAL_SLIDE_TRANSFER_POS = getHorizontalTransferPos();
    }

    public void teleOpInit() {
        setIntakeArmPos(IntakeArmTask.Position.DOWN);
        bucketServo.setPosition(getBucketServoPosition(BucketTask.Position.TRANSFER));
        initHang();
        setClawPosition(ClawTask.ClawPosition.OPEN);
    }
    public void autoInit() {
        setIntakeArmPos(IntakeArmTask.Position.DOWN);
        bucketServo.setPosition(getBucketServoPosition(BucketTask.Position.CARRY));
        initHang();
        setClawPosition(ClawTask.ClawPosition.OPEN);
    }

    public void autoClipInit() {
        setIntakeArmPos(IntakeArmTask.Position.TRANSFER);
        bucketServo.setPosition(getBucketServoPosition(BucketTask.Position.FLAT));
        initHang();
        setClawPosition(ClawTask.ClawPosition.CLOSE);
    }

    // Updates
    public void updateEverything() {
        updateEncoders();
        updateProfilers();
        updatePoseEstimate();
        updatePIDs();
        intakeSampleState.update();
    }

    public abstract void updatePIDs();

    public void updateEncoders() {
        verticalSlidesPos = getVerticalSlidesHeightTicks();
    }
    public void updateHorizontalEncoder() {
        horizontalSlidesPos = horizontalSlides.getCurrentPosition();
    }
    public void updateProfilers() {
        bucketProfiler.updatePosition();
    }


    // Utils


    // Utils for Vertical Slides

    public abstract void setVerticalSlidePowerWithEncoder(double power);
    public abstract void setVerticalSlidePowerWithoutEncoder(double power);
    public abstract void setVerticalSlidesHeight(double posInches, double power);
    public abstract double getVerticalSlidesHeightInch();
    public abstract int getVerticalSlidesHeightTicks();


    // Utils for Horizontal Slides

    public abstract boolean horizontalSlidesIsReversed();
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
        horizontalSlides.setTargetPosition(getLastHorizontalSlideDist());
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
        return horizontalSlidesPos;
    }

    public double getHorizontalSlideDist() {
        return ((double) horizontalSlides.getCurrentPosition()) / HORIZONTAL_TICKS_PER_INCH;
    }

    public abstract double getHorizontalExtendMaxInConstants();
    public abstract double getHorizontalTicksPerInchConstants();
    public abstract double getHorizontalTransferPos();


    // Utils for Bucket Servo

    public abstract ProfileConstraints getBucketServoProfileConstraints();
    public abstract double getBucketServoPosition(BucketTask.Position position);

    public void setBucketServoPosition(BucketTask.Position position) {
        bucketProfiler.setTargetPosition(getBucketServoPosition(position));
    }


    // Utils for Intake Arm
    public abstract double getIntakeArmPosition(IntakeArmTask.Position position);
    public void setIntakeArmPos(IntakeArmTask.Position position) {
        intakeArm.setPosition(getIntakeArmPosition(position));
    }


    // Utils for Roller Intake
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


    // Utils for claw
    public abstract double getClawPosition(ClawTask.ClawPosition position);
    public void setClawPosition(ClawTask.ClawPosition position) {
        claw.setPosition(getClawPosition(position));
    }


    // Utils for lights
    public void setLightColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
        light.setPattern(pattern);
    }
    public String getIntakeColorToString() {
        return intakeSampleState.getStripLightColors();
    }


    // Utils for hang
    public abstract void initHang();
    public abstract void releaseHang();
    public abstract void lowerHang();

}
