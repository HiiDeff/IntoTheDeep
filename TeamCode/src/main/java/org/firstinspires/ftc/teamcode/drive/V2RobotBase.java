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

@Config
public abstract class V2RobotBase extends PinpointDrive {


    // Vertical Slides Constants
    public static double VERTICAL_EXTEND_MIN_IN = -0.2;
    public static double VERTICAL_EXTEND_MAX_IN = 40;
    public static double VERTICAL_EXTEND_LOW_CHAMBER_IN = 10;
    public static double VERTICAL_EXTEND_HIGH_CHAMBER_IN = 17;
    public static double VERTICAL_TICKS_PER_INCH = 32.91 * 14 / 10; // 14 / 10 gear ratio

    // Horizontal Slides Constants
    public static double HORIZONTAL_EXTEND_MIN_IN = -0.05;
    public static double HORIZONTAL_EXTEND_MAX_IN = 19.0;
    public static double HORIZONTAL_TICKS_PER_INCH = 32.91;
    public static double HORIZONTAL_SLIDE_TRANSFER_POS = 172 / 32.91;

    // Intake Constants
    public static double INTAKE_POWER = 1.0, OUTTAKE_POWER = -0.9;

    // States


    // Common
    protected final HardwareMap hardwareMap;

    // Motors
    public final DcMotorEx verticalSlides;
    public final DcMotorEx horizontalSlides;
    public final DcMotorEx intakeRoller;


    // Servos
    public final Servo bucketServo;
    public final Servo claw;
    public final Servo intakeArm;
    public final Servo rungStick;

    // Sensors
    public final RevColorSensorV3 leftIntakeColorSensor, rightIntakeColorSensor;
    public final IntakeSampleState intakeSampleState;

    // Lights
    private final RevBlinkinLedDriver light;

    // Cached Encoder Values
    private int verticalSlidesPos = 0;
    private int horizontalSlidesPos = 0;
    private RungStickTask.Position rungStickPos = RungStickTask.Position.DOWN;


    public V2RobotBase(HardwareMap hardwareMap) {

        super(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Motors:
        verticalSlides = hardwareMap.get(DcMotorEx.class, "verticalSlides");
        verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizontalSlides = hardwareMap.get(DcMotorEx.class, "horizontalSlides");
        horizontalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeRoller = hardwareMap.get(DcMotorEx.class, "intakeRoller");
        intakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servos:
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        claw = hardwareMap.get(Servo.class, "claw");
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        rungStick = hardwareMap.get(Servo.class, "rungStick");

        // Sensors:
        leftIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorLeft");
        rightIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorRight");

        // Lights:
        light = hardwareMap.get(RevBlinkinLedDriver.class, "light");

        // States:
        intakeSampleState = new IntakeSampleState(this);
    }

    public void teleOpInit() {
        setIntakeArmPos(IntakeArmTask.Position.DOWN);
        setBucketServoPosition(BucketTask.Position.TRANSFER);
        RungStickTask.setPosition(this, RungStickTask.Position.DOWN);
    }
    public void autoInit() {
        setIntakeArmPos(IntakeArmTask.Position.TRANSFER);
        setBucketServoPosition(BucketTask.Position.FLAT);
        RungStickTask.setPosition(this, RungStickTask.Position.DOWN);
        setClawPosition(ClawTask.ClawPosition.CLOSE);
    }

    // Updates
    public void updateEverything() {
        updateEncoders();
        updatePoseEstimate();
        intakeSampleState.update();
    }

    public void updateEncoders() {
        verticalSlidesPos = verticalSlides.getCurrentPosition();
    }
    public void updateHorizontalEncoder() {
        horizontalSlidesPos = horizontalSlides.getCurrentPosition();
    }

    // Utils


    // Utils for Vertical Slides
    public void setVerticalSlidePowerWithEncoder(double power) {
        if(verticalSlides.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            verticalSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        verticalSlides.setPower(Utils.clamp(power, -1.0, 1.0));
    }

    public void setVerticalSlidePowerWithoutEncoder(double power) {
        if(verticalSlides.getMode() != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            verticalSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        verticalSlides.setPower(Utils.clamp(power, -1.0, 1.0));
    }

    public void setVerticalSlidesHeight(double posInches, double power) {
        verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlides.setTargetPosition((int) (
                (Utils.clamp(posInches, VERTICAL_EXTEND_MIN_IN, VERTICAL_EXTEND_MAX_IN)
                        * VERTICAL_TICKS_PER_INCH)));
        verticalSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        verticalSlides.setPower(power);
    }

    public double getVerticalSlidesHeight() {
        return (double) verticalSlidesPos /VERTICAL_TICKS_PER_INCH;
    }


    // Utils for Horizontal Slides
    public void setHorizontalSlidesPower(double power) {
        if(horizontalSlides.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            horizontalSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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


    // Utils for Bucket Servo
    public abstract double getBucketServoPosition(BucketTask.Position position);

    public void setBucketServoPosition(BucketTask.Position position) {
        bucketServo.setPosition(getBucketServoPosition(position));
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


    // Utils for rung stick
    public void toggleRungStick(){
        if (rungStickPos == RungStickTask.Position.UP) {
            rungStickPos = RungStickTask.Position.DOWN;
        }
        else{
            rungStickPos = RungStickTask.Position.UP;
        }
        RungStickTask.setPosition(this, rungStickPos);
    }
}
