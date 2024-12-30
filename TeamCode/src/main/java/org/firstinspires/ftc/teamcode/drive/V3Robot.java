package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.task.BucketTask;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.HangTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;

@Config
public class V3Robot extends RobotBase {

    // Bucket Servo Positions
    public static double CARRY_POS = 0.4,
            CHAMBER_DELIVER_POS = 0.70, //TODO: TUNE THIS
            DELIVER_POS = 0.71,
            TRANSFER_POS = 0.23,
            FLAT_POS = 0.58;

    // Profile Constraints
    public static ProfileConstraints BUCKET_CONSTRAINTS = new ProfileConstraints(10, 30, 20);


    // Intake Servo Positions
    public static double INTAKE_ARM_DOWN = 0.10,
            INTAKE_ARM_TRANSFER = 0.80,
            INTAKE_ARM_CLEAR = 0.40;

    // Claw Servo Positions
    public static double CLAW_OPEN_POS = 0.58,
            CLAW_CLOSE_POS = 0.35;

    // Hang Positions
    public static double LEFT_INIT_POS = 0.31;
    public static double LEFT_RELEASE_POS = 0.16;
    public static double LEFT_HANG_POS = 0.86;
    public static double RIGHT_INIT_POS = 0.7;
    public static double RIGHT_RELEASE_POS = 0.795;
    public static double RIGHT_HANG_POS = 0.1;


    // Vertical Slides Constants
    public static double VERTICAL_EXTEND_MIN_IN = -0.2;
    public static double VERTICAL_EXTEND_MAX_IN = 38.6;
    public static double VERTICAL_TICKS_PER_INCH = 1240.0/38.6;

    // Horizontal Slides Constants
    public static double HORIZONTAL_EXTEND_MAX_IN = 22.0;
    public static double HORIZONTAL_TICKS_PER_INCH = 31.8181818;
    public static double HORIZONTAL_SLIDE_TRANSFER_POS = 64 / HORIZONTAL_TICKS_PER_INCH;

    public final DcMotorEx verticalSlidesLeft, verticalSlidesRight;
    public final Servo leftHang, rightHang;

    // PIDF

    public static double PID_FEED_FORWARD = 0.16, PID_MIN_POWER = 0.3, PID_MAX_POWER = 1.0, PID_PK = 0.006, PID_PI = 0.0, PID_PD = 0.0;
    public final V3VerticalSlidesPID verticalSlidesPID;

    // States
    private boolean verticalSlidesRunToPosition = false;


    public V3Robot(HardwareMap hardwareMap) {
        super(hardwareMap);

        verticalSlidesLeft = hardwareMap.get(DcMotorEx.class, "vSlidesL");
        verticalSlidesLeft.setDirection(DcMotor.Direction.REVERSE);
        verticalSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlidesRight = hardwareMap.get(DcMotorEx.class, "vSlidesR");
        verticalSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftHang = hardwareMap.get(Servo.class, "lHang");
        rightHang = hardwareMap.get(Servo.class, "rHang");

        verticalSlidesPID = new V3VerticalSlidesPID(this, PID_MIN_POWER,
                                PID_MAX_POWER, PID_PK, PID_PI, PID_PD);

        setDriveParams();
        init(hardwareMap);
    }

    private void setDriveParams() {

        PinpointDrive.Params pinpointParams = PinpointDrive.PARAMS;
        pinpointParams.xOffset = -3.4;
        pinpointParams.yOffset = 1.5748;

        pinpointParams.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        pinpointParams.xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        pinpointParams.yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        MecanumDrive.Params p = MecanumDrive.PARAMS;
        p.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        p.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        p.inPerTick = 1.0;
        p.lateralInPerTick = 0.851106864; // p.inPerTick;
        p.trackWidthTicks = 14.585145842107918;

        // feedforward parameters (in tick units)
        p.kS = 1.581030305278766;
        p.kV = 0.1325846626043403;
        p.kA = 0.03;

        // path profile parameters (in inches)
        p.maxWheelVel = 60;
        p.minProfileAccel = -30; //TODO change this to -40
        p.maxProfileAccel = 50; //TODO change this to 60

        // turn profile parameters (in radians)
        p.maxAngVel = Math.PI; // shared with path
        p.maxAngAccel = Math.PI;

        // path controller gains
        p.axialGain = 5.0;
        p.lateralGain = 7.0;
        p.headingGain = 8.0; // shared with turn

        p.axialVelGain = 0.0;
        p.lateralVelGain = 0.0;
        p.headingVelGain = 0.0; // shared with turn

    }

    @Override
    public double getBucketServoPosition(BucketTask.Position position) {
        switch(position) {
            case CARRY:
                return CARRY_POS;
            case CHAMBER_DELIVER:
                return CHAMBER_DELIVER_POS;
            case DELIVER:
                return DELIVER_POS;
            case FLAT:
                return FLAT_POS;
            case TRANSFER:
            default:
                return TRANSFER_POS;
        }
    }

    @Override
    public double getIntakeArmPosition(IntakeArmTask.Position position) {
        switch(position) {
            case TRANSFER:
                return INTAKE_ARM_TRANSFER;
            case CLEAR:
                return INTAKE_ARM_CLEAR;
            case DOWN:
            default:
                return INTAKE_ARM_DOWN;
        }
    }

    @Override
    public double getClawPosition(ClawTask.ClawPosition position) {
        switch(position) {
            case CLOSE:
                return CLAW_CLOSE_POS;
            case OPEN:
            default:
                return CLAW_OPEN_POS;
        }
    }

    // Utils for vertical slides
    @Override
    public void setVerticalSlidePowerWithEncoder(double power) {
        verticalSlidesLeft.setPower(Utils.clamp(power, -0.3, 1.0));
        verticalSlidesRight.setPower(Utils.clamp(power, -0.3, 1.0));
    }
    @Override
    public void setVerticalSlidePowerWithoutEncoder(double power) {
        verticalSlidesRunToPosition = false;
        verticalSlidesLeft.setPower(Utils.clamp(power, -1.0, 1.0));
        verticalSlidesRight.setPower(Utils.clamp(power, -1.0, 1.0));
    }
    @Override
    public void setVerticalSlidesHeight(double posInches, double power) {
        verticalSlidesRunToPosition = true;
        verticalSlidesPID.setTargetPosition((int)(posInches*VERTICAL_TICKS_PER_INCH));
    }
    @Override
    public void updatePIDs() {
        if(verticalSlidesRunToPosition) {
            setVerticalSlidePowerWithEncoder(PID_FEED_FORWARD + verticalSlidesPID.getPower());
        }
    }


    @Override
    public double getVerticalSlidesHeightInch() {
        return verticalSlidesLeft.getCurrentPosition()/VERTICAL_TICKS_PER_INCH;
    }
    @Override
    public int getVerticalSlidesHeightTicks() {
        return verticalSlidesLeft.getCurrentPosition();
    }



    // Utils for horizontal slides
    @Override
    public boolean horizontalSlidesIsReversed() {
        return true;
    }
    @Override
    public double getHorizontalExtendMaxInConstants() {
        return HORIZONTAL_EXTEND_MAX_IN;
    }
    @Override
    public double getHorizontalTicksPerInchConstants() {
        return HORIZONTAL_TICKS_PER_INCH;
    }
    @Override
    public double getHorizontalTransferPos() {
        return HORIZONTAL_SLIDE_TRANSFER_POS;
    }

    @Override
    public ProfileConstraints getBucketServoProfileConstraints() {
        return BUCKET_CONSTRAINTS;
    }


    //Utils for hang

    public double getHangTargetPos(HangTask.WhichHang placer, HangTask.Position position) {
        switch (position) {
            case INIT:
                return placer == HangTask.WhichHang.LEFT ? LEFT_INIT_POS: RIGHT_INIT_POS;
            case RELEASE:
                return placer == HangTask.WhichHang.LEFT ? LEFT_RELEASE_POS : RIGHT_RELEASE_POS;
            case HANG:
            default:
                return placer == HangTask.WhichHang.LEFT ? LEFT_HANG_POS : RIGHT_HANG_POS;
        }
    }
    @Override
    public void initHang() {
        HangTask.setHangPosition(this, HangTask.WhichHang.LEFT, HangTask.Position.HANG);
        HangTask.setHangPosition(this, HangTask.WhichHang.RIGHT, HangTask.Position.HANG);
    }
    @Override
    public void releaseHang() {
        HangTask.setHangPosition(this, HangTask.WhichHang.LEFT, HangTask.Position.RELEASE);
        HangTask.setHangPosition(this, HangTask.WhichHang.RIGHT, HangTask.Position.RELEASE);
    }
    @Override
    public void lowerHang() {
        HangTask.setHangPosition(this, HangTask.WhichHang.LEFT, HangTask.Position.HANG);
        HangTask.setHangPosition(this, HangTask.WhichHang.RIGHT, HangTask.Position.HANG);
    }
}
