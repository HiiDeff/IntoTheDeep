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
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.RungStickTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;

@Config
public class V2Robot extends RobotBase {

    // Bucket Servo Positions
    public static double CARRY_POS = 0.7,
            CHAMBER_DELIVER_POS = 0.87,
            DELIVER_POS = 0.9,
            TRANSFER_POS = 0.62, // TODO : make this flatter
            FLAT_POS = 0.79;

    // Profile Constraints
    public static ProfileConstraints BUCKET_CONSTRAINTS = new ProfileConstraints(10, 8, 5);

    // Intake Servo Positions
    public static double INTAKE_ARM_DOWN = 0.69,
            INTAKE_ARM_TRANSFER = 0.08,
            INTAKE_ARM_CLEAR = 0.38;

    // Claw Servo Positions
    public static double CLAW_OPEN_POS = 0.5,
            CLAW_CLOSE_POS = 0.3;

    // Vertical Slides Constants
    public static double VERTICAL_EXTEND_MIN_IN = -0.2;
    public static double VERTICAL_EXTEND_MAX_IN = 40;
    public static double VERTICAL_TICKS_PER_INCH = 32.91 * 14 / 10; // 14 / 10 gear ratio

    // Horizontal Slides Constants
    public static double HORIZONTAL_EXTEND_MAX_IN = 19.0;
    public static double HORIZONTAL_TICKS_PER_INCH = 32.91;
    public static double HORIZONTAL_SLIDE_TRANSFER_POS = 172 / 32.91;



    // V2 Robot Unique Hardware
    public final DcMotorEx verticalSlides;
    public final Servo rungStick;

    public V2Robot(HardwareMap hardwareMap) {
        super(hardwareMap);

        verticalSlides = hardwareMap.get(DcMotorEx.class, "verticalSlides");
        verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rungStick = hardwareMap.get(Servo.class, "rungStick");

        setDriveParams();
        init(hardwareMap);
    }

    private void setDriveParams() {

        PinpointDrive.Params pinpointParams = PinpointDrive.PARAMS;
        pinpointParams.xOffset = -1.1875;
        pinpointParams.yOffset = -2.6250;

        pinpointParams.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        pinpointParams.xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        pinpointParams.yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        MecanumDrive.Params p = MecanumDrive.PARAMS;
        p.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        p.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        p.inPerTick = 1.0;
        p.lateralInPerTick = 0.856566; // p.inPerTick;
        p.trackWidthTicks = 13.798627694384;

        // feedforward parameters (in tick units)
        p.kS = 1.2252550810847085;
        p.kV = 0.13684870806505475;
        p.kA = 0.03;

        // path profile parameters (in inches)
        p.maxWheelVel = 60;
        p.minProfileAccel = -30;
        p.maxProfileAccel = 50;

        // turn profile parameters (in radians)
        p.maxAngVel = Math.PI; // shared with path
        p.maxAngAccel = Math.PI;

        // path controller gains
        p.axialGain = 5.0;
        p.lateralGain = 6.0;
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
        if(verticalSlides.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            verticalSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        verticalSlides.setPower(Utils.clamp(power, -1.0, 1.0));
    }
    @Override
    public void setVerticalSlidePowerWithoutEncoder(double power) {
        if(verticalSlides.getMode() != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            verticalSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        verticalSlides.setPower(Utils.clamp(power, -1.0, 1.0));
    }
    @Override
    public void setVerticalSlidesHeight(double posInches, double power) {
        verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlides.setTargetPosition((int) (
                (Utils.clamp(posInches, VERTICAL_EXTEND_MIN_IN, VERTICAL_EXTEND_MAX_IN)
                        * VERTICAL_TICKS_PER_INCH)));
        verticalSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        verticalSlides.setPower(power);
    }
    @Override
    public double getVerticalSlidesHeightInch() {
        return (double) verticalSlides.getCurrentPosition() /VERTICAL_TICKS_PER_INCH;
    }
    @Override
    public int getVerticalSlidesHeightTicks() {
        return verticalSlides.getCurrentPosition();
    }


    //Utils for horizontal Slides
    @Override
    public boolean horizontalSlidesIsReversed() {
        return false;
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
    @Override
    public void initHang() {
        RungStickTask.setPosition(this, RungStickTask.Position.DOWN);
    }
    @Override
    public void releaseHang() {
        RungStickTask.setPosition(this, RungStickTask.Position.UP);
    }
    @Override
    public void lowerHang() {
        RungStickTask.setPosition(this, RungStickTask.Position.DOWN);
    }
    @Override
    public void updatePIDs() {}

}
