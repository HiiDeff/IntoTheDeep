package org.firstinspires.ftc.teamcode.drive.V3Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.DeliveryArmTask;
import org.firstinspires.ftc.teamcode.task.HangTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.SwipeStickTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.profile.ProfileConstraints;

@Config
public class V3Robot extends RobotBase {


    ///////////////////* CONSTANTS *///////////////////


    // Delivery Servo Positions
    public static double CARRY_POS = 0.3,
            CHAMBER_DELIVER_POS = 0.70, //TODO: TUNE THIS
            DELIVER_POS = 0.80, //TODO: change?

            TRANSFER_POS = 0.23,
            FLAT_POS = 0.55;

    public static double VERTICAL_TICKS_PER_INCH  =  1240.0/38.6, VERTICAL_EXTEND_MAX_INCH = 38.6;

    // Profile Constraints
    public static ProfileConstraints DELIVERY_ARM_CONSTRAINTS = new ProfileConstraints(10, 12, 16);

    // Intake Servo Positions
    public static double INTAKE_ARM_DOWN = 0.14,
            INTAKE_ARM_TRANSFER = 0.83,
            INTAKE_ARM_CLEAR = 0.70,
            INTAKE_ARM_SLIGHTLY_UP = 0.25; //TODO: make this closer to transfer position

    // Claw Servo Positions
    public static double CLAW_OPEN_POS = 0.58,
            CLAW_CLOSE_POS = 0.40;

    // Hang Positions
    public static double LEFT_INIT_POS = 0.58;
    public static double LEFT_RELEASE_POS = 0.08;
    public static double LEFT_HANG_POS = 0.85;
    public static double LEFT_VERTICAL_POS = 0.25;
    public static double RIGHT_INIT_POS = 0.40;
    public static double RIGHT_RELEASE_POS = 0.875;
    public static double RIGHT_HANG_POS = 0.13;
    public static double RIGHT_VERTICAL_POS = 0.70;


    ///////////////////* HARDWARE *///////////////////

    // Sensors
    public final RevColorSensorV3 leftIntakeColorSensor, rightIntakeColorSensor;

    public final V3IntakeSampleState intakeSampleState;


    public V3Robot(HardwareMap hardwareMap) {
        super(hardwareMap);

        // Sensors:
        leftIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorLeft");
        rightIntakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorRight");

        // States:
        intakeSampleState = new V3IntakeSampleState(this);

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
    public double getDeliveryArmServoPosition(DeliveryArmTask.Position position) {
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
            case SLIGHTLY_UP:
                return INTAKE_ARM_SLIGHTLY_UP;
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
    @Override
    public ProfileConstraints getDeliveryArmServoProfileConstraints() {
        return DELIVERY_ARM_CONSTRAINTS;
    }
    @Override
    public double getHangTargetPos(HangTask.WhichHang placer, HangTask.Position position) {
        switch (position) {
            case VERTICAL:
                return placer == HangTask.WhichHang.LEFT ? LEFT_VERTICAL_POS : RIGHT_VERTICAL_POS;
            case INIT:
                return placer == HangTask.WhichHang.LEFT ? LEFT_INIT_POS: RIGHT_INIT_POS;
            case RELEASE:
                return placer == HangTask.WhichHang.LEFT ? LEFT_RELEASE_POS : RIGHT_RELEASE_POS;
            case HANG:
            default:
                return placer == HangTask.WhichHang.LEFT ? LEFT_HANG_POS : RIGHT_HANG_POS;
        }
    }
    // Update Constants
    @Override
    public double getVerticalSlidesTicksPerInch() {
        return VERTICAL_TICKS_PER_INCH;
    }
    @Override
    public double getVerticalSlidesMaxIn() {
        return VERTICAL_EXTEND_MAX_INCH;
    }
    // Color Sensor Utils
    @Override
    public void updateIntakeSampleState() {
        intakeSampleState.update();
    }
    @Override
    public boolean isSampleIn() {
        return intakeSampleState.IsSampleIn();
    }
    public String getIntakeColorToString() {
        return intakeSampleState.getStripLightColors();
    }

}
