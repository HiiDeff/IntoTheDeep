package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.task.BucketTask;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
@Config
public class V2Robot extends V2RobotBase {

    // Bucket Servo Positions
    public static double CARRY_POS = 0.7,
            CHAMBER_DELIVER_POS = 0.88,
            DELIVER_POS = 0.9,
            TRANSFER_POS = 0.62,
            FLAT_POS = 0.78;

    // Intake Servo Positions
    public static double INTAKE_ARM_DOWN = 0.78,
            INTAKE_ARM_TRANSFER = 0.08,
            INTAKE_ARM_CLEAR = 0.42;

    // Claw Servo Positions
    public static double CLAW_OPEN_POS = 1.0,
            CLAW_CLOSE_POS = 0.86;


    public V2Robot(HardwareMap hardwareMap) {
        super(hardwareMap);
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
}
