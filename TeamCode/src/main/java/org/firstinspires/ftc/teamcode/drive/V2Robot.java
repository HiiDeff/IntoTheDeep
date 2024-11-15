package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
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
    public static double CARRY_POS = 0.595,
            CHAMBER_DELIVER_POS = 0.575,
            DELIVER_POS = 0.56,
            TRANSFER_POS = 0.62,
            FLAT_POS = 0.58;

    // Intake Servo Positions
    public static double INTAKE_ARM_DOWN = 0.91,
            INTAKE_ARM_TRANSFER = 0.13,
            INTAKE_ARM_CLEAR = 0.55;

    // Claw Servo Positions
    public static double CLAW_OPEN_POS = 0.77,
            CLAW_CLOSE_POS = 0.61;


    public V2Robot(HardwareMap hardwareMap) {
        super(hardwareMap);
        setDriveParams();
        init(hardwareMap);
    }

    private void setDriveParams() {

        PinpointDrive.Params pinpointParams = PinpointDrive.PARAMS;
        pinpointParams.xOffset = -3.3071;
        pinpointParams.yOffset = -6.6142;

        MecanumDrive.Params p = MecanumDrive.PARAMS;
        p.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        p.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        p.inPerTick = 0.00198392652123995407577497129736;
        p.lateralInPerTick = 0.0013201097409470458; // p.inPerTick;
        p.trackWidthTicks = 6999.207445758258;

        // feedforward parameters (in tick units)
        p.kS = 1.201743834589251;
        p.kV = 0.0002683471418159556;
        p.kA = 0.000045;

        // path profile parameters (in inches)
        p.maxWheelVel = 50;
        p.minProfileAccel = -30;
        p.maxProfileAccel = 50;

        // turn profile parameters (in radians)
        p.maxAngVel = Math.PI; // shared with path
        p.maxAngAccel = Math.PI;

        // path controller gains
        p.axialGain = 5.0;
        p.lateralGain = 5.0;
        p.headingGain = 8.0; // shared with turn

        p.axialVelGain = 0.0;
        p.lateralVelGain = 0.0;
        p.headingVelGain = 0.0; // shared with turn

//        TwoDeadWheelLocalizer.Params localizerParams = TwoDeadWheelLocalizer.PARAMS;
//        localizerParams.perpXTicks = -1166.809201791706;
//        localizerParams.parYTicks = 556.7862908481188;
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
