package org.firstinspires.ftc.teamcode.drive_rr;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

@Config
public class TestRobotPinpoint extends PinpointDrive {

    public static PinpointDrive.Params PINPOINT_PARAMS =
            new PinpointDrive.Params();

    public static MecanumDrive.Params TRUE_PARAMS = new MecanumDrive.Params();

    public TestRobotPinpoint(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        setDriveParams();
        super.init(hardwareMap);
    }

    public static void setDriveParams() {
        TRUE_PARAMS.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        TRUE_PARAMS.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        TRUE_PARAMS.inPerTick = 0.00198392652123995407577497129736;
        TRUE_PARAMS.lateralInPerTick = 0.0013201097409470458; //0.0013275715492669714; // 0.0016824700252845181;
        TRUE_PARAMS.trackWidthTicks = 6999.207445758258;

        // feedforward parameters (in tick units)
        TRUE_PARAMS.kS = 1.201743834589251;
        TRUE_PARAMS.kV = 0.0002683471418159556;
        TRUE_PARAMS.kA = 0.000045;

        // path profile parameters (in inches)
        TRUE_PARAMS.maxWheelVel = 50;
        TRUE_PARAMS.minProfileAccel = -30;
        TRUE_PARAMS.maxProfileAccel = 50;

        // turn profile parameters (in radians)
        TRUE_PARAMS.maxAngVel = Math.PI; // shared with path
        TRUE_PARAMS.maxAngAccel = Math.PI;

        // path controller gains
        TRUE_PARAMS.axialGain = 5.0;
        TRUE_PARAMS.lateralGain = 8.0; //10
        TRUE_PARAMS.headingGain = 8.0; // shared with turn //4

        TRUE_PARAMS.axialVelGain = 0.0;
        TRUE_PARAMS.lateralVelGain = 0.0;
        TRUE_PARAMS.headingVelGain = 0.0; // shared with turn

        MecanumDrive.PARAMS = TRUE_PARAMS;



        PINPOINT_PARAMS.xOffset = -3.3071;
        PINPOINT_PARAMS.yOffset = -6.6142;

        PINPOINT_PARAMS.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        PINPOINT_PARAMS.xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PINPOINT_PARAMS.yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        PinpointDrive.PARAMS = PINPOINT_PARAMS;
    }
}
