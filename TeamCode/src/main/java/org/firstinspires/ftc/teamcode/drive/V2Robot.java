package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.task.DiffyIntakeTask;

public class V2Robot extends V2RobotBase {


    // Intake Diffy Arm Positions
    public static DiffyConstants DELIVER_POS = new DiffyConstants(0.505, 0.503);
    public static DiffyConstants CARRY_POS = new DiffyConstants(0.54, 0.465);
    public static DiffyConstants TRANSFER_POS = new DiffyConstants(0.559, 0.443);
    public static double INTAKE_ARM_DOWN = 0.95, INTAKE_ARM_TRANSFER = 0.13, INTAKE_ARM_CLEAR = 0.5; //change vals in base too TODO: make tasks for abstract functions
    public static int INTAKE_ARM_RPM = 50, INTAKE_ARM_RANGE = 300;
    public static double CLAW_OPEN = 0.77, CLAW_CLOSE = 0.62;
    public V2Robot(HardwareMap hardwareMap) {
        super(hardwareMap);
        //setDriveParams();
        //init(hardwareMap, MecanumDrive.LocalizerMode.DRIVE_ENCODER);
    }


    private void setDriveParams() {
        MecanumDrive.Params p = MecanumDrive.PARAMS;
        p.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        p.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        p.inPerTick = 0.002955;
        p.lateralInPerTick = 0.0020686; // p.inPerTick;
        p.trackWidthTicks = 4438;

        // feedforward parameters (in tick units)
        p.kS = 0.85203712;
        p.kV = 0.000433642;
        p.kA = 0.00008;

        // path profile parameters (in inches)
        p.maxWheelVel = 60;
        p.minProfileAccel = -40;
        p.maxProfileAccel = 60;

        // turn profile parameters (in radians)
        p.maxAngVel = Math.PI; // shared with path
        p.maxAngAccel = Math.PI;

        // path controller gains
        p.axialGain = 5.0;
        p.lateralGain = 5.0;
        p.headingGain = 3.0; // shared with turn

        p.axialVelGain = 0.0;
        p.lateralVelGain = 0.0;
        p.headingVelGain = 0.0; // shared with turn

        TwoDeadWheelLocalizer.Params localizerParams = TwoDeadWheelLocalizer.PARAMS;
        localizerParams.perpXTicks = -0.375;
        localizerParams.parYTicks = 0.0;
    }

    @Override
    public DiffyConstants getDiffyPosition(DiffyPosition position) {
        switch(position) {
            case DELIVER:
                return DELIVER_POS;
            case CARRY:
                return CARRY_POS;
            case TRANSFER:
            default:
                return TRANSFER_POS;
        }
    }
}
