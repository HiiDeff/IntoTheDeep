package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.task.ClawTask;
import org.firstinspires.ftc.teamcode.task.IntakeArmTask;
import org.firstinspires.ftc.teamcode.task.V2DiffyTask;
@Config
public class V2Robot extends V2RobotBase {


    // Intake Diffy Arm Positions
    public static double DELIVER_LEFT_POS = 0.505, DELIVER_RIGHT_POS = 0.503;
    public static double CARRY_LEFT_POS = 0.54, CARRY_RIGHT_POS = 0.42;
    public static double TRANSFER_LEFT_POS = 0.58, TRANSFER_RIGHT_POS = 0.41;
    public static double INTAKE_ARM_DOWN = 0.91, INTAKE_ARM_TRANSFER = 0.13, INTAKE_ARM_CLEAR = 0.55; //change vals in base too TODO: make tasks for abstract functions
    public static int INTAKE_ARM_RPM = 50, INTAKE_ARM_RANGE = 300;
    public static double CLAW_OPEN_POS = 0.77, CLAW_CLOSE_POS = 0.62;
    public V2Robot(HardwareMap hardwareMap) {
        super(hardwareMap);
        //setDriveParams();
        init(hardwareMap, MecanumDrive.LocalizerMode.DRIVE_ENCODER);
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
    public double getDiffyTargetPos(V2DiffyTask.WhichDiffy servo, V2DiffyTask.DiffyPosition position) {
        switch(position) {
            case DELIVER:
                return servo == V2DiffyTask.WhichDiffy.LEFT ? DELIVER_LEFT_POS : DELIVER_RIGHT_POS;
            case CARRY:
                return servo == V2DiffyTask.WhichDiffy.LEFT ? CARRY_LEFT_POS : CARRY_RIGHT_POS;
            case TRANSFER:
            default:
                return servo == V2DiffyTask.WhichDiffy.LEFT ? TRANSFER_LEFT_POS : TRANSFER_RIGHT_POS;
        }
    }

    @Override
    public double getIntakeArmPos(IntakeArmTask.Position position) {
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
