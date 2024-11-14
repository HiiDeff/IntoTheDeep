package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.task.V1DiffyTask;
import org.firstinspires.ftc.teamcode.task.V2DiffyTask;

public class LM1Robot extends RobotBase {


    // Intake Diffy Arm Positions
    public static double DOWN_LEFT_LEFT_POS = 1.0, DOWN_LEFT_RIGHT_POS = 0.6;
    public static double DOWN_CENTER_LEFT_POS = 0.87, DOWN_CENTER_RIGHT_POS = 0.47;
    public static double DOWN_RIGHT_LEFT_POS = 0.73, DOWN_RIGHT_RIGHT_POS = 0.33;
    public static double FORWARD_LEFT_POS = 0.53, FORWARD_RIGHT_POS = 0.85;
    public static double UP_LEFT_POS = 0.4, UP_RIGHT_POS = 0.98;

    public LM1Robot(HardwareMap hardwareMap) {
        super(hardwareMap);
        setDriveParams();
        //init(hardwareMap, MecanumDrive.LocalizerMode.TWO_DEAD_WHEEL);
        super.initWithLocalizer(hardwareMap, MecanumDrive.LocalizerMode.DRIVE_ENCODER);
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
    public double getDiffyTargetPos(V1DiffyTask.WhichDiffy servo, V1DiffyTask.DiffyPosition position) {
        switch(position) {
            case DOWN_LEFT:
                return servo == V1DiffyTask.WhichDiffy.LEFT ? DOWN_LEFT_LEFT_POS : DOWN_LEFT_RIGHT_POS;
            case DOWN_CENTER:
                return servo == V1DiffyTask.WhichDiffy.LEFT ? DOWN_CENTER_LEFT_POS : DOWN_CENTER_RIGHT_POS;
            case DOWN_RIGHT:
                return servo == V1DiffyTask.WhichDiffy.LEFT ? DOWN_RIGHT_LEFT_POS : DOWN_RIGHT_RIGHT_POS;
            case UP:
                return servo == V1DiffyTask.WhichDiffy.LEFT ? UP_LEFT_POS : UP_RIGHT_POS;
            case FORWARD:
            default:
                return servo == V1DiffyTask.WhichDiffy.LEFT ? FORWARD_LEFT_POS : FORWARD_RIGHT_POS;
        }
    }
}
