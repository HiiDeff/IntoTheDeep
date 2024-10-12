package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

public class LM1Robot extends RobotBase {


    // Intake Diffy Arm Positions
    public static DiffyPosition INTAKE_LEFT = new DiffyPosition(1.0, 0.6);
    public static DiffyPosition INTAKE_CENTER = new DiffyPosition(0.87, 0.47);
    public static DiffyPosition INTAKE_RIGHT = new DiffyPosition(0.73, 0.33);
    public static DiffyPosition INTAKE_UP = new DiffyPosition(0.53, 0.85);

    public LM1Robot(HardwareMap hardwareMap) {
        super(hardwareMap);
        setDriveParams();
        //init(hardwareMap, MecanumDrive.LocalizerMode.TWO_DEAD_WHEEL);
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

    public static class DiffyPosition {
        double left, right;
        public DiffyPosition(double a, double b){
            this.left = a;
            this.right = b;
        }
    }
}
