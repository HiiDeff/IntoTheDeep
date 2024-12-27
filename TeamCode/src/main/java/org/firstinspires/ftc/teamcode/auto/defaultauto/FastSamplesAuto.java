package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.V2Robot;

@Config
@Autonomous(name = "V2 Robot - FAST SAMPLE Auto")
public class FastSamplesAuto extends FastSamplesAutoPath{

    public static double Sample1IntakeAlignPosX = 25.2245;
    public static double Sample1IntakeAlignPosY = 27.9765;
    public static double Sample2IntakeAlignPosX = 26.2486;
    public static double Sample2IntakeAlignPosY = 14.2094;
    public static double Sample3IntakeAlignPosX = 27.5199;
    public static double Sample3IntakeAlignPosY = 22.2543;

    public static int INTAKE_DELAY = 1700;

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return new V2Robot(hardwareMap);
    }

    @Override
    protected Pose2d getPreloadBucketAlignPos() {
        return new Pose2d(7.2371, 23.5296, -Math.PI/4);
    }

    @Override
    protected Pose2d getPreloadBucketDeliverPos() {
        return new Pose2d(7.1296, 24.996, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample1IntakeAlignPos() {
        return new Pose2d(Sample1IntakeAlignPosX, Sample1IntakeAlignPosY, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample1IntakePos() {
        return new Pose2d(26.5173, 26.83326, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample1BucketAlignPos() {
        return new Pose2d(7.1296, 24.996, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample1BucketDeliverPos() {return new Pose2d(7.2371, 23.5296, -Math.PI/4);}

    @Override
    protected Pose2d getSample2IntakeAlignPos() {
        return new Pose2d(Sample2IntakeAlignPosX, Sample2IntakeAlignPosY, Math.PI/4);
    }

    @Override
    protected Pose2d getSample2IntakePos() {
        return new Pose2d(28.87, 16.8822, Math.PI/4);
    }

    @Override
    protected Pose2d getSample2BucketAlignPos() {
        return new Pose2d(7.1296, 24.996, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample2BucketDeliverPos() {
        return new Pose2d(7.1296, 24.996, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample3IntakeAlignPos() {
        return new Pose2d(Sample3IntakeAlignPosX, Sample3IntakeAlignPosY, Math.PI/4);
    }

    @Override
    protected Pose2d getSample3IntakePos() {
        return new Pose2d(29.1674, 24.2149, Math.PI/4);
    }

    @Override
    protected Pose2d getSample3BucketAlignPos() {
        return new Pose2d(7.1296, 24.996, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample3BucketDeliverPos() {
        return new Pose2d(7.1296, 24.996, -Math.PI/4);
    }

    @Override
    protected Pose2d getParkAlignPos() {
        return new Pose2d(52.5578, 1.3899, Math.PI/2);
    }

    @Override
    protected Pose2d getParkPos() {
        return new Pose2d(51.8927, -10.3844, Math.PI/2);
    }



    @Override
    protected int getIntakeDelay() {
        return INTAKE_DELAY;
    }
}
