package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

@Config
@Autonomous(name = "V2 Robot - SAMPLE Auto")
public class SamplesAuto extends SamplesAutoPath {

    public static double Sample1IntakeAlignPosX = 38.463;
    public static double Sample1IntakeAlignPosY = 1.6317;
    public static double Sample2IntakeAlignPosX = 37.5016;
    public static double Sample2IntakeAlignPosY = 8.6623;
    public static double Sample3IntakeAlignPosX = 37.3329;
    public static double Sample3IntakeAlignPosY = 17.9128;

    public static int INTAKE_DELAY = 2000;

    @Override
    protected V2RobotBase createRobot(HardwareMap hardwareMap) {
        return new V2Robot(hardwareMap);
    }

    @Override
    protected Pose2d getPreloadBucketAlignPos() {
        return new Pose2d(5.9944, 24.2038, -Math.PI/4);
    }

    @Override
    protected Pose2d getPreloadBucketDeliverPos() {
        return new Pose2d(5.9944, 24.2038, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample1IntakeAlignPos() {
        return new Pose2d(Sample1IntakeAlignPosX, Sample1IntakeAlignPosY, Math.PI/2);
    }

    @Override
    protected Pose2d getSample1IntakePos() {
        return new Pose2d(37.0738, 6.5973, Math.PI/2);
    }

    @Override
    protected Pose2d getSample1BucketAlignPos() {
        return new Pose2d(6.6921, 23.5466, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample1BucketDeliverPos() {
        return new Pose2d(6.2944, 23.9038, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample2IntakeAlignPos() {
        return new Pose2d(Sample2IntakeAlignPosX, Sample2IntakeAlignPosY, Math.PI/2);
    }

    @Override
    protected Pose2d getSample2IntakePos() {
        return new Pose2d(35.9134, 13.2687, Math.PI/2);
    }

    @Override
    protected Pose2d getSample2BucketAlignPos() {
        return new Pose2d(6.6921, 23.5466, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample2BucketDeliverPos() {
        return new Pose2d(6.2944, 23.9038, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample3IntakeAlignPos() {
        return new Pose2d(Sample3IntakeAlignPosX, Sample3IntakeAlignPosY, Math.PI/2);
    }

    @Override
    protected Pose2d getSample3IntakePos() {
        return new Pose2d(35.0852, 18.4767, Math.PI/2);
    }

    @Override
    protected Pose2d getSample3BucketAlignPos() {
        return new Pose2d(6.6921, 23.5466, -Math.PI/4);
    }

    @Override
    protected Pose2d getSample3BucketDeliverPos() {
        return new Pose2d(6.2944, 23.9038, -Math.PI/4);
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
