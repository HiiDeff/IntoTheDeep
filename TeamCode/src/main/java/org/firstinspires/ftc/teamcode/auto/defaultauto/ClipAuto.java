package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.V2Robot;
import org.firstinspires.ftc.teamcode.drive.V2RobotBase;

@Autonomous(name = "V2 Robot - Clip Auto")
public class ClipAuto extends ClipAutoPath {


    @Override
    protected V2RobotBase createRobot(HardwareMap hardwareMap) {
        return new V2Robot(hardwareMap);
    }

    @Override
    protected Pose2d getPreloadSubmersibleDeliverPos() {
        return new Pose2d(-30.99, -3.65, 0);
    }

    @Override
    protected Pose2d getFirstSampleIntakePos() {
        return new Pose2d(-27.65, 19.79, 2.04256);
    }

    @Override
    protected Pose2d getFirstSampleOuttakePos() {
        return new Pose2d(-27.65, 19.79, 0.87266);
    }

    @Override
    protected Pose2d getSecondSampleIntakePos() {
        return new Pose2d(-25.42, 29.29, 2.09439);
    }

    @Override
    protected Pose2d getSecondSampleOuttakePos() {
        return new Pose2d(-25.42, 29.29, Math.PI/4);
    }

    @Override
    protected Pose2d getThirdSampleIntakePos() {
        return new Pose2d(-18.60, 37.46, 2.30255);
    }

    @Override
    protected Pose2d getThirdSampleOuttakePos() {
        return new Pose2d(-18.60, 38.46, 0.69813);
    }

    @Override
    protected Pose2d getSpecimenIntakePos() {
        return new Pose2d(1.0, 30.56, -Math.PI);
    }

    @Override
    protected Pose2d getSubmersibleDeliverPos(int cycleNumber) {
        return new Pose2d(-31.31, -2-cycleNumber*2, -0.001);
    }

    @Override
    protected Pose2d getParkPos() {
        return new Pose2d(-8.97, 37.65, Math.PI/3);
    }
}
