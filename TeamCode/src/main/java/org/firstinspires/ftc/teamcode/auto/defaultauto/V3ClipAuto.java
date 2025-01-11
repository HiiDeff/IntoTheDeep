package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.V2Robot;

@Config

@Autonomous(name = "V3 CLIP Auto")
public class V3ClipAuto extends ClipAutoPath {

    public static double PRELOAD_DELIVER_X = -30.5, PRELOAD_DELIVER_Y = 3.0,
            CYCLE_DELIVER_X = -30.5, CYCLE_OFFSET_X = -0.0, CYCLE_OFFSET_Y = 2,
            CYCLE_DELIVER_Y = -11.0;


    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap); //new V2Robot(hardwareMap);
    } //TODO ask abt this

    @Override
    protected Pose2d getPreloadSubmersibleDeliverPos() {
        return new Pose2d(PRELOAD_DELIVER_X, PRELOAD_DELIVER_Y, 0.0);
    }

    @Override
    protected Pose2d getFirstSampleIntakePos() {
        return new Pose2d(-20.962, 17.441, 2.2208);
    }

    @Override
    protected Pose2d getFirstSampleOuttakePos() {
        return new Pose2d(-20.962, 17.441, 0.8298);
    }

    @Override
    protected Pose2d getSecondSampleIntakePos() {
        return new Pose2d(-23.6228, 30.1771, 2.2092);
    }

    @Override
    protected Pose2d getSecondSampleOuttakePos() {
        return new Pose2d(-23.6228, 30.1771, 0.6202);
    }

    @Override
    protected Pose2d getThirdSampleIntakePos() {
        return new Pose2d(-22.6415, 37.7287, 2.0);
    }

    @Override
    protected Pose2d getThirdSampleOuttakePos() {
        return new Pose2d(-20.4855, 35.1955, 0.6121);
    }

    @Override
    protected Pose2d getSpecimenIntakePos(int cycleNumber) {
        return new Pose2d(0.5+0.00*cycleNumber, 30.56, -Math.PI+0.05); //x = 0.5+0.05*cycleNumber for V2 Robot
    }

    @Override
    protected Pose2d getSubmersibleDeliverPos(int cycleNumber) {
        return new Pose2d(CYCLE_DELIVER_X+cycleNumber*CYCLE_OFFSET_X, CYCLE_DELIVER_Y+cycleNumber*CYCLE_OFFSET_Y, -0.05);
    }

    @Override
    protected Pose2d getParkPos() {
        return new Pose2d(-17.48, 16.55, 0.8561);
    }
}
