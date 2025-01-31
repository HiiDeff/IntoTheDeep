package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config

@Autonomous(name = "V3 CLIP Auto")
public class V3ClipAuto extends ClipAutoPath {

    public static double PRELOAD_DELIVER_X = -31, PRELOAD_DELIVER_Y = 3.0,
            CYCLE_DELIVER_X = -31.5, CYCLE_OFFSET_X = -0.2, CYCLE_OFFSET_Y = 3.25,
            CYCLE_DELIVER_Y = -14.0, PARK_X = -14.57, PARK_Y = 13.11, PARK_HEADING = 1.06,
            INTAKE_HEADING = 0.05, DELIVER_HEADING = -0.23, DELIVER_HEADING_OFFSET = 0.04;
    public static double THIRD_ANGLE = 1.905;


    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    } //TODO ask abt this

    @Override
    protected Pose2d getPreloadSubmersibleDeliverPos() {
        return new Pose2d(PRELOAD_DELIVER_X, PRELOAD_DELIVER_Y, 0.0);
    }

    @Override
    protected Pose2d getFirstSampleIntakePos() {
        return new Pose2d(-20.962, 17.441, 2.21);
    }

    @Override
    protected Pose2d getFirstSampleOuttakePos() {
        return new Pose2d(-20.962, 17.441, 0.8298);
    }

    @Override
    protected Pose2d getSecondSampleIntakePos() {
        return new Pose2d(-23.6228, 30.1771, 2.1792);
    }

    @Override
    protected Pose2d getSecondSampleOuttakePos() {
        return new Pose2d(-23.6228, 30.1771, 0.6202);
    }

    @Override
    protected Pose2d getThirdSampleIntakePos() {
        return new Pose2d(-22.6415, 37.7287, THIRD_ANGLE);
    }

    @Override
    protected Pose2d getThirdSampleOuttakePos() {
        return new Pose2d(-20.4855, 35.1955, 0.6121);
    }

    @Override
    protected Pose2d getSpecimenIntakePos(int cycleNumber) {
        return new Pose2d(1.5+0.00*(cycleNumber>1?1:0), 30.56, -Math.PI+INTAKE_HEADING); //x = 0.5+0.05*cycleNumber for V2 Robot
    }

    @Override
    protected Pose2d getSubmersibleDeliverPos(int cycleNumber) {
        return new Pose2d(CYCLE_DELIVER_X+cycleNumber*CYCLE_OFFSET_X, CYCLE_DELIVER_Y+cycleNumber*CYCLE_OFFSET_Y, DELIVER_HEADING+DELIVER_HEADING_OFFSET*cycleNumber);
    }

    @Override
    protected Pose2d getParkPos() {
        return new Pose2d(PARK_X, PARK_Y, PARK_HEADING);
    }
}
