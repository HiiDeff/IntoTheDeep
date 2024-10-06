package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive_rr.TestRobotDeadWheels;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class RobotFactory {
    public enum Robot {
        LM1_ROBOT,
        TEST_ROBOT;
    }
    public static Robot ACTIVE_ROBOT = Robot.LM1_ROBOT;

    public static MecanumDrive createRobot(HardwareMap hardwareMap) {
        switch (ACTIVE_ROBOT) {
            case TEST_ROBOT:
                return new TestRobotDeadWheels(hardwareMap);
            case LM1_ROBOT:
                return new LM1Robot(hardwareMap);
            default:
                return new LM1Robot(hardwareMap);
        }
    }
}
