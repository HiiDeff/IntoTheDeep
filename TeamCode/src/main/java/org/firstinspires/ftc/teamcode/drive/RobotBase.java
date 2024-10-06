package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class RobotBase extends MecanumDrive {

    // Common
    protected final HardwareMap hardwareMap;

    public RobotBase(HardwareMap hardwareMap) {
        super(new Pose2d(0.0, 0.0, 0.0));
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


    }

    public void teleOpInit() {

    }
    public void autoInit() {

    }
    public void updateEverything() {
        updatePoseEstimate();
    }
}
