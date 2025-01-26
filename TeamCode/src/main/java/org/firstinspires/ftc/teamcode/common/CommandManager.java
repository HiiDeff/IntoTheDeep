package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.util.GamePad;

public class CommandManager {

    public enum Command {
        UNSPECIFIED,
        CANCEL,
        DELIVERY_BUCKET,
        DELIVERY_CHAMBER,
        INTAKE_START,
        INTAKE_DOWN,
        INTAKE_UP,
        INTAKE_REVERSE,
        TRANSFER,
        TO_BUCKET_HIGH,
        TO_BUCKET_LOW,
        TO_CHAMBER_HIGH,
        TO_OBSERVATION_ZONE,

        HANG_RELEASE,
        HANG_RETRACT
    }
    private final GamePad gp1;
    private final GamePad gp2;

    private Command command = Command.UNSPECIFIED;
    private Command preValidCommand = Command.UNSPECIFIED;
    public CommandManager(GamePad gp1, GamePad gp2) {
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    // This should be called in every
    public void update() {
        if (command != Command.UNSPECIFIED) {
            preValidCommand = command;
        }
        command = Command.UNSPECIFIED;
        if (gp2.onceA()) {
            command = Command.INTAKE_START;
        } else if (gp2.onceB()) {
            command = Command.TRANSFER;
        } else if (gp2.onceDpadDown()) {
            command = Command.TO_BUCKET_LOW;
        } else if (gp2.onceDpadUp()) {
            command = Command.TO_BUCKET_HIGH;
        } else if (gp2.onceX()) {
            command = Command.DELIVERY_BUCKET;
        } else if (gp2.onceY()) {
            command = Command.CANCEL;
        }
    }

    public Command getCommand() {
        return command;
    }

    public Command getPreValidCommand() {
        return preValidCommand;
    }
}
