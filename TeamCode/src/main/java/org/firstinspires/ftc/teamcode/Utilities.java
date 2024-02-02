package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Utilities {

    // Logs a message.
    public static void log(String message, Telemetry telemetry) {

        // Show the message.
        telemetry.addData("Message", message);
        telemetry.update();

    }

}
