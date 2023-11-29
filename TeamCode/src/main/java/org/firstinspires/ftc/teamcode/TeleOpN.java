package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TeleOpN extends LinearOpMode {

    @Override
    public void runOpMode() {
        String[][] hexagons = {
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡", "⬡"},
                {"⬡", "⬡", "⬡", "⬡", "⬡", "⬡"}
        };
        int selectedX = 0;
        int selectedY = 0;
        //hexagons[1][2] Outputs 7

        telemetry.addData("", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            String graph = "  " + hexagons[0][0] + hexagons[1][0] + hexagons[2][0] + hexagons[3][0] + hexagons[4][0] + hexagons[5][0] + "\n" +
                    hexagons[0][1] + hexagons[1][1] + hexagons[2][1] + hexagons[3][1] + hexagons[4][1] + hexagons[5][1] + "\n" +
                    "  " + hexagons[0][2] + hexagons[1][2] + hexagons[2][2] + hexagons[3][2] + hexagons[4][2] + hexagons[5][2] + "\n" +
                    hexagons[0][1] + hexagons[1][1] + hexagons[2][1] + hexagons[3][1] + hexagons[4][1] + hexagons[5][1] + "\n"

                    ;

            telemetry.addData("", graph);
            telemetry.update();
        }

    }

}