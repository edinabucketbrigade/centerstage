package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class TeleOpN extends LinearOpMode {

    private static final double THRESHOLD = 0.5;
    private static final int MINIMUM_COLUMN = 1;
    private static final int MAXIMUM_COLUMN = 6;
    private static final int MINIMUM_ROW = 1;
    private static final int MAXIMUM_ROW = 11;
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

        int leftColumn = MINIMUM_COLUMN;
        int leftRow = MINIMUM_ROW;

        boolean isLeftXNeutral = true;

        while (opModeIsActive()) {
            String graph = "  " + hexagons[0][0] + hexagons[1][0] + hexagons[2][0] + hexagons[3][0] + hexagons[4][0] + hexagons[5][0] + "\n" +
                    hexagons[0][1] + hexagons[1][1] + hexagons[2][1] + hexagons[3][1] + hexagons[4][1] + hexagons[5][1] + "\n" +
                    "  " + hexagons[0][2] + hexagons[1][2] + hexagons[2][2] + hexagons[3][2] + hexagons[4][2] + hexagons[5][2] + "\n" +
                    hexagons[0][1] + hexagons[1][1] + hexagons[2][1] + hexagons[3][1] + hexagons[4][1] + hexagons[5][1] + "\n"

                    ;

            int rightX = 0;
            int rightY = 0;
            if(isLeftXNeutral && gamepad1.left_stick_x > THRESHOLD){
                leftColumn = Math.min(leftColumn + 1, MAXIMUM_COLUMN);
                isLeftXNeutral = false;
            }
            if(isLeftXNeutral && gamepad1.left_stick_x < -THRESHOLD){
                leftColumn = Math.max(leftColumn - 1, MINIMUM_COLUMN);
                isLeftXNeutral = false;
            }
            if(gamepad1.left_stick_x < THRESHOLD && gamepad1.left_stick_x > -THRESHOLD){
                isLeftXNeutral = true;
            }
            /*if(gamepad1.left_stick_y > 0){
                if(gamepad1.left_stick_y < 0){
                    leftRow += 1;
                }
            }*/
            /*if(gamepad1.right_stick_x > 0){
                if(gamepad1.right_stick_x < 0){
                    rightX += 1;
                }
            }
            if(gamepad1.right_stick_y > 0){
                if(gamepad1.right_stick_y < 0){
                    rightY += 1;
                }
            }*/
            /*if(gamepad1.left_stick_button){
                leftColumn = MINIMUM_COLUMN;
                leftRow = MINIMUM_ROW;
            }*/

            telemetry.addData("", graph);
            telemetry.addData("Left Column", leftColumn);
            telemetry.addData("Left Row", leftRow);
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Left Stick X Neutral", isLeftXNeutral);
            //telemetry.addData("RightX", rightX);
            //telemetry.addData("RightY", rightY);
            telemetry.update();
        }

    }

}