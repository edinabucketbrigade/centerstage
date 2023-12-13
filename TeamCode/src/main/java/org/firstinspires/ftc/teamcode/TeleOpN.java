package org.firstinspires.ftc.teamcode;

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
        int rightColumn = MINIMUM_COLUMN;
        int rightRow = MINIMUM_ROW;

        boolean isLeftXNeutral = true;
        boolean isRightXNeutral = true;
        boolean isLeftYNeutral = true;
        boolean isRightYNeutral = true;

        while (opModeIsActive()) {
            String graph = "  " + hexagons[0][0] + hexagons[1][0] + hexagons[2][0] + hexagons[3][0] + hexagons[4][0] + hexagons[5][0] + "\n" +
                    hexagons[0][1] + hexagons[1][1] + hexagons[2][1] + hexagons[3][1] + hexagons[4][1] + hexagons[5][1] + "\n" +
                    "  " + hexagons[0][2] + hexagons[1][2] + hexagons[2][2] + hexagons[3][2] + hexagons[4][2] + hexagons[5][2] + "\n" +
                    hexagons[0][1] + hexagons[1][1] + hexagons[2][1] + hexagons[3][1] + hexagons[4][1] + hexagons[5][1] + "\n";

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
            if(isLeftYNeutral && gamepad1.left_stick_y > THRESHOLD){
                leftRow = Math.max(leftRow - 1, MINIMUM_ROW);
                isLeftYNeutral = false;
            }
            if(isLeftYNeutral && gamepad1.left_stick_y < -THRESHOLD){
                leftRow = Math.min(leftRow + 1, MAXIMUM_ROW);
                isLeftYNeutral = false;
            }
            if(gamepad1.left_stick_y < THRESHOLD && gamepad1.left_stick_y > -THRESHOLD){
                isLeftYNeutral = true;
            }



            if(isRightXNeutral && gamepad1.right_stick_x > THRESHOLD){
                rightColumn = Math.min(rightColumn + 1, MAXIMUM_COLUMN);
                isRightXNeutral = false;
            }
            if(isRightXNeutral && gamepad1.right_stick_x < -THRESHOLD){
                rightColumn = Math.max(rightColumn - 1, MINIMUM_COLUMN);
                isRightXNeutral = false;
            }
            if(gamepad1.right_stick_x < THRESHOLD && gamepad1.right_stick_x > -THRESHOLD){
                isRightXNeutral = true;
            }
            if(isRightYNeutral && gamepad1.right_stick_y > THRESHOLD){
                rightRow = Math.max(rightRow - 1, MINIMUM_ROW);
                isRightYNeutral = false;
            }
            if(isRightYNeutral && gamepad1.right_stick_y < -THRESHOLD){
                rightRow = Math.min(rightRow + 1, MAXIMUM_ROW);
                isRightYNeutral = false;
            }
            if(gamepad1.right_stick_y < THRESHOLD && gamepad1.right_stick_y > -THRESHOLD){
                isRightYNeutral = true;
            }

            telemetry.addData("", graph);
            telemetry.addData("Left Column", leftColumn);
            telemetry.addData("Left Row", leftRow);
            telemetry.addData("Right Column", rightColumn);
            telemetry.addData("Right Row", rightRow);
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Left Stick X Neutral", isLeftXNeutral);
            telemetry.addData("Left Stick Y Neutral", isLeftYNeutral);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
            telemetry.addData("Right Stick X Neutral", isRightXNeutral);
            telemetry.addData("Right Stick Y Neutral", isRightYNeutral);
            telemetry.update();
        }

    }

}