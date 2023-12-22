package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class TeleOpN extends LinearOpMode {

    private static final double THRESHOLD = 0.5;
    private static final int MINIMUM_COLUMN = 1;
    private static final int MAXIMUM_COLUMN_ODD_ROW = 6;
    private static final int MAXIMUM_COLUMN_EVEN_ROW = 7;
    private static final int MINIMUM_ROW = 1;
    private static final int MAXIMUM_ROW = 11;
    @Override
    public void runOpMode() {
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
            if(isLeftXNeutral && gamepad1.left_stick_x > THRESHOLD){
                int maximumColumn = getMaximumColumn(leftRow);
                leftColumn = Math.min(leftColumn + 1, maximumColumn);
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
                int maximumColumn = getMaximumColumn(rightRow);
                rightColumn = Math.min(rightColumn + 1, maximumColumn);
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
            if (!isEven(leftRow) && leftColumn == MAXIMUM_COLUMN_EVEN_ROW){
                leftColumn = MAXIMUM_COLUMN_ODD_ROW;
            }
            if (!isEven(rightRow) && rightColumn == MAXIMUM_COLUMN_EVEN_ROW){
                rightColumn = MAXIMUM_COLUMN_ODD_ROW;
            }

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
            String output = "\n";
            for(int row = MAXIMUM_ROW; row >= MINIMUM_ROW; row--) {
                if (!isEven(row)){
                    output += "  ";
                }
                int maximumColumn = getMaximumColumn(row);
                for (int column = MINIMUM_COLUMN; column <= maximumColumn; column++) {
                    if (column == leftColumn && leftRow == row) {
                        output += "Ⓛ";
                    } else if (column == rightColumn && rightRow == row){
                        output += "Ⓡ";
                    } else{
                        output += "〇";
                    }
                }
                output += "\n";
            }
            /*for(int row = MINIMUM_ROW; row <= MAXIMUM_ROW; row++){
                if (row == leftColumn && leftRow == 2){
                    leftRow2String += "⬢";
                } else {
                    leftRow2String += "⬡";
                }
            }*/
            telemetry.addData("output", output);
            /*if (leftColumn == 1){
                telemetry.addData("Left Column", "⬢⬡⬡⬡⬡⬡");
            } else if (leftColumn == 2){
                telemetry.addData("Left Column", "⬡⬢⬡⬡⬡⬡");
            } else if (leftColumn == 3){
                telemetry.addData("Left Column", "⬡⬡⬢⬡⬡⬡");
            } else if (leftColumn == 4){
                telemetry.addData("Left Column", "⬡⬡⬡⬢⬡⬡");
            } else if (leftColumn == 5){
                telemetry.addData("Left Column", "⬡⬡⬡⬡⬢⬡");
            } else if (leftColumn == 6){
                telemetry.addData("Left Column", "⬡⬡⬡⬡⬡⬢");
            }
            if (rightColumn == 1){
                telemetry.addData("Right Column", "⬢⬡⬡⬡⬡⬡");
            } else if (rightColumn == 2){
                telemetry.addData("Right Column", "⬡⬢⬡⬡⬡⬡");
            } else if (rightColumn == 3){
                telemetry.addData("Right Column", "⬡⬡⬢⬡⬡⬡");
            } else if (rightColumn == 4){
                telemetry.addData("Right Column", "⬡⬡⬡⬢⬡⬡");
            } else if (rightColumn == 5){
                telemetry.addData("Right Column", "⬡⬡⬡⬡⬢⬡");
            } else if (rightColumn == 6){
                telemetry.addData("Right Column", "⬡⬡⬡⬡⬡⬢");
            }*/
            telemetry.update();
        }

    }
    public int getMaximumColumn(int row){
        if (isEven(row)){
            return MAXIMUM_COLUMN_EVEN_ROW;
        } else {
            return MAXIMUM_COLUMN_ODD_ROW;
        }
    }
    public boolean isEven(int value){
        if (value%2 == 0){
            return true;
        } else {
            return false;
        }
    }
}