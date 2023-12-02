package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
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
            int leftX = 0;
            int leftY = 0;
            int rightX = 0;
            int rightY = 0;
            if(gamepad1.left_stick_x > 0){
                if(gamepad1.left_stick_x < 0){
                    leftX += 1;
                }
            }
            if(gamepad1.left_stick_y > 0){
                if(gamepad1.left_stick_y < 0){
                    leftY += 1;
                }
            }
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
            if(gamepad1.left_stick_button){
                leftX = 0;
                leftY = 0;
            }

            telemetry.addData("", graph);
            telemetry.addData("LeftX", leftX);
            telemetry.addData("LeftY", leftY);
            //telemetry.addData("RightX", rightX);
            //telemetry.addData("RightY", rightY);
            telemetry.update();
        }

    }

}