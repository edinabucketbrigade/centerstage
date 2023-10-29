package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class TeleOpG extends LinearOpMode {

    // Declare motors
    private ElapsedTime runtime = new ElapsedTime();

    private RobotHardwareA robotHardware = null;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotHardware = new RobotHardwareA(this);

        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousA = false;

        waitForStart();

        while (opModeIsActive()) {
            robotHardware.update();

            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentA = gamepad1.a;

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            //double liftPower  = gamepad1.x ? 1.0 : 0.0;
            //int liftPos  = lift.getCurrentPosition();

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            //lift.setPower(liftPower);

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.


            /*leftBackPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftFrontPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightBackPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightFrontPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad*/

            if (currentY && !previousY) {
                robotHardware.toggleArm();
                robotHardware.toggleWrist();
            }


            if (currentX && !previousX) {
                robotHardware.toggleRightClaw();
            }
            if (currentB && !previousB) {
                robotHardware.toggleLeftClaw();
            }

            // Send calculated power to wheels
            robotHardware.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            telemetry.addData("Status", "Running");

            telemetry.addData("Run time", runtime);

            telemetry.addData("Front left/right power", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/right power", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            telemetry.update();

            previousB = currentB;
            previousX = currentX;
            previousY = currentY;
            previousA = currentA;
        }

    }

}