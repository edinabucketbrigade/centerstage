package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class TeleOpG extends LinearOpMode {
    // Declare servo positions (can be edited in FTC Dashboard)
    public static double LEFT_CLAW_MINIMUM = 0.275;
    public static double LEFT_CLAW_MAXIMUM = 0.6;
    public static double RIGHT_CLAW_MINIMUM = 0.7;
    public static double RIGHT_CLAW_MAXIMUM = 1;
    public static double WRIST_MINIMUM = 0.06;
    public static double WRIST_MAXIMUM = 0.93;
    public static int ARM_MINIMUM = 0;
    public static int ARM_MAXIMUM = 300;

    // Declare motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private Servo wristServo = null;
    private Servo leftClawServo = null;
    private Servo rightClawServo = null;

    @Override
    public void runOpMode() {
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");

        // Initialize hardware variables
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        // Wheel reversing (test for correct directions)
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousA = false;

        double leftClawPosition = LEFT_CLAW_MINIMUM;
        double rightClawPosition = RIGHT_CLAW_MINIMUM;
        double wristPosition = WRIST_MINIMUM;
        int armPosition = ARM_MINIMUM;

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
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


            if (currentX && !previousX) {
                leftClawPosition = leftClawPosition == LEFT_CLAW_MINIMUM ? LEFT_CLAW_MAXIMUM : LEFT_CLAW_MINIMUM;
                leftClawServo.setPosition(leftClawPosition);
            }
            if (currentB && !previousB) {
                rightClawPosition = rightClawPosition == RIGHT_CLAW_MINIMUM ? RIGHT_CLAW_MAXIMUM : RIGHT_CLAW_MINIMUM;
                rightClawServo.setPosition(rightClawPosition);
            }
            if (currentA && !previousA) {
                wristPosition = wristPosition == WRIST_MINIMUM ? WRIST_MAXIMUM : WRIST_MINIMUM;
                wristServo.setPosition(wristPosition);
            }
            if (currentY && !previousY) {
                armPosition = armPosition == ARM_MINIMUM ? ARM_MAXIMUM : ARM_MINIMUM;
                armMotor.setTargetPosition(armPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.1);
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Status", "Running");

            telemetry.addData("Run time", runtime);

            telemetry.addData("Front left/right power", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/right pwoer", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            telemetry.addData("Left claw position", leftClawServo.getPosition());
            telemetry.addData("Right claw position", rightClawServo.getPosition());
            telemetry.addData("Wrist target position", wristServo.getPosition());

            telemetry.update();

            previousB = currentB;
            previousX = currentX;
            previousY = currentY;
            previousA = currentA;
        }

        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}