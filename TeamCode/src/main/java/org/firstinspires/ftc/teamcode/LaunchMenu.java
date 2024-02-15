package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LaunchMenu.State.BACKDROP;
import static org.firstinspires.ftc.teamcode.LaunchMenu.State.DELAY;
import static org.firstinspires.ftc.teamcode.LaunchMenu.State.IDLE;
import static org.firstinspires.ftc.teamcode.LaunchMenu.State.OBJECTIVES;
import static org.firstinspires.ftc.teamcode.LaunchMenu.State.PARK;
import static org.firstinspires.ftc.teamcode.LaunchMenu.State.ALLIANCE;
import static org.firstinspires.ftc.teamcode.LaunchMenu.State.CLOSE;
import static org.firstinspires.ftc.teamcode.LaunchMenu.State.STACK;

import static bucketbrigade.casperlibrary.Objectives.PURPLE;
import static bucketbrigade.casperlibrary.Objectives.PURPLE_YELLOW;
import static bucketbrigade.casperlibrary.Objectives.PURPLE_YELLOW_WHITE;
import static bucketbrigade.casperlibrary.RobotRoutes.DEFAULT_GRAB_STACK_X;
import static bucketbrigade.casperlibrary.RobotRoutes.DEFAULT_GRAB_STACK_Y;
import static bucketbrigade.casperlibrary.RobotRoutes.DEFAULT_PLACE_BACKDROP_X;
import static bucketbrigade.casperlibrary.RobotRoutes.DEFAULT_PLACE_BACKDROP_Y;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import bucketbrigade.casperlibrary.Objectives;

public class LaunchMenu {

    enum State {ALLIANCE, CLOSE, PARK, DELAY, OBJECTIVES, BACKDROP, STACK, IDLE}

    private static final int MINIMUM_DELAY = 0;
    private static final int MAXIMUM_DELAY = 30;
    private static final double POSITION_INCREMENT = 0.5;

    public int delay;
    public double grabStackX = DEFAULT_GRAB_STACK_X;
    public double grabStackY = DEFAULT_GRAB_STACK_Y;
    public Objectives objectives;
    public boolean parkLeft;
    public double placeBackdropX = DEFAULT_PLACE_BACKDROP_X;
    public double placeBackdropY = DEFAULT_PLACE_BACKDROP_Y;
    public boolean redAlliance;
    public boolean startClose;

    private Gamepad currentGamepad = new Gamepad();
    private LinearOpMode opMode;
    private Gamepad previousGamepad = new Gamepad();

    private State state = ALLIANCE;

    public LaunchMenu(LinearOpMode opMode) throws InterruptedException {

        this.opMode = opMode;

    }

    public void update() {

        Gamepad gamepad1 = opMode.gamepad1;

        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        switch(state) {
            case ALLIANCE:
                prompt("Alliance", "X = blue, B = red");
                if (currentGamepad.x && !previousGamepad.x) {
                    redAlliance = false;
                    state = CLOSE;
                }
                else if (currentGamepad.b && !previousGamepad.b) {
                    redAlliance = true;
                    state = CLOSE;
                }
                break;
            case CLOSE:
                prompt("Start", "X = close, B = far");
                if (currentGamepad.x && !previousGamepad.x) {
                    startClose = true;
                    state = PARK;
                }
                else if (currentGamepad.b && !previousGamepad.b) {
                    startClose = false;
                    state = PARK;
                }
                break;
            case PARK:
                prompt("Park", "X = left, B = right");
                if (currentGamepad.x && !previousGamepad.x) {
                    parkLeft = true;
                    state = OBJECTIVES;
                }
                else if (currentGamepad.b && !previousGamepad.b) {
                    parkLeft = false;
                    state = OBJECTIVES;
                }
                break;
            case OBJECTIVES:
                prompt("Objectives", "X = purple/yellow/white, A = purple/yellow, B = purple");
                if (currentGamepad.x && !previousGamepad.x) {
                    objectives = PURPLE_YELLOW_WHITE;
                    state = DELAY;
                }
                else if (currentGamepad.a && !previousGamepad.a) {
                    objectives = PURPLE_YELLOW;
                    state = DELAY;
                }
                else if (currentGamepad.b && !previousGamepad.b) {
                    objectives = PURPLE;
                    state = DELAY;
                }
                break;
            case DELAY:
                prompt("Delay = " + delay, "X = ok, dpad = adjust");
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    delay = Math.min(delay + 1, MAXIMUM_DELAY);
                }
                else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                    delay = Math.max(delay - 1, MINIMUM_DELAY);
                }
                else if (currentGamepad.x && !previousGamepad.x) {
                    state = BACKDROP;
                }
                break;
            case BACKDROP:
                String backdropCaption = "Backdrop = (" + placeBackdropX + ", " + placeBackdropY + ")";
                prompt(backdropCaption, "X = ok, dpad = adjust");
                if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                    placeBackdropX -= POSITION_INCREMENT;
                }
                else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                    placeBackdropY -= POSITION_INCREMENT;
                }
                else if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                    placeBackdropX += POSITION_INCREMENT;
                }
                else if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    placeBackdropY += POSITION_INCREMENT;
                }
                else if (currentGamepad.x && !previousGamepad.x) {
                    state = STACK;
                }
                break;
            case STACK:
                String stackCaption = "Stack = (" + grabStackX + ", " + grabStackY + ")";
                prompt(stackCaption, "X = ok, dpad = adjust");
                if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                    grabStackX -= POSITION_INCREMENT;
                }
                else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                    grabStackY -= POSITION_INCREMENT;
                }
                else if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                    grabStackX += POSITION_INCREMENT;
                }
                else if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    grabStackY += POSITION_INCREMENT;
                }
                else if (currentGamepad.x && !previousGamepad.x) {
                    state = IDLE;
                }
                break;
            case IDLE:
                break;
            default:
                break;
        }
    }

    private void prompt(String caption, String value) {
        Telemetry telemetry = opMode.telemetry;
        telemetry.addData(caption, value);
        telemetry.update();
    }

    public boolean isActive() {
        return state != IDLE;
    }

    public void addTelemetry() {
        Telemetry telemetry = opMode.telemetry;
        telemetry.addData("Alliance", redAlliance ? "Red" : "Blue");
        telemetry.addData("Start", startClose ? "Close" : "Far");
        telemetry.addData("Park", parkLeft ? "Left" : "Right");
        telemetry.addData("Objectives", objectives);
        telemetry.addData("Delay", delay);
        telemetry.addData("Backdrop", placeBackdropX + ", " + placeBackdropY);
        telemetry.addData("Stack", grabStackX + ", " + grabStackY);
    }

}
