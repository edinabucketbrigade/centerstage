package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// Ramps up a value over time.
public class Ramp {

    // Duration in milliseconds
    private double durationMilliseconds;

    // Timer
    private ElapsedTime timer = new ElapsedTime();

    // Initializes this.
    public Ramp(double durationMilliseconds) throws InterruptedException {

        // If the duration is not positive...
        if(durationMilliseconds <= 0) {

            // Complain.
            throw new InterruptedException("The duration must be positive.");

        }

        // Remember the duration.
        this.durationMilliseconds = durationMilliseconds;

    }

    // Resets this.
    public void reset() {

        // Reset the timer.
        timer.reset();

    }

    // Apply the
    public double apply(double input) {

        // Get the timer's milliseconds.
        double currentMilliseconds = timer.milliseconds();

        // Compute a fraction.
        double fraction = Math.min(currentMilliseconds / durationMilliseconds, 1);

        // Apply the fraction to the input.
        double output = input * fraction;

        // Return the result.
        return output;

    }

}