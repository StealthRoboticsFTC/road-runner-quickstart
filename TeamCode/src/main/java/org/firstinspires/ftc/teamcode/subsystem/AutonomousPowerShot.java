package org.firstinspires.ftc.teamcode.subsystem;

/*
When a certain set of buttons are pressed, the robot will move from anywhere on the field to
a preset series of points on the field starting with a point where it will shoot one ring and knock
down the left power-shot goal, then straife to the right where it will shoot another ring and
knock down the middle power-shot goal, then straife to the right again where it will knock down the
power-shot goal, and then give control back to the drivers.

1. Envoke roadrunner code to move to left most point
2. Start up the shooter motors
3. Shoot one ring
4. Using roadrunner, straife to the middle point
5. Shoot one ring
6. Using roadrunner, straife to the right point
7. Shoot one ring
8. Turn off shooter motors
 */

/*
left: (-10.3522, 23.366)
middle: (-10.3522, 16.8147)
right: (-11.122, 7.264)
 */

public class AutonomousPowerShot {
    private Shooter shooter;
    private SampleMecanumDrive drive;

    public AutonomousPowerShot(Shooter shooter, SampleMecanumDrive drive) {
        this.shooter = shooter;
        this.drive = drive;
    }

    public void run() {
        //Do roadrunner here
        shooter.startRampUp();
        shooter.fire(1);
        //Roadrunner
        shooter.fire(1);
        //Roadrunner
        shooter.fire(1);
        shooter.stop();
    }

}