package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Example;
import org.firstinspires.ftc.teamcode.HardwareMap.IMDriveBot;
import org.firstinspires.ftc.teamcode.HardwareMap.IMHardwareBot;

// CHAWKS: Name it something useful!
@TeleOp(name = "IM Op Mode TEST", group = "A")
//@Disabled
public class IconManiacsTeleOpTEST extends HardwareMap_Example {

    IMDriveBot bot = new IMDriveBot();
    double leftPower;
    double rightPower;
    double intakePower;
    double conveyorPower;
    double drive;
    double strafe;
    double rotate;
    double speed = 0.6;
    @Override
    public void runOpMode() {
        /*
            CHAWKS: On Driver Station, telemetry will be display!
                    How is this useful for debugging?
        */
        // Send telemetry message to Driver Station
        telemetry.addData("Status: ", "Hit [Init] to Initialize ze bot");    //
        telemetry.update();

        /*
            CHAWKS: Step 0. Initialize OUR ROBOT
        */
        // MUST HAVE THIS LINE BELOW
        bot.init(hardwareMap);

        // Send telemetry message to "Driver Station" signify robot waiting;
        telemetry.addData("Status: ", "Hit [PLAY] to start!");    //
        telemetry.update();

        /*
            CHAWKS: Step 1. Hit Play to run through the code!
        */
        // MUST HAVE THIS LINE BELOW
        waitForStart();

        /*
            CHAWKS: Remember opModeIsActive?! It's a loop!
        */
        // run until the end of the match (driver presses [STOP])
        // MUST HAVE!
        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;  // maps the joysticks to the motors respective of the sides
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
            bot.frontLeft.setPower((drive + strafe + rotate));
            bot.backRight.setPower((drive + strafe - rotate));
            bot.backLeft.setPower(drive - strafe + rotate);
            bot.frontRight.setPower(drive - strafe - rotate);

            if(gamepad2.right_trigger == 1){ // if the gamepad 2 right trigger gets pressed....
                bot.claw.setPosition(11.5);
                bot.claw2.setPosition(11.5);
            } else if(gamepad2.left_trigger == 1){ // if the gamepad 2 left trigger gets pressed...
                bot.claw.setPosition(0);
                bot.claw2.setPosition(0);
            }

            //bot.clawPOS = Range.clip(bot.clawPOS, bot.SERVO_LOW_RANGE, bot.SERVO_HIGH_RANGE); // sets the cap for the range of movement for the claw
            // this is the function that actually moves the servo
            // Show the elapsed game time and wheel power.
            telemetry.addData("G2:claw", "%.2f", bot.clawPOS); // shows the current position of the claw servo
            //   telemetry.addData("G1:platform", "%.2f", bot.platformPOS); // shows the current position of the platform servo
            // telemetry.addData("Status", "Run Time: " + runtime.toString()); // idk what this does
            //       telemetry.addData("G1:Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); // shows the current position of the motors that move the robot itself
            telemetry.update(); //updates the info to the bottom of the driver station phone
        }
    }

}
