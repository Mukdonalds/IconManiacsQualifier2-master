package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Example;
import org.firstinspires.ftc.teamcode.HardwareMap.IMHardwareBot;

// CHAWKS: Name it something useful!
@TeleOp(name = "IM Op Mode", group = "A")
//@Disabled
public class IconManiacsTeleOp extends HardwareMap_Example {

    IMHardwareBot bot = new IMHardwareBot();
    public float drive;
    public float strafe;
    public float rotate;
    public float distance = 400f;
    public float power = 0.43f;

    public void restOfTheCode() {

        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        bot.frontLeft.setPower(drive + strafe + rotate);
        bot.backRight.setPower(drive + strafe - rotate);
        bot.backLeft.setPower(drive - strafe + rotate);
        bot.frontRight.setPower(drive - strafe - rotate);

        if (gamepad2.right_bumper) {
            bot.intake.setPower(1);
        } else if (gamepad2.left_bumper) {
            bot.intake.setPower(-1);
        } else {
            bot.intake.setPower(0);
        }

        if (gamepad2.right_trigger == 1) {
            bot.conveyorBelt.setPower(1);
        } else if (gamepad2.left_trigger == 1) {
            bot.conveyorBelt.setPower(-1);
        } else {
            bot.conveyorBelt.setPower(0);
        }

        if (gamepad2.dpad_up) {
            bot.moveArm(0.5, 7, "up");
        } else if (gamepad2.dpad_down) {
            bot.moveArm(0.5, 7, "down");
        }

        if (gamepad2.a) { // if the gamepad 2 left bumper gets pressed....
            //bot.claw.setPosition(11.5);
           bot.secondClaw.setPosition(1);
        } else if (gamepad2.y) { // if the gamepad 2 right bumper gets pressed...
          //  bot.claw.setPosition(0);
            bot.secondClaw.setPosition(0);
        }

    }
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

            restOfTheCode();

            if(gamepad2.b) {
                bot.shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // encoder reset

                double circumference = 3.14*3.937; //pi*diameter
                double rotationsRequired = distance/circumference; // distance input divided by the circumference of the wheel
                int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed multiplied by the ticks per revolution of the GOBILDA motor

                bot. shooter.setTargetPosition(-encoderTargetPOS); // sets the target position to reach

                bot.shooter.setPower(power); // sets the power of the motor accordingly to the motor

                bot.shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION); //changes the run mode of the motor

                while(bot.shooter.isBusy()){ //waiting for the motor to reach the target position
                    restOfTheCode();
                }
                bot.shooter.setPower(0); // turns off the motor
            } else if (gamepad2.x){
                bot.shootRing(0,0);
            }

            //bot.clawPOS = Range.clip(bot.clawPOS, bot.SERVO_LOW_RANGE, bot.SERVO_HIGH_RANGE); // sets the cap for the range of movement for the claw
            // this is the function that actually moves the servo
            // Show the elapsed game time and wheel power.
            telemetry.addData("secondClaw:", "%.2f", bot.secondClaw.getPosition()); // shows the current position of the claw servo
            //   telemetry.addData("G1:platform", "%.2f", bot.platformPOS); // shows the current position of the platform servo
            // telemetry.addData("Status", "Run Time: " + runtime.toString()); // idk what this does
            //       telemetry.addData("G1:Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); // shows the current position of the motors that move the robot itself
            telemetry.update(); //updates the info to the bottom of the driver station phone
                }
        }
    }


