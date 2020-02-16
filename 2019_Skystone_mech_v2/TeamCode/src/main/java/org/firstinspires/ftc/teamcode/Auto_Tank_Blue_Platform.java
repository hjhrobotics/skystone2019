/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


@Autonomous(name="91_Auto:TANK_MovePlatform", group="Linear Opmode")
//@Disabled
public class Auto_Tank_Blue_Platform extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    MechRobot robot = new MechRobot();
    int autoCase = 1;
    double secondsToPlatform;
    double commandStartTime;




    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Range: ", robot.getRange() + "mm");

        robot.initGyro(hardwareMap);
        telemetry.update();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry


            switch(autoCase) {
                case 1:
                    robot.drive(-.5, -.2);
                    if(robot.gyroAngle1() > 15 ) {
                        autoCase = 2;
                    }
                    break;
                case 2:

                    robot.drive(-.4, -.4);
                    if(robot.getRange() < 70) {
                       autoCase = 3;

                    }
                    break;
                case 3:
                    robot.drive(0, -.4);
                    if(robot.gyroAngle1() < 0) {
                        runtime.reset();
                        autoCase = 4;
                    }
                    break;
                case 4:
                    robot.drive(0, 0);
                    if(runtime.seconds() > 1) {
                        autoCase = 5;
                    }
                    break;
                case 5:
                    robot.grabPlatform();
                    if(runtime.seconds() > 2.5) {
                        autoCase = 6;
                    }

                    break;
                case 6:
                    robot.drive(.5, -.5);
                    if(robot.gyroAngle1() < -175) {
                        runtime.reset();
                        autoCase = 7;
                    }
                    break;
                case 7:
                    robot.drive(-.4,-.4);
                    if(runtime.seconds() > 2.7) {
                        autoCase = 8;
                    }

                    break;
                case 8:
                    robot.drive(0,0);
                    robot.releasePlatform();
                    if(runtime.seconds() > 3.5) {
                        autoCase = 9;
                    }


                    break;
                case 9:
                    robot.drive(.4, 0);
                    if(robot.gyroAngle1() < 84 && runtime.seconds() > 4.5) {
                        autoCase = 10;
                        runtime.reset();
                    }
                    break;
                case 10:
                    robot.drive(.4, .4);
                    if(runtime.seconds() > 2) {
                       autoCase = 11;
                    }
                    break;
                default:
                    robot.drive(0, 0);
                    break;
            }





            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Angle 1: ", robot.gyroAngle1());
            telemetry.addData("Range: ", robot.getRange() + "mm");
            telemetry.addData("Seconds to Platform: ", secondsToPlatform + " sec");
            telemetry.addData("Command Start: ", commandStartTime + " sec");
            telemetry.addData("Auto Case: ", autoCase);
            if(robot.getTouch2()) {
                telemetry.addData("Touch Sensor 2: ", "Pressed");
            } else {
                telemetry.addData("Touch Sensor 2: ", "Not Pressed");
            }


            telemetry.update();
        }
    }
}
