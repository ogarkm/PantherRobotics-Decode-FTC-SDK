package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.GameState;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;

@TeleOp(name="Meet1_Tele", group="FINAL")
public class finalTeleOp extends LinearOpMode {

    private StateMachine stateMachine;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap hardware = new hwMap(hardwareMap);
        stateMachine = new StateMachine(hardware);

        waitForStart();
        stateMachine.setRobotState(RobotState.TELEOP);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            stateMachine.getDriveTrain().teleopDrive(x, y, rx);

            if (gamepad1.right_trigger > 0.5) {
                stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.PRECISION);
            } else if (gamepad1.left_trigger > 0.5) {
                stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.TURBO);
            } else {
                stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.NORMAL);
            }

            if (gamepad1.back) {
                stateMachine.emergencyStop();
            }

            hardware.checkFlickPos(); // TODO - MAKE SURE THIS GETS TRANSFERRED

            if (gamepad1.a) {
                stateMachine.setGameState(GameState.SCORING);
            }
            else {
                stateMachine.setGameState(GameState.IDLE);
            }

            // Telemetry
            telemetry.addData("Robot State", stateMachine.getCurrentRobotState());
            telemetry.addData("Drive State", stateMachine.getDriveTrain().getDriveState());
            telemetry.addData("Transfer System", stateMachine.getTransferSystem().getTransferState());
            telemetry.update();
        }

        stateMachine.setRobotState(RobotState.DISABLED);
    }
}