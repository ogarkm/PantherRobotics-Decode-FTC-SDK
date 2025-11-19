package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hware.hwMapExt;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.GameState;

@TeleOp(name="Meet1_Tele", group="FINAL")
public class finalTeleOp extends LinearOpMode {

    private StateMachine stateMachine;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMapExt hardware = new hwMapExt(hardwareMap);
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
            // --- intake control (both intakes together) ---
            if (gamepad1.right_bumper) {
                // intake in
                stateMachine.getIntake().in();
                stateMachine.setGameState(GameState.INTAKING);
            } else if (gamepad1.left_bumper) {
                // intake out
                stateMachine.getIntake().out();
                stateMachine.setGameState(GameState.INTAKING);
            } else {
                // no buttons -> stop intake
                stateMachine.getIntake().stop();
                stateMachine.setGameState(GameState.IDLE);
            }

            if (gamepad1.back) {
                stateMachine.emergencyStop();
            }

            stateMachine.update();

            // Telemetry
            telemetry.addData("Robot State", stateMachine.getCurrentRobotState());
            telemetry.addData("Drive State", stateMachine.getDriveTrain().getDriveState());
            telemetry.update();
        }

        stateMachine.setRobotState(RobotState.DISABLED);
    }
}