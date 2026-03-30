// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.NeutralZoneLeftTrajectory$0;
import static frc.robot.generated.ChoreoTraj.NeutralZoneLeftTrajectory$1;
import static frc.robot.generated.ChoreoTraj.NeutralZoneLeftTrajectory$2;

import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$0;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$1;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$2;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$3;

import static frc.robot.generated.ChoreoTraj.NZLeftTrajectory$0;
import static frc.robot.generated.ChoreoTraj.NZLeftTrajectory$1;

import static frc.robot.generated.ChoreoTraj.NZRightTrajectory$0;
import static frc.robot.generated.ChoreoTraj.NZRightTrajectory$1;

import static frc.robot.generated.ChoreoTraj.NZLeftTrajectory_Center$0;
import static frc.robot.generated.ChoreoTraj.NZLeftTrajectory_Center$1;

import static frc.robot.generated.ChoreoTraj.NZRightTrajectory_Center$0;
import static frc.robot.generated.ChoreoTraj.NZRightTrajectory_Center$1;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.dashboard.Dashboard;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.Speed;

public final class AutoRoutines {
        private final Swerve swerve;
        private final Intake intake;
        private final Floor floor;
        private final Feeder feeder;
        private final Shooter shooter;
        private final Hood hood;
        private final Hanger hanger;
        private final Limelight limelight;

        private final Dashboard dashboard;

        private final SubsystemCommands subsystemCommands;

        private final AutoFactory autoFactory;
        private final AutoChooser autoChooser;

        public AutoRoutines(
                        Swerve swerve,
                        Intake intake,
                        Floor floor,
                        Feeder feeder,
                        Shooter shooter,
                        Hood hood,
                        Hanger hanger,
                        Limelight limelight,
                        Dashboard dashboard) {
                this.swerve = swerve;
                this.intake = intake;
                this.floor = floor;
                this.feeder = feeder;
                this.shooter = shooter;
                this.hood = hood;
                this.hanger = hanger;
                this.limelight = limelight;

                this.dashboard = dashboard;

                this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger,
                                dashboard);

                this.autoFactory = swerve.createAutoFactory();
                this.autoChooser = new AutoChooser();
        }

        public void configure() {
                autoChooser.addRoutine("Outpost and Depot", this::outpostAndDepotRoutine);
                autoChooser.addRoutine("NZ Left No Climb", this::nzLeftNoClimbRoutine);
                autoChooser.addRoutine("NZ Right No Climb", this::nzRightNoClimbRoutine);
                autoChooser.addRoutine("NZ Left Center No Climb", this::nzLeftCenterNoClimbRoutine);
                autoChooser.addRoutine("NZ Right Center No Climb", this::nzRightCenterNoClimbRoutine);

                SmartDashboard.putData("Auto Chooser", autoChooser);
                RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        }

        private AutoRoutine nzLeftNoClimbRoutine() {
                final AutoRoutine routine = autoFactory.newRoutine("NZ Left No Climb");

                final AutoTrajectory startToFirstStop = NZLeftTrajectory$0.asAutoTraj(routine);
                final AutoTrajectory firstStopToShootingPose = NZLeftTrajectory$1.asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.setAutoSelected("NZ Left No Climb");
                                                        dashboard.clearAutoFailure();
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoPath("NZLeftTrajectory$0");
                                                        dashboard.setAutoMarker("RoutineStart");
                                                        dashboard.setAutoState("StartToFirstStop");
                                                }),
                                                startToFirstStop.resetOdometry(),
                                                startToFirstStop.cmd()));

                startToFirstStop.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("StartToFirstStop");
                                                        dashboard.setAutoMarker("ReachedFirstStop");
                                                        dashboard.setAutoState("LowerIntakeAndDriveToShoot");
                                                        dashboard.setAutoPath("NZLeftTrajectory$1");
                                                }),
                                                Commands.waitSeconds(0.2), // cambiar este si brownout
                                                Commands.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                                                Commands.waitSeconds(0.2),
                                                Commands.runOnce(() -> dashboard.setAutoMarker("IntakeLowered"))));

                startToFirstStop.done().onTrue(intake.intakeCommand());

                startToFirstStop.done().onTrue(firstStopToShootingPose.cmd());
                firstStopToShootingPose.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("LowerIntakeAndDriveToShoot");
                                                        dashboard.setAutoMarker("ReachedShootingPose");
                                                        dashboard.setAutoState("AimAndShoot");
                                                        dashboard.setAutoWaitingOn("LaunchSequence");
                                                }),
                                                intake.runOnce(() -> intake.set(Intake.Speed.STOP)),
                                                subsystemCommands.aimAndShoot().withTimeout(10.0),
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("AimAndShoot");
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoMarker("RoutineComplete");
                                                        dashboard.setAutoState("Done");
                                                })));

                return routine;
        }

        private AutoRoutine nzLeftCenterNoClimbRoutine() {
                final AutoRoutine routine = autoFactory.newRoutine("NZ Left Center No Climb");

                final AutoTrajectory startToFirstStop = NZLeftTrajectory_Center$0.asAutoTraj(routine);
                final AutoTrajectory firstStopToShootingPose = NZLeftTrajectory_Center$1.asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.setAutoSelected("NZ Left Center No Climb");
                                                        dashboard.clearAutoFailure();
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoPath("NZLeftTrajectory_Center$0");
                                                        dashboard.setAutoMarker("RoutineStart");
                                                        dashboard.setAutoState("StartToFirstStop");
                                                }),
                                                startToFirstStop.resetOdometry(),
                                                startToFirstStop.cmd()));

                startToFirstStop.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("StartToFirstStop");
                                                        dashboard.setAutoMarker("ReachedFirstStop");
                                                        dashboard.setAutoState("LowerIntakeAndDriveToShoot");
                                                        dashboard.setAutoPath("NZLeftTrajectory_Center$1");
                                                }),
                                                Commands.waitSeconds(0.2),
                                                Commands.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                                                Commands.waitSeconds(0.2),
                                                Commands.runOnce(() -> dashboard.setAutoMarker("IntakeLowered"))));

                startToFirstStop.done().onTrue(intake.intakeCommand());

                startToFirstStop.done().onTrue(firstStopToShootingPose.cmd());

                firstStopToShootingPose.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("LowerIntakeAndDriveToShoot");
                                                        dashboard.setAutoMarker("ReachedShootingPose");
                                                        dashboard.setAutoState("AimAndShoot");
                                                        dashboard.setAutoWaitingOn("LaunchSequence");
                                                }),
                                                intake.runOnce(() -> intake.set(Intake.Speed.STOP)),
                                                subsystemCommands.aimAndShoot().withTimeout(10.0),
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("AimAndShoot");
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoMarker("RoutineComplete");
                                                        dashboard.setAutoState("Done");
                                                })));

                return routine;
        }

        private AutoRoutine nzRightNoClimbRoutine() {
                final AutoRoutine routine = autoFactory.newRoutine("NZ Right No Climb");

                final AutoTrajectory startToFirstStop = NZRightTrajectory$0.asAutoTraj(routine);
                final AutoTrajectory firstStopToShootingPose = NZRightTrajectory$1.asAutoTraj(routine);
                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.setAutoSelected("NZ Right No Climb");
                                                        dashboard.clearAutoFailure();
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoPath("NZRightTrajectory$0");
                                                        dashboard.setAutoMarker("RoutineStart");
                                                        dashboard.setAutoState("StartToFirstStop");
                                                }),
                                                startToFirstStop.resetOdometry(),
                                                startToFirstStop.cmd()));

                startToFirstStop.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("StartToFirstStop");
                                                        dashboard.setAutoMarker("ReachedFirstStop");
                                                        dashboard.setAutoState("LowerIntakeAndDriveToShoot");
                                                        dashboard.setAutoPath("NZRightTrajectory$1");
                                                }),

                                                Commands.waitSeconds(0.2), // cambiar este si brownout
                                                Commands.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                                                Commands.waitSeconds(0.2),
                                                Commands.runOnce(() -> dashboard.setAutoMarker("IntakeLowered"))));

                startToFirstStop.done().onTrue(intake.intakeCommand());

                startToFirstStop.done().onTrue(firstStopToShootingPose.cmd());
                firstStopToShootingPose.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("LowerIntakeAndDriveToShoot");
                                                        dashboard.setAutoMarker("ReachedShootingPose");
                                                        dashboard.setAutoState("AimAndShoot");
                                                        dashboard.setAutoWaitingOn("LaunchSequence");
                                                }),
                                                intake.runOnce(() -> intake.set(Intake.Speed.STOP)),
                                                subsystemCommands.aimAndShoot().withTimeout(10.0),
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("AimAndShoot");
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoMarker("RoutineComplete");
                                                        dashboard.setAutoState("Done");
                                                })));

                return routine;
        }

        private AutoRoutine nzRightCenterNoClimbRoutine() {
                final AutoRoutine routine = autoFactory.newRoutine("NZ Right Center No Climb");

                final AutoTrajectory startToFirstStop = NZRightTrajectory_Center$0.asAutoTraj(routine);
                final AutoTrajectory firstStopToShootingPose = NZRightTrajectory_Center$1.asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.setAutoSelected("NZ Right Center No Climb");
                                                        dashboard.clearAutoFailure();
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoPath("NZRightTrajectory_Center$0");
                                                        dashboard.setAutoMarker("RoutineStart");
                                                        dashboard.setAutoState("StartToFirstStop");
                                                }),
                                                startToFirstStop.resetOdometry(),
                                                startToFirstStop.cmd()));

                startToFirstStop.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("StartToFirstStop");
                                                        dashboard.setAutoMarker("ReachedFirstStop");
                                                        dashboard.setAutoState("LowerIntakeAndDriveToShoot");
                                                        dashboard.setAutoPath("NZRightTrajectory_Center$1");
                                                }),
                                                Commands.waitSeconds(0.2),
                                                Commands.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                                                Commands.waitSeconds(0.2),
                                                Commands.runOnce(() -> dashboard.setAutoMarker("IntakeLowered"))));

                startToFirstStop.done().onTrue(intake.intakeCommand());

                startToFirstStop.done().onTrue(firstStopToShootingPose.cmd());

                firstStopToShootingPose.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("LowerIntakeAndDriveToShoot");
                                                        dashboard.setAutoMarker("ReachedShootingPose");
                                                        dashboard.setAutoState("AimAndShoot");
                                                        dashboard.setAutoWaitingOn("LaunchSequence");
                                                }),
                                                // intake.runOnce(() -> intake.set(Intake.Speed.STOP)),
                                                subsystemCommands.aimAndShoot().withTimeout(10.0),
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("AimAndShoot");
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoMarker("RoutineComplete");
                                                        dashboard.setAutoState("Done");
                                                })));

                return routine;
        }

        @Deprecated(since = "2026-03", forRemoval = false)
        private AutoRoutine neutralZoneRoutine() {
                final AutoRoutine routine = autoFactory.newRoutine("Neutral Zone");

                final AutoTrajectory startToNeutralZone = NeutralZoneLeftTrajectory$0.asAutoTraj(routine);
                final AutoTrajectory neutralZoneToShootingPose = NeutralZoneLeftTrajectory$1.asAutoTraj(routine);
                final AutoTrajectory shootingPoseToTower = NeutralZoneLeftTrajectory$2.asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.setAutoSelected("Neutral Zone Left");
                                                        dashboard.clearAutoFailure();
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoPath("NeutralZoneLeftTrajectory$0");
                                                        dashboard.setAutoMarker("RoutineStart");
                                                        dashboard.setAutoState("StartToNeutralZone");
                                                }),
                                                startToNeutralZone.resetOdometry(),
                                                startToNeutralZone.cmd()));

                routine.observe(hanger::isHomed).onTrue(
                                Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

                startToNeutralZone.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());

                startToNeutralZone.doneDelayed(0.2).onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("StartToNeutralZone");
                                                        dashboard.setAutoPath("NeutralZoneLeftTrajectory$1");
                                                        dashboard.setAutoMarker("ReachedNeutralZone");
                                                        dashboard.setAutoState("NeutralZoneToShootingPose");
                                                }),
                                                neutralZoneToShootingPose.cmd()));

                neutralZoneToShootingPose.active().whileTrue(limelight.idle());

                neutralZoneToShootingPose.atTime(0.5).onTrue(
                                Commands.parallel(
                                                shooter.spinUpCommand(2600),
                                                hood.positionCommand(0.32)));

                neutralZoneToShootingPose.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("NeutralZoneToShootingPose");
                                                        dashboard.setAutoMarker("ReachedShootingPose");
                                                        dashboard.setAutoState("AimAndShoot");
                                                        dashboard.setAutoWaitingOn("LaunchSequence");
                                                }),
                                                subsystemCommands.aimAndShoot().withTimeout(10.0),
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("AimAndShoot");
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoPath("NeutralZoneLeftTrajectory$2");
                                                        dashboard.setAutoMarker("FinishedVolley");
                                                        dashboard.setAutoState("ShootingPoseToTower");
                                                }),
                                                shootingPoseToTower.cmd()));

                shootingPoseToTower.active().whileTrue(limelight.idle());

                shootingPoseToTower.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> dashboard.setAutoMarker("ClimbPrep")),
                                                hanger.positionCommand(Hanger.Position.HANGING)));

                shootingPoseToTower.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("ShootingPoseToTower");
                                                        dashboard.setAutoMarker("ReachedTower");
                                                        dashboard.setAutoState("HangFinish");
                                                }),
                                                hanger.positionCommand(Hanger.Position.HUNG),
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("HangFinish");
                                                        dashboard.setAutoMarker("RoutineComplete");
                                                        dashboard.setAutoState("Done");
                                                })));

                return routine;
        }

        private AutoRoutine outpostAndDepotRoutine() {
                final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot");

                final AutoTrajectory startToOutpost = OutpostAndDepotTrajectory$0.asAutoTraj(routine);
                final AutoTrajectory outpostToDepot = OutpostAndDepotTrajectory$1.asAutoTraj(routine);
                final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectory$2.asAutoTraj(routine);
                final AutoTrajectory shootingPoseToTower = OutpostAndDepotTrajectory$3.asAutoTraj(routine);

                routine.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.setAutoSelected("Outpost and Depot");
                                                        dashboard.clearAutoFailure();
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoPath("OutpostAndDepotTrajectory$0");
                                                        dashboard.setAutoMarker("RoutineStart");
                                                        dashboard.setAutoState("StartToOutpost");
                                                }),
                                                startToOutpost.resetOdometry(),
                                                startToOutpost.cmd()));

                routine.observe(hanger::isHomed).onTrue(
                                Commands.sequence(
                                                Commands.waitSeconds(0.5),
                                                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))));

                startToOutpost.doneDelayed(1).onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("StartToOutpost");
                                                        dashboard.setAutoPath("OutpostAndDepotTrajectory$1");
                                                        dashboard.setAutoMarker("ReachedOutpost");
                                                        dashboard.setAutoState("OutpostToDepot");
                                                }),
                                                outpostToDepot.cmd()));

                outpostToDepot.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());

                outpostToDepot.doneDelayed(0.1).onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("OutpostToDepot");
                                                        dashboard.setAutoPath("OutpostAndDepotTrajectory$2");
                                                        dashboard.setAutoMarker("ReachedDepot");
                                                        dashboard.setAutoState("DepotToShootingPose");
                                                }),
                                                depotToShootingPose.cmd()));

                depotToShootingPose.active().whileTrue(limelight.idle());

                depotToShootingPose.atTime(0.5).onTrue(
                                Commands.parallel(
                                                shooter.spinUpCommand(2600),
                                                hood.positionCommand(0.32)));

                depotToShootingPose.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("DepotToShootingPose");
                                                        dashboard.setAutoMarker("ReachedShootingPose");
                                                        dashboard.setAutoState("AimAndShoot");
                                                        dashboard.setAutoWaitingOn("LaunchSequence");
                                                }),
                                                subsystemCommands.aimAndShoot().withTimeout(5),
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("AimAndShoot");
                                                        dashboard.clearAutoWaitingOn();
                                                        dashboard.setAutoPath("OutpostAndDepotTrajectory$3");
                                                        dashboard.setAutoMarker("FinishedVolley");
                                                        dashboard.setAutoState("ShootingPoseToTower");
                                                }),
                                                shootingPoseToTower.cmd()));

                shootingPoseToTower.active().whileTrue(limelight.idle());

                shootingPoseToTower.active().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> dashboard.setAutoMarker("ClimbPrep")),
                                                hanger.positionCommand(Hanger.Position.HANGING)));

                shootingPoseToTower.done().onTrue(
                                Commands.sequence(
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("ShootingPoseToTower");
                                                        dashboard.setAutoMarker("ReachedTower");
                                                        dashboard.setAutoState("HangFinish");
                                                }),
                                                hanger.positionCommand(Hanger.Position.HUNG),
                                                Commands.runOnce(() -> {
                                                        dashboard.completeAutoState("HangFinish");
                                                        dashboard.setAutoMarker("RoutineComplete");
                                                        dashboard.setAutoState("Done");
                                                })));

                return routine;
        }
}
