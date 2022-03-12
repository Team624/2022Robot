package com.swervedrivespecialties.swervelib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

@FunctionalInterface
public interface SteerControllerFactory<Controller extends SteerController, SC> {
    default void addDashboardEntries(
            ShuffleboardContainer container,
            Controller controller
    ) {
        container.addNumber("Current Angle", () -> Math.toDegrees(controller.getStateAngle()));
        container.addNumber("Target Angle", () -> Math.toDegrees(controller.getReferenceAngle()));
    }

    default Controller create(
            ShuffleboardContainer dashboardContainer,
            SC steerConfiguration,
            String canbus,
            ModuleConfiguration moduleConfiguration
    ) {
        var controller = create(steerConfiguration, canbus, moduleConfiguration);
        addDashboardEntries(dashboardContainer, controller);

        return controller;
    }

    default Controller create(
            ShuffleboardContainer dashboardContainer,
            SC steerConfiguration,
            ModuleConfiguration moduleConfiguration
    ) {
        var controller = create(steerConfiguration, moduleConfiguration);
        addDashboardEntries(dashboardContainer, controller);

        return controller;
    }

    default Controller create(
            SC steerConfiguration, 
            ModuleConfiguration moduleConfiguration
    ) {
        return create(steerConfiguration, "", moduleConfiguration);
    }

    Controller create(SC steerConfiguration, String canbus, ModuleConfiguration moduleConfiguration);
}
