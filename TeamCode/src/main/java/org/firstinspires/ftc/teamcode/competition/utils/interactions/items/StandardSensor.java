package org.firstinspires.ftc.teamcode.competition.utils.interactions.items;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance.wrappers.SensorWrapper;

@Deprecated
public abstract class StandardSensor extends InteractionSurface {
    public abstract int getData();

    abstract int getDistance(DistanceUnit unit);

    abstract void stop();

    abstract void close();

    abstract SensorWrapper getInternalSensor(StandardDistanceSensor.StandardDistanceSensorInternalType type);

    public abstract boolean didTimeoutOccur();
}
