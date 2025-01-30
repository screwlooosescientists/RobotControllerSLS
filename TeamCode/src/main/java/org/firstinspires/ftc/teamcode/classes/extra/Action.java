package org.firstinspires.ftc.teamcode.classes.extra;

@FunctionalInterface
public interface Action<T> {
    void execute(T parameter);
}
