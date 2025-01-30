package org.firstinspires.ftc.teamcode.classes.extra;

/**
 * Represents a node in a robot's navigation path.
 * A node includes its position, target heading, and optional actions or conditions.
 */
public class Node {

    public float X;
    public float Y;
    public float TargetHeading;
    public float accuracy;
    public boolean HasCondition;

    /** The action to be performed when the node is reached. */
    private Action<?> performAction;

    /** The asynchronous action to be performed when the node is reached. */
    private Action<?> performActionAsync;

    /**
     * Constructs a new Node instance with the given parameters.
     *
     * @param X The x coordinate of the node.
     * @param Y The y coordinate of the node.
     * @param TargetHeading The target heading of the node in degrees.
     * @param HasCondition A boolean indicating whether the node has a condition to be completed before continuing.
     * @param accuracy The accuracy of the node, representing the distance the robot needs to be from the node to continue.
     */
    public Node(float X, float Y, float TargetHeading, boolean HasCondition, float accuracy) {
        this.X = X;
        this.Y = Y;
        this.accuracy = accuracy;
        this.TargetHeading = TargetHeading;
        this.HasCondition = HasCondition;
    }

    /**
     * Sets the action to be performed when the node is reached.
     *
     * @param action A lambda or method reference defining the action to be performed.
     * @param <T> The type of the parameter the action accepts.
     */
    public <T> void setPerformAction(Action<T> action) {
        this.performAction = action;
    }

    /**
     * Sets the asynchronous action to be performed when the node is reached.
     *
     * @param action A lambda or method reference defining the asynchronous action to be performed.
     * @param <T> The type of the parameter the action accepts.
     */
    public <T> void setPerformActionAsync(Action<T> action) {
        this.performActionAsync = action;
    }

    /**
     * Executes the predefined action for this node with the given parameter.
     *
     * @param parameter The parameter to pass to the action.
     * @param <T> The type of the parameter.
     */
    public <T> void PerformAction(T parameter) {
        if (performAction != null) {
            ((Action<T>) performAction).execute(parameter);
        }
    }

    /**
     * Executes the predefined asynchronous action for this node with the given parameter.
     *
     * @param parameter The parameter to pass to the asynchronous action.
     * @param <T> The type of the parameter.
     */
    public <T> void PerformActionAsync(T parameter) {
        if (performActionAsync != null) {
            ((Action<T>) performActionAsync).execute(parameter);
        }
    }
}
