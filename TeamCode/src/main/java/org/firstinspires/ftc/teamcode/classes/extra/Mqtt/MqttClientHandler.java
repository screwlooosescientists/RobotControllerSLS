package org.firstinspires.ftc.teamcode.classes.extra.Mqtt;

import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

//TODO Test On controll hub and add documentation
public class MqttClientHandler {

    private MqttClient client;
    private String brokerUrl;
    private String clientId;

    // Constructor to initialize the client with the broker URL, client ID
    public MqttClientHandler(String brokerUrl, String clientId) {
        this.brokerUrl = brokerUrl;
        this.clientId = clientId;
    }

    // Connects to the MQTT broker
    public void connect() {
        try {
            // Create an MQTT client instance
            client = new MqttClient(brokerUrl, clientId);

            // Set MQTT connection options
            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);  // Use clean session (no saved state)
            client.connect(options);  // Connect to the broker

            System.out.println("Connected to broker: " + brokerUrl);
        } catch (MqttException e) {
            System.out.println("Failed to connect to broker: " + e.getMessage());
        }
    }

    // Method to subscribe to a topic and set a message callback
    public void subscribe(String topic, IMqttMessageListener messageListener) {
        try {
            client.subscribe(topic, (receivedTopic, msg) -> {
                String message = new String(msg.getPayload());  // Decode the message
                System.out.println("Received message on topic " + receivedTopic + ": " + message);
                // Call the callback handler if provided
                if (messageListener != null) {
                    messageListener.messageArrived(receivedTopic, msg);
                }
            });
            System.out.println("Subscribed to topic: " + topic);
        } catch (MqttException e) {
            System.out.println("Error subscribing to topic: " + e.getMessage());
        }
    }

    // Method to publish a message to the topic
    public void publish(String topic, String messageContent) {
        try {
            MqttMessage message = new MqttMessage(messageContent.getBytes());
            message.setQos(0);  // Set the QoS level to 0 (at most once)
            client.publish(topic, message);  // Publish the message to the topic
            System.out.println("Published message: " + messageContent + " to topic: " + topic);
        } catch (MqttException e) {
            System.out.println("Error publishing message: " + e.getMessage());
        }
    }

    // Method to disconnect the client
    public void disconnect() {
        try {
            if (client != null && client.isConnected()) {
                client.disconnect();
                System.out.println("Disconnected from broker.");
            }
        } catch (MqttException e) {
            System.out.println("Error disconnecting: " + e.getMessage());
        }
    }

    // Getter for the client to check connection status (optional)
    public boolean isConnected() {
        return client != null && client.isConnected();
    }
}