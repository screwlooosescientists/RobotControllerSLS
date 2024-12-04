package org.firstinspires.ftc.teamcode.classes.extra.Mqtt;

import io.moquette.broker.Server;
import io.moquette.broker.config.MemoryConfig;

import java.util.Properties;

//TODO: Test this class on controll hub
public class MqttBroker {
    public static void StartBroker(String[] args) {
        Server mqttBroker = new Server();
        try {
            // Configure the broker
            Properties configProps = new Properties();
            configProps.setProperty("port", "1883"); // Default MQTT port
            configProps.setProperty("host", "0.0.0.0"); // Listen on all interfaces
            configProps.setProperty("allow_anonymous", "true"); // Allow anonymous clients

            // Start the MQTT broker
            mqttBroker.startServer(new MemoryConfig(configProps));
            System.out.println("MQTT Broker is running on the REV Robotics Hub.");

            // Keep the application running
            Runtime.getRuntime().addShutdownHook(new Thread(() -> {
                System.out.println("Stopping MQTT Broker...");
                mqttBroker.stopServer();
            }));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
