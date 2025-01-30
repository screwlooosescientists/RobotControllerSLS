package org.firstinspires.ftc.teamcode.classes.extra.websockets;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import fi.iki.elonen.NanoWSD;

public class TrackTracerWebHandler extends NanoWSD {

    private static final List<GenericWebSocket> connectedClients = new ArrayList<>();
    private final int port;

    public TrackTracerWebHandler(int port) {
        super(port);
        this.port = port;
    }

    public int getPort() {
        return port;
    }

    @Override
    protected WebSocket openWebSocket(IHTTPSession handshake) {
        GenericWebSocket client = new GenericWebSocket(handshake);
        connectedClients.add(client);
        return client;
    }

    @Override
    public void start() {
        try {
            super.start();
            System.out.println("WebSocket server started on port " + getPort());
        } catch (IOException e) {
            System.err.println("Failed to start WebSocket server: " + e.getMessage());
        }
    }

    @Override
    public void stop() {
        super.stop();
        System.out.println("WebSocket server stopped.");
    }

    // New method to send any JSON string to all connected clients
    public void sendJsonToAllClients(String json) {
        for (GenericWebSocket client : connectedClients) {
            try {
                client.send(json);
            } catch (IOException e) {
                System.err.println("Error sending data to client: " + e.getMessage());
            }
        }
    }

    private static class GenericWebSocket extends WebSocket {

        public GenericWebSocket(IHTTPSession handshakeRequest) {
            super(handshakeRequest);
        }

        @Override
        protected void onOpen() {
            System.out.println("WebSocket connection opened: " + this.hashCode());
        }

        @Override
        protected void onClose(WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) {
            System.out.println("WebSocket connection closed: " + this.hashCode());
            connectedClients.remove(this);
        }

        @Override
        protected void onMessage(WebSocketFrame message) {
            System.out.println("Received message: " + message.getTextPayload());
        }

        @Override
        protected void onPong(WebSocketFrame pong) {
            System.out.println("Received pong: " + pong);
        }

        @Override
        protected void onException(IOException e) {
            System.err.println("WebSocket exception: " + e.getMessage());
        }
    }
}


