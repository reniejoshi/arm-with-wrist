package org.tahomarobotics.robot.whoami;

@SuppressWarnings("ALL")
public enum Identity {
    BETA("00:00:00:00:00:00"), BEARITONE("00:80:2f:32:fd:29");

    private final String addr;

    private Identity(String addr) {
        this.addr = addr;
    }
}
