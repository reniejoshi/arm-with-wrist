package org.tahomarobotics.robot.util.identity;

@SuppressWarnings("ALL")
public enum Identity {
    BETA("00:80:2f:32:fd:29"),
    BEEF("00:80:2f:40:6a:02");

    private final String addr;

    private Identity(String addr) {
        this.addr = addr;
    }
}
