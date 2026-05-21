# Fix CycloneDDS Local Discovery on Pi

**Date:** 2026-05-21  
**Status:** Approved

## Problem

`cyclone_pi.xml` listed only the PC (`10.83.158.34`) as a unicast peer.  
With `AllowMulticast=false`, CycloneDDS only sends SPDP discovery announcements to explicit peers.  
Result: Pi-local nodes (driver, SLAM, radar) never discover each other → SLAM waits for TF forever → `/map` never published.

## Fix

### 1. Add loopback peer to CycloneDDS config

```xml
<Discovery>
  <Peers>
    <Peer Address="127.0.0.1"/>    <!-- Pi local inter-process discovery -->
    <Peer Address="10.83.158.34"/> <!-- PC (Humble) -->
  </Peers>
</Discovery>
```

### 2. Move config out of /tmp

Commit `cyclone_pi.xml` to `esibot_bringup/config/cyclone_pi.xml`.  
Update `CYCLONEDDS_URI` in `start_robot.sh` to reference that path.  
`/tmp` is cleared on reboot; a committed file survives.

## Expected result

- `ros2 node list` shows all Pi nodes
- `/tf` visible from SLAM (odom → base_footprint → laser_link chain complete)
- SLAM activates, publishes `/map`
- RViz on PC displays map and scan
