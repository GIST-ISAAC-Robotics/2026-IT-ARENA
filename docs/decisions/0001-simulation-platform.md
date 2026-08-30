# ADR 0001: PC simulation and Jetson deployment platforms

Date: 2026-08-30
Status: accepted for the first milestone

## Decision

Use the existing WSL2 Ubuntu 24.04 distro with ROS 2 Jazzy and Gazebo Harmonic for PC simulation. Preserve the Jetson's JetPack 6.2.1 installation and plan to build competition packages for ROS 2 Humble on its Ubuntu 22.04 base.

## Reasons

- Jazzy + Harmonic is the officially recommended ROS/Gazebo LTS pairing on Ubuntu 24.04.
- The existing WSL distro already has WSLg and working NVIDIA GPU forwarding.
- JetPack 6.2.1 uses Jetson Linux 36.4.4 with an Ubuntu 22.04 root filesystem. Reflashing it only for distro symmetry adds hardware-driver and recovery risk without improving the algorithms.
- Gazebo belongs behind a simulation adapter. Perception, planning, and control code can remain portable across Humble and Jazzy if distro-specific APIs are kept at the edges and CI checks both.
- Running Jazzy in a Jetson container remains an escape hatch, not the baseline, because direct RealSense, serial, GPIO, and acceleration integration would add complexity.

## Rejected alternatives

### Replace WSL 24.04 with Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress

This gives exact ROS distro symmetry, but moves the simulator onto an older pairing close to the event and discards an already healthy WSL environment. A second WSL distro may still be added later for Humble compatibility testing without replacing the current one.

### Use ROS 2 Humble + Gazebo Harmonic everywhere

This pairing exists, but Gazebo documents it as possible-with-caution and distributes non-default packages that can conflict with the normal Humble `ros_gz` packages. It is unnecessary for the first milestone.

### Reflash the Jetson to a 24.04-based environment

Not selected. JetPack's board-support and acceleration stack are the controlling constraints. Reflashing a working target before a concrete dependency requires it is needless risk.

## Compatibility rules

- Use standard messages where possible.
- Isolate `ros_gz` dependencies to simulation packages.
- Avoid relying on Jazzy-only node APIs in portable autonomy packages.
- Add Humble and Jazzy build checks before hardware integration begins.
- Keep configuration files and topic contracts shared even when launch files differ.
