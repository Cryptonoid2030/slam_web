# SLAM Lab guides

[Open the demo](https://cognipilot.github.io/slam_web/) and click **Run** when loading finishes. Nothing needs installing.

| Guide | What you can do |
| --- | --- |
| [Using the lab](guides/using-the-lab.md) | Fly, explore interiors, adjust graphics and use the mobile layout. |
| [Editing Modelica](guides/editing-modelica.md) | Select an example, edit source and save your project. |
| [Sensors and timing](guides/sensors.md) | Set camera, LiDAR, IMU and GPS rates and understand their outputs. |
| [Hosting and deployment](guides/deployment.md) | Build the static site and understand the deployment roadmap. |
| [Live iPhone capture](guides/live-iphone.md) | Stream LiDAR, camera and IMU from an iPhone into the native SLAM and mirror the map in Safari. |
| [Hardware captures](guides/hardware-captures.md) | Record office/hallway inputs and refuse software fallback before performance runs. |

The demo runs quadrotor physics, inertial navigation and GPU sensors. Full visual
SLAM execution is still being integrated; opening its source does not activate it.

The Modelica library belongs to [CogniPilot Modelica Models](https://github.com/CogniPilot/modelica_models).
The web app provides rendering, editing and browser execution through
[Rumoca](https://github.com/CogniPilot/rumoca).
