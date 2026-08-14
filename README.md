![](https://img.shields.io/github/downloads/EZ-Robotics/EZ-Template/total.svg)
![](https://github.com/EZ-Robotics/EZ-Template/workflows/Build/badge.svg)
[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)

⚡️ **coding made ez** ⚡️

💅 **docs and tutorials** 💅 

💥 **pid, odometry, pure pursuit, boomerang** 💥

📚 **focus on consistency** 📚

🧐 **out of the box documentation** 🧐

🔌 **plug and play example project** 🔌

[See a complete playlist of EZ-Template autons here!](https://www.youtube.com/playlist?list=PLyZbi14KopZK70GTSD5NpygoAcM2_ls7T)


## Fork additions (crispytoast7/EZ-Template, branch 462-additions)

This fork adds five modules to stock EZ-Template 3.2.2, all exposed through
`EZ-Template/api.hpp`:

- **`ez::dsr`** — four-sided distance-sensor position resets with rejection
  guards (`EZ-Template/dsr.hpp`)
- **`ez::PIDAutoTuner`** — relay-feedback auto-tuning for drive/turn/swing/
  heading with SD persistence, tracker-offset measurement, and an interactive
  IMU scale wizard (`EZ-Template/tuner.hpp`)
- **`ez::json_*`** — SD-card JSON autonomous routines planned in
  `tools/auton_planner.html` (`EZ-Template/json_auton.hpp`)
- **`ez::health`** — device preflight: IMU, drive motors, odom trackers, and
  DSR sensors (`EZ-Template/health.hpp`)
- **`ez::screen_flip_set`** — 180-degree screen rotation for upside-down
  mounted brains, experimental (`EZ-Template/display.hpp`)


## [Discord Server](https://discord.gg/EHjXBcK2Gy)
Need extra assistance using EZ-Template?  Feel free to join our [Discord Server](https://discord.gg/EHjXBcK2Gy)! 

## Features
EZ-Template is built with high attention to the user experience.

* Built with 💜 and [PROS](https://pros.cs.purdue.edu/)
  * Powerful open source Development platform for VEX V5 
  * Customize and extend with other community PROS libraries
* 🔌 Example project is plug-and-play
  * Simple to setup
  * Get up and running in minutes
* 👀 Simple to use API
  * PID for driving, turning, swing turns, and arcs
  * Odometry with Pure Pursuit and Boomerang
  * Asynchronous PID with blocking functions
  * "Tug of war" detection
  * Overheat detection and exiting
  * Live PID tuning
  * Tracking wheel support
* 📺 Autonomous selector
  * Easy to add autons
  * SD card saving
* 🎮 Joystick control
  * Tank drive, single stick arcade, and dual stick arcade
  * Joystick input curves
  * Adjust joystick curves live through the controller
* PID for your own subsystems
* Slew for your own subsystems


## Design Principles
* **Quick to get going.**  EZ-Template should make it easy to learn and use.  Anything is achievable by users, even if it takes them more code and more time to write.  
* **Intuitive.**  Users will not feel overwhelmed when looking at an EZ-Template project or adding new features.  It should look intuitive and easy to build on top of.  
* **Sensible Defaults.**  Common and popular performance optimizations and configurations will be done for users, but only with the option to override them.  

We believe that, as developers, knowing how a library works helps us become better at using it.  We're dedicating effort to creating tutorials and documentation with the hope that reading it will gain the user a deeper understanding of the tool, and become even more proficient in using it.  

## [Support me on Patreon!](https://www.patreon.com/roboticsisez)
Supporting me on Patreon will help guarantee that EZ-Template continues to get maintained and allow me to develop products for teams to use.  [Click here](https://www.patreon.com/roboticsisez) to see my Patreon!

## [Download and Installation](https://ez-robotics.github.io/EZ-Template/tutorials/installation)
Learn how to install and setup EZ-Template [here](https://ez-robotics.github.io/EZ-Template/tutorials/installation)!

## [License](https://opensource.org/licenses/MPL-2.0)
This project is licensed under the Mozilla Public License, version 2.0 - see the [LICENSE](https://opensource.org/licenses/MPL-2.0) file for the full license.