# EZ-Template — 462 fork

fork of [EZ-Template](https://github.com/EZ-Robotics/EZ-Template) v3.2.2 with extra modules for team 462 Override. everything below is what's different from stock EZ — for everything else see the [EZ docs](https://ez-robotics.github.io/EZ-Template/).

## added modules

all exposed through `EZ-Template/api.hpp`, namespace `ez`.

### dsr — distance sensor wall resets (`EZ-Template/dsr.hpp`)

register a distance sensor per side (front/right/back/left) with its offset from the robot center. `ez::dsr::correct()` reads them, figures out which wall each one faces, and overwrites that axis of the pose. bad readings get rejected (low confidence, out of range, off square, or something in the way).

```cpp
ez::dsr::sensor_add(&back_distance, ez::dsr::Side::BACK, 7.0);
ez::dsr::sensor_add(&right_distance, ez::dsr::Side::RIGHT, 7.0);

double x = chassis.odom_x_get(), y = chassis.odom_y_get();
if (ez::dsr::correct(chassis.drive_imu_get(), x, y)) {
  chassis.odom_x_set(x);
  chassis.odom_y_set(y);
}
```

### auto tuner (`EZ-Template/tuner.hpp`)

relay-feedback auto tuning for drive/turn/swing/heading PIDs. checks the result at multiple move sizes (12/24/48 in, 45/90/135 deg), watches motor temps and battery, stays inside a runway so it can run on a real field, and saves to the sd card. `ez::pid_constants_load()` restores everything on boot. also measures the tracking wheel's distance-to-center from a spin test and has an imu scale wizard (`ez::imu_scale_measure`).

```cpp
ez::PIDAutoTuner tuner(chassis);
tuner.runway_set(60.0);
tuner.tune_drive(12.0, 60);
tuner.set_heading_from_drive();
tuner.tune_turn(90.0, 60);
tuner.tune_swing(90.0, 60);
tuner.save_to_sd();
```

### json autons (`EZ-Template/json_auton.hpp`)

runs autons from a json file on the sd card (`/usd/autons.json`) — each one shows up in the auton selector by name. supports smooth pure-pursuit runs, boomerang arrival headings, reverse, per-point speed/waits, and a built-in `dsr` action that squares up to the nearest wall and resets the pose. other actions go to a handler you set:

```cpp
ez::json_action_handler_set([](const std::string& a) {
  if (a == "intake_on") { intake.move(127); return; }
});
ez::json_register_selector(chassis);  // in initialize()
```

### health checks (`EZ-Template/health.hpp`)

`ez::health::preflight(chassis, master)` checks the imu, drive motors, odom trackers, and dsr sensors all respond, prints anything dead with its port, and rumbles the controller.

### screen rotation (`EZ-Template/display.hpp`)

`ez::screen_rotation_set(180)` rotates the brain screen in 90 deg steps for sideways/upside-down mounts. experimental — touch input doesn't rotate with it.

## branches

- `462-additions` — this, on pros kernel 4.1.1 (stable)
- `pros-4.2.2` — experimental: builds on kernel 4.2.2 (fixes the hard-float/softfp lib mismatch that breaks 4.2.2 builds) and adds lemlib hardware 0.5.0 + units, hooked into health checks. not tested on a robot yet.
