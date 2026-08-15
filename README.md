# EZ-Template — 462 fork

> **status: this branch gets tested later.** `462-additions` is the priority when i'm back at school in a few weeks; this experimental kernel-4.2.2 branch gets tested after that. treat every feature as unverified until then.

fork of [EZ-Template](https://github.com/EZ-Robotics/EZ-Template) v3.2.2 with extra modules for my team. everything below is what's different from stock EZ — for everything else see the [EZ docs](https://ez-robotics.github.io/EZ-Template/).

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

or use the controller menu and pick what to tune at run time — `tuner.interactive(master, &horiz_tracker)` — LEFT/RIGHT selects (drive / turn / swing / heading / everything / tracker offset), A runs it, B saves and exits.

none of the modules bind any controller buttons on their own; the interactive tuner and the imu wizard only read buttons inside the function you chose to call.

### json autons (`EZ-Template/json_auton.hpp`)

> heads up: this module is nowhere near done. the format and executor are still changing a lot — don't build anything on it yet.

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

## what's left to test on a robot

testing this branch comes after `462-additions`. nothing here has run on real hardware — even the emulator boot check has only been done on the 4.1.1 branch so far.

- [ ] builds boot on a real brain with kernel 4.2.2 (the hard-float fix is the whole point of this branch)
- [ ] lemlib hardware 0.5.0 devices respond + health checks see them
- [ ] health preflight catches dead motors/sensors and passes when everything's plugged in
- [ ] auto tuner end to end: relay test oscillates, constants usable at 12/24/48 in and 45/90/135 deg, runway/temp/battery guards fire, sd save + reload on boot
- [ ] tracker offset spin test matches a tape measure (sign/flip included), odom stops drifting sideways
- [ ] imu scale wizard lands near 1.0 and persists
- [ ] dsr with real sensors: noise/confidence, off-square + blocked-view rejection, corrected pose matches tape measure on all four sides
- [ ] screen rotation on a rotated brain
- [ ] json autons smoke test: file loads, selector shows names, one path + dsr action runs

## branches

- `462-additions` — pros kernel 4.1.1 (stable). tested first.
- `pros-4.2.2` — this. experimental: builds on kernel 4.2.2 (fixes the hard-float/softfp lib mismatch that breaks 4.2.2 builds) and adds lemlib hardware 0.5.0 + units, hooked into health checks. tested after `462-additions`.
