# EZ-Template — 462 fork

> **status: builds and boots in an emulator, no real hardware yet.** Everything physical is unverified until i'm back at school in a few weeks.

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

relay-feedback auto tuning for drive/turn/swing/heading PIDs: a relay test measures the ultimate gain and period (averaged over 3 runs). this is then tuned into constants. kp is bisected until every move size passes — 12/24/48 in for drive, 45/90/135 deg for turns — without going over the overshoot limit. it monitors motor temps and battery the whole time, and every translating test stays inside a runway (`runway_set`, default 60 in, min 24) so it can run on a vex field. 

heading has two paths: `tune_heading` relay-tests the heading hold while driving forward (gains capped against the drive result so straight driving can't jitter), or `set_heading_from_drive` derives conservative constants from the drive tune without moving.

`save_to_sd()` writes everything to `/usd/pid_constants.txt` and `ez::pid_constants_load()` restores it on boot — pid constants, the tracker offset, and the imu scaler. also included: tracking-wheel distance-to-center measurement from a spin test (`ez::tracker_offset_measure`) and an imu scale wizard (`ez::imu_scale_measure`).

```cpp
ez::PIDAutoTuner tuner(chassis);
tuner.runway_set(60.0);
tuner.tune_drive(12.0, 60);
tuner.set_heading_from_drive();
tuner.tune_turn(90.0, 60);
tuner.tune_swing(90.0, 60);
tuner.save_to_sd();
```

or use the controller menu and pick what to tune at run time — `tuner.interactive(master, &horiz_tracker)` — LEFT/RIGHT selects (drive / turn / swing / heading / everything / tracker offset), A runs it, B saves and exits. "everything" runs drive, heading-derived-from-drive, turn, and swing — it does NOT run the heading relay test or the tracker offset; pick those individually. the tracker offset item only shows up if you pass a horizontal tracker.

heads up: nothing calls `interactive()` for you — the example `main.cpp` doesn't wire it in anywhere, so add it yourself (an auton selector entry is good).

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

`ez::screen_rotation_set(90)` rotates the brain screen in 90 deg steps for sideways/upside-down mounts. touch rotates with it.

at 90/270 the screen goes portrait (240 wide x 480 tall), which llemu's fixed layout can't fit — so the selector rebuilds as a portrait copy of the llemu screen: same colors, font, and three-button bar (styles pulled from the kernel's llemu itself), but long lines wrap instead of clipping and the buttons forward to llemu's real ones, so registered callbacks behave identically. 0/180 keep stock llemu untouched.

verified in the emulator at all four rotations, touch included. still unverified on real hardware. known gap: opening the brain pid tuner while portrait is active is untested.

## what's left to test on a robot

the image boots in the emulator, but everything physical still needs a real robot:

- [ ] health preflight catches dead motors/sensors and passes when everything's plugged in
- [ ] auto tuner end to end: relay test oscillates, constants usable at 12/24/48 in and 45/90/135 deg, runway/temp/battery guards fire, sd save + reload on boot
- [ ] interactive tuner menu buttons feel right + don't fight the stock pid tuner on X
- [ ] tracker offset spin test matches a tape measure (sign/flip included), odom stops drifting sideways
- [ ] imu scale wizard lands near 1.0 and persists
- [ ] dsr with real sensors: noise/confidence, off-square + blocked-view rejection, corrected pose matches tape measure on all four sides
- [ ] screen rotation on a real brain (all 4 rotations + touch already verified in the emulator, portrait selector included)
- [ ] json autons smoke test: file loads, selector shows names, one path + dsr action runs
- [ ] get it running correctly on kernel 4.2.2 WITH LemLib hardware

## branches

- `462-additions` — this, on pros kernel 4.1.1 (stable)
- `pros-4.2.2` — experimental: eventually will build on kernel 4.2.2 and add lemlib hardware 0.5.0 + units, hooked into health checks
