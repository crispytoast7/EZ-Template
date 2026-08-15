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

relay-feedback (astrom-hagglund) auto tuning for drive/turn/swing/heading PIDs: a relay test measures the ultimate gain and period (averaged over 3 runs), tyreus-luyben turns them into constants, then kp is bisected until every move size passes — 12/24/48 in for drive, 45/90/135 deg for turns — without blowing the overshoot limit. watches motor temps and battery the whole time, and every translating test stays inside a runway (`runway_set`, default 60 in, min 24) so it can run on a real field.

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

heads up: nothing calls `interactive()` for you — the example `main.cpp` doesn't wire it in anywhere, so add it yourself (an auton selector entry works well).

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

`ez::screen_rotation_set(90)` rotates the brain screen in 90 deg steps for sideways/upside-down mounts. touch rotates with it — lvgl transforms pointer input itself whenever display rotation is set.

at 90/270 the screen goes portrait, which llemu's fixed layout can't fit — so the selector rebuilds as a portrait copy of the llemu screen: same colors, font, and three-button bar, but long lines wrap instead of clipping and the buttons forward to llemu's real ones, so registered callbacks behave identically. 0/180 keep stock llemu untouched.

on this branch the portrait screen is ported to lvgl 9 (what kernel 4.2.2 vendors) and is compile-verified only — the emulator can't run this kernel at all (it stalls or crashes inside lvgl on the pristine branch too), so unlike the 4.1.1 branch, portrait has never actually rendered here. treat it as untested until it's on a real brain.

## what's left to test on a robot

testing this branch comes after `462-additions`. nothing here has run on real hardware — and unlike the 4.1.1 branch, the emulator is no help: vex-v5-qemu can't run this kernel (it stalls or data-aborts inside lvgl even on a pristine build), so everything below rides on a real brain.

- [ ] builds boot on a real brain with kernel 4.2.2 (the hard-float fix is the whole point of this branch)
- [ ] lemlib hardware 0.5.0 devices respond + health checks see them
- [ ] health preflight catches dead motors/sensors and passes when everything's plugged in
- [ ] auto tuner end to end: relay test oscillates, constants usable at 12/24/48 in and 45/90/135 deg, runway/temp/battery guards fire, sd save + reload on boot
- [ ] tracker offset spin test matches a tape measure (sign/flip included), odom stops drifting sideways
- [ ] imu scale wizard lands near 1.0 and persists
- [ ] dsr with real sensors: noise/confidence, off-square + blocked-view rejection, corrected pose matches tape measure on all four sides
- [ ] screen rotation on a rotated brain, portrait selector at 90/270 included (lvgl 9 port is compile-verified only — it has never rendered anywhere)
- [ ] json autons smoke test: file loads, selector shows names, one path + dsr action runs

## branches

- `462-additions` — pros kernel 4.1.1 (stable). tested first.
- `pros-4.2.2` — this. experimental: builds on kernel 4.2.2 (fixes the hard-float/softfp lib mismatch that breaks 4.2.2 builds) and adds lemlib hardware 0.5.0 + units, hooked into health checks. tested after `462-additions`.
