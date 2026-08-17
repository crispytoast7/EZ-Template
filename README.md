# EZ-Template — 462 fork

> **status: main branch, on pros kernel 4.2.2.** boots fully in an emulator (with the black-screen kernel bugs fixed here); real-robot testing happens when i'm back at school in a few weeks. treat every feature as unverified on hardware until then.

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

or use the controller menu and pick what to tune at run time — `tuner.interactive(master)` — LEFT/RIGHT selects (drive / turn / swing / heading / everything / tracker offset), A runs it, B saves and exits. "everything" runs drive, heading-derived-from-drive, turn, and swing — it does NOT run the heading relay test or the tracker offset; pick those individually. the tracker offset item shows up if the chassis has any tracking wheel installed, and one spin sequence measures ALL of them (vertical and horizontal) at once — each gets applied and saved under its own sd label. register your trackers on the chassis before calling `ez::pid_constants_load()` so the saved offsets have somewhere to go on boot.

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

at 90/270 the screen goes portrait, which llemu's fixed layout can't fit — so the selector rebuilds as a portrait copy of the llemu screen: same colors, font, and three-button bar, but long lines wrap instead of clipping and the buttons forward to llemu's real ones, so registered callbacks behave identically. 0/180 keep stock llemu untouched.

lvgl 9 (what kernel 4.2.2 vendors) dropped built-in software rotation, so this branch does it in the display flush path: each flushed band gets software-rotated and remapped to panel coordinates before it reaches the screen. verified in the emulator at all four rotations (portrait selector included), with 0 deg pixel-identical to stock. touch is transformed by lvgl itself, so pixels and touch stay consistent.

## what's left to test on a robot

nothing here has run on real hardware yet — but the big 4.2.x mystery is solved: the frozen-black-screen that upstream hit on real robots ([ez-template pr #309](https://github.com/EZ-Robotics/EZ-Template/pull/309)) was pros kernel 4.2.2 dropping the `.stack` section from its linker script, so the boot stack tramples the last globals in bss — which then cascaded into ez writing through a null FILE* when fopen fails. both are fixed on this branch (gdb-verified in the emulator: abort gone, full pros banner prints). stock vex-v5-qemu can't run this kernel without local patches to its shim (three so far: two spinlock deadlocks and a fragile pipe write) — with those, this branch now boots all the way into opcontrol in the emulator.

- [ ] builds boot on a real brain with kernel 4.2.2 (the hard-float fix is the whole point of this branch)
- [ ] lemlib hardware 0.5.0 devices respond + health checks see them
- [ ] health preflight catches dead motors/sensors and passes when everything's plugged in (test it lying too: unplug one motor + one sensor and check it names them with the right ports)
- [ ] auto tuner end to end: relay test oscillates, constants usable at 12/24/48 in and 45/90/135 deg, runway abort actually cuts motors before the field edge, temp wait resumes after cooling, sd save + reload on boot
- [ ] heading hold, both paths: derived-from-drive (what "everything" gives you) drives straight without jitter; tune_heading relay version if you want it tighter
- [ ] interactive tuner menu buttons feel right + don't fight the stock pid tuner on X
- [ ] imu scale wizard lands near 1.0 and persists — run it BEFORE the tracker offsets, the offset math trusts the imu
- [ ] tracker offset spin test on every installed tracker (vertical AND horizontal): each matches a tape measure (sign/flip included), odom stops drifting sideways
- [ ] dsr with real sensors: noise/confidence, off-square + blocked-view rejection, corrected pose matches tape measure on all four sides
- [ ] screen rotation on a real brain: all 4 rotations + touch (already verified in the emulator, portrait selector included)
- [ ] brain pid tuner (X) while portrait rotation is active — known untested interaction, may draw llemu widgets onto the portrait screen
- [ ] boot with NO sd card: tuner save fails gracefully, constants fall back to whatever's in code
- [ ] on a comp switch: pid tuner + run-auton-on-DOWN+B stay disabled, selector still pages, selected auton fires in auton mode
- [ ] json autons smoke test: file loads, selector shows names, one path + dsr action runs

## branches

- `main` — this. all the custom features on pros kernel 4.2.2, plus lemlib hardware 0.5.0 + units hooked into health checks.
- `pros-4.2.2-enhancements` — stock ez-template on kernel 4.2.2 with only the boot fixes. features get added here gradually.
- `pros-4.2.2-fixes` — the upstream pr payload (kernel bump + the three boot fixes). frozen.
- the old kernel-4.1.1 branch (hardware-confirmed boot + rotation) is preserved at the `archive/462-additions` tag.
