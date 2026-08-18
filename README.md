# EZ-Template — 462 fork

fork of [EZ-Template](https://github.com/EZ-Robotics/EZ-Template) with extra modules for my team. everything below is what's different from stock EZ — for everything else see the [EZ docs](https://ez-robotics.github.io/EZ-Template/).

## branches

| branch | what it is | status |
|---|---|---|
| `main` | all the custom features below, on kernel 4.2.2, plus lemlib hardware 0.5.0 + units | emulator-verified, no hardware yet |
| `pros-4.2.2-enhancements` | stock ez-template on kernel 4.2.2 + the boot fixes, features added one at a time (see [its section](#whats-on-pros-422-enhancements)) | emulator-verified per feature |
| `pros-4.2.2-fixes` | just the kernel bump + the three boot fixes, sitting on upstream's `feature/pros-4.2.1` — the payload for the upstream pr. frozen | pr open |
| `462-additions-OLD` / tag `archive/462-additions` | the old kernel-4.1.1 line | archived |
| `pros-4.2.2-OLD` | pre-merge snapshot of main | archived |

## the 4.2.2 black screen, solved

the frozen-black-screen everyone hit on pros 4.2.x on real robots ([ez-template pr #309](https://github.com/EZ-Robotics/EZ-Template/pull/309)) was two stacked bugs, found with gdb against the emulator:

1. pros kernel 4.2.1/4.2.2 dropped the `.stack` section from its linker script while the kernel still starts the boot stack at `__stack` — so the stack silently overwrites the last globals in your program before user code runs. which globals get hit depends on link order, which is why everyone broke differently.
2. ez writes to sd files without null-checking `fopen`, and a null write is a data abort that pros silently skips — the task wedges, screen stays black.

both fixed on every branch here (plus a third: the example project's screen task printed before llemu existed, which corrupts memory on lvgl 9).

## features on `main`

all exposed through `EZ-Template/api.hpp`, namespace `ez`. none of the modules bind controller buttons on their own — things only read buttons inside functions you chose to call.

### auto pid tuner (`EZ-Template/tuner.hpp`)

relay-feedback auto tuning for drive/turn/swing/heading PIDs: a relay test measures the ultimate gain and period (averaged over 3 runs). this is then tuned into constants. kp is bisected until every move size passes — 12/24/48 in for drive, 45/90/135 deg for turns — without going over the overshoot limit. it monitors motor temps and battery the whole time, and every translating test stays inside a runway (`runway_set`, default 60 in, min 24) so it can run on a vex field.

heading has two paths: `tune_heading` relay-tests the heading hold while driving forward (gains capped against the drive result so straight driving can't jitter), or `set_heading_from_drive` derives conservative constants from the drive tune without moving.

```cpp
ez::PIDAutoTuner tuner(chassis);
tuner.runway_set(60.0);
tuner.tune_drive(12.0, 60);
tuner.set_heading_from_drive();
tuner.tune_turn(90.0, 60);
tuner.tune_swing(90.0, 60);
tuner.save_to_sd();
```

or use the controller menu — `tuner.interactive(master)` — LEFT/RIGHT selects (drive / turn / swing / heading / everything / tracker offset), A runs it, B saves and exits. "everything" runs drive, heading-derived-from-drive, turn, and swing — it does NOT run the heading relay test or the tracker offset; pick those individually. nothing calls `interactive()` for you — add it yourself (an auton selector entry is good).

`save_to_sd()` writes to `/usd/pid_constants.txt`; `ez::pid_constants_load(chassis)` restores everything on boot — pid constants, tracker offsets, and the imu scaler. register your trackers on the chassis before calling it.

### tracker offsets (part of the tuner)

measures every installed tracking wheel's distance-to-center in one spin sequence — vertical AND horizontal at once. spin about the center makes each wheel roll `offset * angle`, so five 360s give each wheel's offset, which gets applied and saved under its own sd label (`trk-l/r/b/f`). run it from the tuner menu ("trk offset", shows up when any tracker is installed) or directly:

```cpp
ez::tracker_offsets_measure(chassis);  // needs ~2 ft clear space, imu scaled first
```

### imu scale wizard (part of the tuner)

fixes gyro gain error (a 0.6% error is invisible in one turn, ~2 deg after a few auton turns — and it corrupts the tracker offset math, so run this FIRST):

```cpp
ez::imu_scale_measure(chassis, master);  // spin the robot 10 turns by hand vs a straight edge, press A
```

applies the scaler immediately and saves it to the sd card; restored every boot by `pid_constants_load`.

### dsr — distance sensor wall resets (`EZ-Template/dsr.hpp`)

uses the field walls as a ruler to fix odom drift. register a distance sensor per side with its offset from robot center; `correct()` figures out which wall each sensor faces and overwrites that axis of the pose. bad readings get rejected (low confidence, out of range, more than 15 deg off square, or an implied jump over 18 in — something in the way).

```cpp
ez::dsr::sensor_add(&back_distance, ez::dsr::Side::BACK, 7.0);
ez::dsr::sensor_add(&right_distance, ez::dsr::Side::RIGHT, 7.0);

double x = chassis.odom_x_get(), y = chassis.odom_y_get();
if (ez::dsr::correct(chassis.drive_imu_get(), x, y)) {
  chassis.odom_x_set(x);
  chassis.odom_y_set(y);
}
```

`ez::dsr::align_heading(x, y, heading)` gives the smallest turn that squares a sensor at the nearest wall.

### health checks (`EZ-Template/health.hpp`)

```cpp
ez::health::preflight(chassis, master);  // end of initialize()
```

pings the imu, every drive motor, odom trackers, and dsr sensors; prints each dead device with its port, rumbles the controller on failure, and returns — it reports, never blocks. run it after transport, battery swaps, and any time the robot acts weird: it turns "why isn't it moving" into a port number. lemlib devices can be registered with `ez::health::device_add(&dev, "name")`.

### screen rotation (`EZ-Template/display.hpp`)

```cpp
ez::screen_rotation_set(90);  // once, in initialize(), AFTER ez::as::initialize(). 0/90/180/270
```

for sideways/upside-down brain mounts. at 90/270 the selector rebuilds as a portrait copy of the llemu screen (same theme, long lines wrap, buttons forward to llemu's real ones); 180 is the stock screen flipped. touch works at every angle — lvgl transforms pointer input, and the flush path software-rotates each rendered band onto the panel (lvgl 9 dropped built-in rotation). don't call it in a loop — once.

### json autons (`EZ-Template/json_auton.hpp`)

> heads up: this module is nowhere near done. the format and executor are still changing a lot — don't build anything on it yet.

runs autons from `/usd/autons.json` — each shows up in the auton selector by name. pure-pursuit runs, boomerang arrival headings, reverse, per-point speed/waits, and a built-in `dsr` action that squares up and resets the pose. other actions go to a handler you set:

```cpp
ez::json_action_handler_set([](const std::string& a) {
  if (a == "intake_on") { intake.move(127); return; }
});
ez::json_register_selector(chassis);  // in initialize()
```

### lemlib hardware 0.5.0 + units

vendored for YOUR subsystem code (typed units, swappable encoder interface, clean connected/disconnected answers) — ez itself doesn't use it beyond the health registry. caveat: files that include lemlib motor headers can't also use okapi literals (`_in`, `_deg`, ...) — the suffixes collide; use plain doubles there.

## what's on `pros-4.2.2-enhancements`

stock ez-template + kernel 4.2.2, growing one verified feature at a time. stock readme and example project stay stock over there — this list is the changelog:

1. **boot fixes** — the `.stack` linker restore, null-checked sd writes, screen prints guarded until llemu exists
2. **screen rotation** — same as main's (portrait selector, all four angles, touch)
3. **health checks** — like main's but leaner: no dsr slice, and instead of the lemlib registry it takes ANY pros smart device — `ez::health::device_add(&intake, "intake")` covers motors off the drive and standalone sensors, and a wrong device in the right port reads as dead too
4. **imu scale wizard** — standalone (no tuner): `ez::imu_scale_load(chassis)` in initialize restores the saved scaler, `ez::imu_wizard_register(chassis)` adds "IMU Scale Wizard" to the auton selector — run it like any auton and follow the controller prompts

## thanks

huge thanks to Chris | 11342 for helping test many of these features on real hardware!

