# MathWorks Minidrone Competition – Round 1: Autonomous Line Follower

An autonomous line-following and landing algorithm for the **MathWorks Minidrone Competition (PX4 Multicopter)**, built with **MATLAB R2026b** and **Simulink** using a Model-Based Design workflow.

The drone takes off, follows a red line made of 2 to 7 straight sections (with corners from 30° to 330°), and lands softly on a red circle at the end of the track. The whole algorithm lives in **one MATLAB Function block** (`hopGuidance.m`) inside the competition model.

> **Team:** `<Dosapaglu>` 

---

## Table of contents

1. [The task and how it is judged](#1-the-task-and-how-it-is-judged)
2. [Requirements](#2-requirements)
3. [Repository contents](#3-repository-contents)
4. [How the algorithm works](#4-how-the-algorithm-works)
5. [Step-by-step guide (Stages 1 to 10)](#5-step-by-step-guide-stages-1-to-10)
6. [Parameters you may need to change](#6-parameters-you-may-need-to-change)
7. [Troubleshooting](#7-troubleshooting)
8. [Known limitations](#8-known-limitations)
9. [Results](#9-results)
10. [Credits and licence](#10-credits-and-licence)

---

## 1. The task and how it is judged

**Round 1 (simulation):** build a vision-based algorithm that makes a simulated PX4 quadcopter follow a red line and land on a red circle.

Judging priorities, in order:

1. **Code generation capability** of the model.
2. **Number of sections** successfully navigated in each track.
3. **Accuracy of the path**, including an accurate and soft landing on the circle.
4. **Time** taken to finish the track.

Track rules used by the judges:

- 2 to 7 sections; the angle between connected sections is between 30° and 330°.
- Track colour is red `#FF0000`.
- No crossing sections and no very close side-by-side sections.
- The arena is about 4 m x 4 m.

In Round 2 (hardware), each completed stage earns 1 point: takeoff, each section, soft landing, and accurate landing (at least 20% of the drone body on the circle).

---

## 2. Requirements

- **MATLAB R2026b** (the rules allow only this release).
- Toolboxes listed in the competition guide, including Simulink, Stateflow, Aerospace Blockset, UAV Toolbox, Computer Vision Toolbox, Image Processing Toolbox, Embedded Coder and MATLAB Coder.
- Support package: **UAV Toolbox Support Package for PX4 Autopilots** (choose "Setup later" if asked for hardware setup; it is not needed for Round 1).
- A Windows PC with a graphics card.
- The competition project, opened with `PX4MultiCopterPerceptionAndControlStart`.

---

## 3. Repository contents

```
.
├── README.md             this file
├── hopGuidance.m         the algorithm (paste into a MATLAB Function block)
├── docs/                 optional: screenshots of the wiring and Scope
└── tracks/               optional: saved waypoint files of your test tracks
```

**Important before you publish:**

- The competition model, the rules PDF and the code-generation guide are **MathWorks' material** (the model shows "Copyright 2026 The MathWorks, Inc."). Upload only your own algorithm file, documentation and test tracks, **not** the full project archive, unless the organisers allow it.
- Check whether the organisers want solutions kept private until judging is finished.
- Remove personal paths (for example `C:\Users\<name>\...`) from anything you upload.
- Do not commit build output: `slprj/`, `*_ert_rtw/`, `*.slxc`.

---

## 4. How the algorithm works

**Idea:** find the *start end* of the line first, fly there, follow the whole line to the end, then land on the circle. Finding the start first matters because every completed section earns a point.

Think of a person following a painted line with a small torch, dropping breadcrumbs behind them so they always know which way is "forward".

### Building blocks

1. **Seeing red.** Each pixel is tested with colour ratios (red much larger than green and blue), which is more robust to lighting than a fixed threshold. Red pixels are grouped into separate **blobs**: the line, the circle, and any other lines are different blobs.
2. **Pixels to metres (self-calibration).** After takeoff the drone makes two tiny test hops (8 cm north, then 8 cm east) and measures how the camera image shifts. This gives the camera-to-position mapping (`mapIdx`, 0 to 7) and the metres-per-pixel scale (`fovw`).
3. **Breadcrumb trail.** The drone remembers every place it has been. It aims at the red point of its own line that is **farthest from all breadcrumbs**. That point is always forward, even at a sharp corner. Only points within a few centimetres of the best one are averaged, so it aims at one arm and not between two.
4. **Start end or circle end.** At a line end the code counts how many times the line crosses an imaginary ring: 1 crossing means a line end, 2 or more means the middle or a corner. An end with no circle next to it is the **start**.
5. **Circle detection.** The circle is an isolated, round, compact blob of the right size relative to the line width. It is only accepted just past the end of the line, so a corner is never mistaken for it.
6. **Speed control.** On straight lines the drone aims further ahead (fast); in sharp turns it aims only a few centimetres ahead (slow and accurate).
7. **Remembered landing spot.** Once the whole circle is seen, its centre is converted to a floor position and remembered. During descent the drone keeps steering to that point, even when the circle fills the image.

### States (`dbg(1)` on the Scope)

| State | What the drone is doing |
|---|---|
| 0 | Takeoff to hover height; measure the line width |
| 7, 8 | Calibration hops (north, then east) |
| 9 | Return to the takeoff point and check whether it is already at the start end |
| 14 | **Survey:** climb high and look for a line end with no circle (the start) |
| 15 | Fly to that start end at survey height and re-check from above |
| 17 | Descend straight onto the start |
| 16 | Survey failed: come back down |
| 10 | Spiral search (no red visible at all) |
| 11 | Lock onto the line and work out where on the line the drone is |
| 1 | **Follow the line** with the trail and corner-aware speed |
| 3 | Line ended: creep forward a little and look for the circle |
| 12 | Turn around at a plain end (it must be the start) |
| 13 | Fallback: slow circle hunt |
| 4 | Centre over the circle |
| 5 | Descend softly |
| 6 | Landed |

`dbg(2)` is the route: `0` start not known, `1` reached the circle end first and is heading to the start, `2` start known and doing the full run. `dbg(4)` is the calibration mapping and `dbg(5)` is the estimated footprint `fovw`.

### Soft and accurate landing

- Descent speed is 0.40 m/s above 0.4 m and 0.12 m/s below it, with touchdown at 0.08 m.
- The drone only keeps descending while it is close to the remembered circle centre.

---

## 5. Step-by-step guide (Stages 1 to 10)

Do the stages in order and do not move on until the **Check** of each stage passes. The original guide has ten stages; Stage 9 is the code-generation check and Stage 10 is the submission.

### Stage 1: Open the project

1. Start **MATLAB R2026b**.
2. In the Command Window run:
   ```matlab
   PX4MultiCopterPerceptionAndControlStart
   ```
3. Wait until the Project panel shows folders such as `controller`, `mainModels` and `scripts`, and the main model opens.
4. Click **Run** once to see the default behaviour, then stop it.

**Check:** the 3D viewer opens and the drone flies.

### Stage 2: Make a backup

1. Open the **PROJECT** tab of the **MATLAB main window** (not the Simulink window), then **Share > Archive**.
2. Save as `backup_original.mlproj` outside the project folder.

If you cannot find **Share**, make sure the project is open; if the toolstrip is cut off, maximise the window.

### Stage 3: Test the algorithm without Simulink

An offline test script (`testHopOffline.m`) can run the algorithm against a synthetic camera and a simple drone model. It is a sanity check of the logic only; it does not replace the Simulink run.

1. Put the algorithm file and the test script in one folder outside the project, for example `C:\DroneTest`.
2. In MATLAB: `cd C:\DroneTest`, then run `testHopOffline()`.
3. It prints state changes and ends with a line such as `RESULT landed=1 ... distance to circle centre=...`.

**Check:** `landed=1` and a small distance to the circle centre. (Keep these test files out of the project subfolders; see Stage 10.)

### Stage 4: Wire the block

Open **PerceptionAndGuidance > Image processing and Guidance** (the green block) and build:

**Inputs**

1. `CurrentPosition` (port 1) into a **Demux** with 4 outputs (X, Y, Z, Yaw).
2. Demux outputs **X, Y, Z** into a **Mux** with 3 inputs; the Mux output goes to **`pos`**.
3. Demux output **Yaw** goes to **`yaw`**.
4. `Camera` (port 2) goes **directly** to **`img`** (do not route it through "get camera data").

**Outputs**

1. `xd, yd, zd, yawd` into the existing 4-input Mux, in that order, then through the existing `single` block to `Trajectory`.
2. `landed` to a **Terminator**.
3. `dbg` to a **Scope**.

**Set exact sizes and types** (open the MATLAB Function block, then **Edit Data**):

| Name | Size | Type |
|---|---|---|
| `img` | `[120 160 3]` | `uint8` |
| `pos` | `3` | `double` |
| `yaw` | `1` | `double` |
| `xd`, `yd`, `zd`, `yawd` | `1` | `double` |
| `landed` | `1` | `boolean` |
| `dbg` | `[5 1]` | `double` |

Use the **MATLAB Function** block from the **Simulink** library (not the HDL Coder one, and not "Interpreted MATLAB Fcn").

**Check:** press **Ctrl+D** (Update Diagram) with no red errors. An "index out of bounds" message usually means an input port is unconnected or the sizes are not set.

### Stage 5: Paste the code

1. Open the MATLAB Function block, delete everything inside.
2. Paste the contents of `code` and save.
3. Press **Ctrl+D** again.

**Check:** no errors.

### Stage 6: Set the sample time and measure the line width

- **`Ts` (sample time).** The block runs every `Ts` seconds. Set `Ts` at the top of the code to the real sample time of the block. To find it, use the model's sample-time legend (the Debug tab, information overlays) or the solver step size in Model Settings. In the author's model the guidance signals showed `D5 = 0.2 s`, so `Ts = 0.2`. In this version all waits are written in seconds and converted using `Ts`, so **only `Ts` needs changing**.
- **`lineWpx` (line width).** This is only a fallback: the code measures the line width itself from the camera image. Leave it unless detection clearly fails.

### Stage 7: First real run

1. Click **Run**.
2. Watch the **Scope**: the first line is the state number (see the table in section 4).
3. A normal run goes `0, 7, 8, 9` (takeoff, calibration, return), then follows the line (`1`), then `3`, `4`, `5`, and ends in `6`. If the start is not under the drone you may also see `14, 15, 17` (survey) or `10` (spiral search).

**Check:** the drone reaches state 6 on the circle. If `fovw` (`dbg(5)`) stays exactly `0.45`, the calibration did not succeed and defaults are being used.

### Stage 8: Test on many tracks

Build several tracks with the **Track Builder** and run each one.

1. In the Project's `scripts` folder run `openTrackBuilder.m`.
2. Set the colour to `FF0000`.
3. Click **Clear**, then click in the arena to add waypoints in order (or edit the table if it allows).
4. Click **Show Landing Marker**.
5. Click **Save Waypoints** (save outside the project subfolders).
6. Click **Generate Ground Pattern**, then **Update Virtual World**.
7. Press **Run** in the Simulink model and check that the 3D viewer shows the track you drew.

Suggested test tracks, as (East, North) waypoints. These are suggestions; check each one against the rules (2 to 7 sections, angles between 30° and 330°, no crossings) in the Track Builder preview.

| # | Name | Waypoints | Why |
|---|---|---|---|
| 1 | 2 sections, 90° | (2.0, 0.5), (2.0, 2.0), (3.2, 2.0) | simplest case |
| 2 | 3 sections, two right angles | (1.0, 0.5), (1.0, 2.0), (2.5, 2.0), (2.5, 3.3) | right then left |
| 3 | Sharp hairpin (about 35°) | (0.8, 0.8), (2.4, 3.2), (2.43, 1.7) | hardest corner type |
| 4 | Left then right turns | (3.4, 0.6), (3.4, 2.2), (1.9, 2.2), (1.9, 3.4) | both turn directions |
| 5 | 5 sections, zigzag | (0.5, 0.5), (0.5, 1.5), (1.5, 1.5), (1.5, 0.7), (2.5, 0.7), (2.5, 2.5) | many corners |
| 6 | 7 sections, spiral | (0.5, 0.5), (3.5, 0.5), (3.5, 3.5), (0.5, 3.5), (0.5, 1.3), (2.7, 1.3), (2.7, 2.7), (1.3, 2.7) | maximum length |

Also recreate one of the sample tracks from the rules document.

Keep a results table (see section 9). **Do not change settings between tracks**; after any code change re-run **all** tracks.

**Check:** every section finished and the drone landed on the circle for all tracks.

### Stage 9: Code generation check (judged first)

Follows the organisers' "Algorithm Code Generation Check" guide. It checks only the copied algorithm logic, in a separate model.

1. In the green **Image processing and Guidance** subsystem, select your algorithm and its connecting lines, **excluding** "get camera data". Copy with **Ctrl+C**.
2. Press **Ctrl+N** for a blank model, paste with **Ctrl+V**, and save as `StudentAlgorithmCheck.slx` **outside the project folder**.
3. Connect Inports and Outports so that **dimensions, data types and sample times match the original**.
4. Press **Ctrl+E** (Model Settings), open **Hardware Implementation**, set **Hardware board** to **Raspberry Pi** and click **Apply**.
5. In **Solver**, choose **Fixed-step** and **discrete (no continuous states)**, set the intended step size and click **Apply**.
6. Open **Apps > Embedded Coder**. In the **C CODE** tab choose **Build > Generate Code** (source only; no Raspberry Pi is needed).
7. Open the **Diagnostic Viewer**. Success means it reports completed code generation. If there is an error, select the **first** error, fix the cause, and generate again.

Repeat this check after **every** algorithm change.

### Stage 10: Submit

1. Open **PROJECT > Project Issues** and fix what is listed. Warnings about missing files can also appear on the untouched template; compare with a fresh copy.
2. Make sure you added **no** Simulink models or MATLAB files inside the project subfolders. Extra files go in the main project folder. Remove code-generation output and the check model from the project.
3. Save the Simulink model (Ctrl+S) so there is no unsaved-changes asterisk.
4. **Share > Archive**, choose **Export Profile: Whole Project**, and name the file `<Dosapaglu>_MDC<India>_<2026>.mlproj` (for example `DroneSquad_MDCIndia_2026.mlproj`).
5. The team captain submits it with **Submit your entry** on the competition page, using the registered email.

### Opening the archive on another computer

Install MATLAB R2026b with the same toolboxes and the PX4 support package, then double-click the `.mlproj` in MATLAB's Current Folder panel (or **Home > Open**). Extract into a **new, empty** folder and open the main model from the Project panel, so the search path is correct. Do not just unzip it and open one model by hand.

---

## 6. Parameters you may need to change

All parameters are at the top of `code`.

| Parameter | Default | Meaning |
|---|---|---|
| `Ts` | `0.2` | Block sample time in seconds (set to your block's real value) |
| `hoverAlt` | `1.2` | Line-following altitude [m] |
| `useSurvey` | `true` | Climb high to find the start end; set `false` if the drone must not climb that high |
| `surveyAlt1`, `surveyAlt2` | `4.0`, `5.5` | Survey heights [m] |
| `searchAlt` | `1.8` | Spiral search height [m] |
| `followLead` | `0.35` | Aim distance on straight lines [m] (higher = faster) |
| `cornerLead` | `0.07` | Aim distance in sharp turns [m] (lower = more accurate) |
| `autoCal` | `true` | Run the two-hop calibration; set `false` to use `mapFixed` and `fovwFixed` and save a few seconds |
| `mapFixed`, `fovwFixed` | `0`, `0.45` | Values to use when `autoCal = false` (copy from `dbg(4)` and `dbg(5)`) |
| `cLo`, `cHi` | `1.4`, `2.8` | Circle size range in line widths |
| `centTolPx` | `6` | Centring tolerance before descent [px] |
| `descFast`, `descSlow` | `0.40`, `0.12` | Landing descent speeds [m/s] |
| `landAlt` | `0.08` | Touchdown altitude [m] |

---

## 7. Troubleshooting

| Symptom | What to check |
|---|---|
| "Model Not Found" on `flightControlSystem` | The model was opened outside the project. Close it and reopen through the project (`currentProject`, or reopen the `.mlproj`) |
| Ctrl+D says an array index is out of range | An input port is unconnected or sizes in **Edit Data** are not set |
| The drone starts at a different place on every track | Start points normally differ between tracks (the takeoff is at the first waypoint). Check the survey states on the Scope (`14, 15, 17` or `10`); try `useSurvey = false` for a test |
| Drone flies away from the line | The camera mapping may be wrong: check `dbg(4)` and `dbg(5)`; if `fovw` is exactly `0.45` calibration failed |
| It never switches to centring on the circle | Check the circle and line sizes in the camera image and `cLo`, `cHi` |
| Waits seem too long or short | `Ts` does not match the block's real sample time |
| Warnings in **Project Issues** about missing files | Compare with a fresh copy of the template; remove generated code and test files from the project |
| Code generation errors | Open the Diagnostic Viewer, fix the **first** error and generate again |

---

## 8. Known limitations

- Results depend on the simulator's camera and on the real floor texture used for calibration. If calibration fails, defaults are used and the drone may behave worse.
- The survey needs the start end to be inside the camera footprint at survey height; otherwise the drone falls back to the spiral search.
- At survey height the circle is only a few pixels wide, so telling the start end from the circle end is less reliable there.
- The algorithm has not been validated on real hardware (Round 2).
- The code-generation check must be re-run on every version; passing it covers only the copied logic.

---

## 9. Results

Fill in your own results (do not publish numbers you have not measured).

| Track | Sections finished (of N) | Landed on circle? | Time [s] | Notes |
|---|---|---|---|---|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |
| 5 | | | | |
| 6 | | | | |

---

## 10. Credits and licence

- Competition, model and rules: **MathWorks Minidrone Competition**. The competition materials belong to MathWorks.
- Algorithm and documentation: `<your team and member names>`.
- Licence: `<choose a licence for your own code, for example MIT>`.
