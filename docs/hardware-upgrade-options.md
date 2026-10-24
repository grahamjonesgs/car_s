# car\_s hardware upgrade options

A copy of the [live doc](https://claude.ai/artifact/RQFt9Y8Arrv8kGhYCeobKW), exported on 2026-10-24. The live doc is the one to edit; re-export it here after big changes.

## Summary

The best first step is a ring of low ToF distance sensors plus bump switches, then a floor-facing optical flow sensor: together under about £50, they fix the two weaknesses that showed up most in testing.

The car's weaknesses today:

- **Lidar blind spots.** It sees one flat slice about 10 cm above the floor, and nothing closer than 15 cm. Cables, shoe edges, low rails, tabletops and chair seats are invisible, which probably caused Nav2's "Failed to make progress" stalls in the big room.
- **Open-loop odometry on a skid steer.** `/odom` counts commanded step pulses, with no encoders. It can't see wheels skidding in turns, stalls or missed steps. The gyro and lidar scan matching are the only real measures of motion.
- **Stepper limits.** Torque drops with speed and battery voltage, missed steps go unnoticed, and the vibration reaches the IMU.

Costs are approximate (from memory, not looked up), in pounds.

| Priority | Upgrade | Fixes | Cost (approx.) | Work needed |
| --- | --- | --- | --- | --- |
| 1 | Low ToF or ultrasonic sensor ring (4–6) | Low obstacles, the under-15 cm blind spot, glass (ultrasonic) | £20–30 | I²C wiring, a small node, Nav2 range layer config |
| 1 | Bump switches | Anything every other sensor missed | £5 | GPIO wiring, motor stop in the controller |
| 2 | Optical flow sensor (PMW3901 / PAA5100) | Real ground speed: slip, stalls, missed steps | £15–25 | SPI wiring, a node, EKF config |
| 3 | Battery voltage/current monitor (INA219 / INA226) | Weak motors on a low battery, no warning | £5–10 | I²C wiring, a small node |
| 3 | TMC2209 stepper drivers | Noise and vibration; later, stall detection | £20–40 for 4 | Plug-in swap, microstep code change, recalibration |
| 4 | Better lidar (LD19 / RPLIDAR C1) | Sparse scans at range, slow 7 Hz rate | £60–90 | New driver package, URDF frame |
| 4 | Depth camera (OAK-D Lite / RealSense D435) | 3D obstacles: table edges, overhangs, clutter | £100–300 | USB power, on-Pi processing, Nav2 layer |
| 5 | Pi camera | Live view, AprilTags, object detection (not navigation) | £25 | libcamera on Ubuntu, streaming setup |

## Obstacle sensing

Cheap sensors near the floor cover the lidar's blind spots; a depth camera is the bigger fix for 3D obstacles, and a Pi camera adds little for navigation.

### Low distance sensor ring

- **What:** 4–6 short-range sensors round the bumper, below the lidar. VL53L1X ToF sensors (about 4 m range) or ultrasonic sensors.
- **Fixes:** obstacles below the lidar's scan height, the under-15 cm blind spot, and glass and mirrors (ultrasonic only; the lidar misses them).
- **Integration:** each publishes `sensor_msgs/Range`, which Nav2's range sensor layer (`nav2_costmap_2d::RangeSensorLayer`) adds to the costmap, with no planner code.
- **Catch:** VL53L1X sensors all share I²C address 0x29. Either give each one an XSHUT GPIO and set addresses at start-up, or use a TCA9548A I²C multiplexer.

### Bump switches

- **What:** micro-switches behind a front (and rear) bumper plate.
- **Fixes:** collisions with anything every other sensor missed.
- **Integration:** wire to free GPIOs. The controller should stop the motors itself on a bump, and publish it so Nav2 can back off.

### Depth camera

- **What:** a USB depth camera such as a Luxonis OAK-D Lite or Intel RealSense D435.
- **Fixes:** obstacles in 3D in front of the car: table edges, overhangs, low clutter, chair legs as solid objects. Also gives a colour image.
- **Integration:** `depthimage_to_laserscan` makes an extra scan, or a voxel layer in the local costmap uses the 3D points. No new code.
- **Catches:** raw depth is too much for WiFi, so convert it on the Pi first. Lidar and camera on the Pi 4's USB may need a powered hub.

### Pi camera

- **What:** a CSI camera module.
- **Gives:** a live view while driving (the Pi 4 has hardware H.264), AprilTag or ArUco markers for docking or correcting AMCL, object detection on the Beelink's GPU or NPU.
- **Doesn't give:** distance, so no help with obstacles. Visual odometry would add little over the calibrated gyro and lidar.
- **Catches:** on Ubuntu 24.04 it goes through `libcamera` and `camera_ros`, and Camera Module 3 support there has been patchy; the older v2 (IMX219) is safer. Mount it out of the lidar's scan plane.

## Odometry

An optical flow sensor is the one upgrade that measures what the car really does over the ground, which step counting and encoders both miss on a skid steer.

### Optical flow sensor

- **What:** a floor-facing motion sensor, like a mouse sensor but for larger distances, such as the PMW3901 or PAA5100. Talks SPI, and SPI0's pins are used by the microstep lines today, so they need moving first (see Pi 4 constraints).
- **Fixes:** it measures real forward and sideways motion, so it sees skids in turns, stalls and missed steps.
- **Integration:** a small node publishing a velocity, which the EKF fuses as a second source next to `/odom`. Drift should drop, especially in rotation.
- **Catch:** it needs a textured surface, and can struggle on glossy, very dark or changing floors. Test it on each floor in the house before relying on it.

### Wheel encoders

- **What:** magnetic encoders (such as AS5600) on the motor shafts, or closed-loop stepper motors.
- **Fixes:** stalls and missed steps.
- **Doesn't fix:** wheels skidding sideways in turns, which is the bigger error on a skid steer. So lower value here than optical flow.

## Drive and power

TMC2209 drivers can probably plug into the existing board in standalone mode; stall detection needs extra wiring. A battery monitor is a cheap reliability win.

### TMC2209 stepper drivers

The driver board looks like an Arduino-style CNC Shield v3: StepStick sockets X, Y, Z and A, with shared M0/M1/M2 and ENABLE. The M0–M2 naming suggests DRV8825 drivers. Open question: confirm the board and the chips on it.

**Standalone mode: no new board needed.** StepStick-format TMC2209s (BigTreeTech, FYSETC) fit the same sockets, and STEP, DIR, ENABLE and the motor pins match. The gain is much quieter, smoother running and less vibration for the IMU.

- **Microsteps differ.** Standalone TMC2209s use two pins for 1/8, 1/16, 1/32 or 1/64 (interpolated to 1/256), not the DRV8825's 1–32 table. Update the controller's `motor_steps` table and `key_cmd`'s `0`–`5` keys, then rerun `tools/odom_cal.py`. Otherwise odometry is out by a factor of 2 or more.
- **Set the current** with each module's potentiometer, using its own Vref formula.
- **Voltage:** rated to about 29 V. Check the battery.
- **Orientation:** some modules have a reversed pin layout. Match EN and GND to the board, or the driver can be destroyed.
- **Directions:** recheck with the wheels off the ground; fix a reversed motor in the controller's DIR logic.

**UART mode (StallGuard stall detection): extra wiring, maybe a new board.** Each driver's UART pin must reach the Pi's serial port, and the shield doesn't route it. In UART mode the MS pins set each driver's address, but the shield ties them together, so all four would share one address. Fix by cutting traces, an adapter, or a board with separate MS pins per socket.

### Battery monitor

- **What:** an INA219 or INA226 voltage and current sensor on I²C.
- **Fixes:** steppers lose torque as the battery drops, causing more missed steps with no warning.
- **Use:** slow down or return to the start on a low battery, and see what the motors draw.

## Lidar, and what's not worth it

A newer lidar would make maps and exploration more robust; a better IMU or GPS would not help this car.

### Better lidar

- **What:** an LDROBOT LD19 or RPLIDAR C1, around £60–90.
- **Fixes:** the A1 gives 360 points at about 7 Hz, so distant walls come out as sparse dots. In the big room this made the map's edge one giant frontier round the car, which confused exploration until `explore_lite` was patched. Denser, faster scans give solid walls and steadier scan matching.
- **Work:** a different ROS driver package, and a new `laser_joint` in the URDF. Check which way its 0° faces when mounting (the A1 is mounted backwards).

### Not worth it

- **A better IMU** (such as BNO085): after the start-up calibration, the MPU6050's gyro bias at rest is about 0.0001 rad/s, and the EKF heading drifted 0.13° in 40 s standing still.
- **GPS:** doesn't work indoors.
- **Using the magnetometer:** it reads about twice the Earth's field because of the steppers. Moving it away from the motors and calibrating it might make it usable, but indoors it would still be disturbed by the building.

## Recommended order and Pi constraints

Start with the cheap safety sensors, then fix the odometry, and check pins before buying: SPI and the main UART are already taken.

1. Bump switches and a low VL53L1X ring, with Nav2's range layer.
2. Optical flow sensor, fused in the EKF. Free SPI0 first (below), and test on every floor type.
3. Battery monitor.
4. TMC2209s in standalone mode, with the microstep table updated and odometry recalibrated.
5. Then, depending on what still goes wrong: a better lidar (sparse maps), a depth camera (3D obstacles), TMC2209 UART mode (stall detection).
6. A Pi camera any time, for viewing and markers.

### Pi 4 constraints

| Resource | In use | Free / options |
| --- | --- | --- |
| GPIO | Motors 6, 12, 13, 15, 16, 18, 19, 26; ENABLE 14; microstep 9, 10, 11; LEDs 23, 24; I²C 2, 3 | 4, 5, 7, 8, 17, 20, 21, 22, 25, 27. Avoid 0 and 1 (HAT ID EEPROM) |
| I²C bus 1 | IMU 0x68, magnetometer 0x0D | INA219/226 at 0x40 fits. VL53L1X all default to 0x29: XSHUT pins or a TCA9548A multiplexer |
| SPI0 | Its pins 9, 10, 11 are used as the microstep lines | Move M0–M2 to free GPIOs (e.g. 5, 22, 27) in the controller and wiring, freeing SPI0 for an optical flow sensor |
| UART | The main UART's pins 14 and 15 are ENABLE and a DIR line | The Pi 4's extra UART3 on GPIO 4 and 5 (`dtoverlay=uart3`), for TMC2209 UART mode |
| USB | Lidar (CP210x) | A depth camera with the lidar may need a powered hub; the Pi 4 shares about 1.2 A across its USB ports |
| PWM | Both hardware PWM channels drive the motors; audio must stay off | None spare |

## Parts list (Digi-Key South Africa)

The core upgrades (Qwiic distance ring, battery monitor, bump switches) come to about R2,400 for 23 parts and a pack of jumper wires, in stock (cable stock not checked); the optical flow sensor and TMC2209 SilentStepSticks are not currently orderable.

Prices and stock as shown on digikey.co.za on 21 Oct 2026, unit price at quantity 1 in rand. Check whether they include VAT, and add shipping.

### Qwiic wiring plan

Pi I²C bus 1 → Qwiic SHIM → INA228 (0x40) → Qwiic Mux (moved to 0x71) → one VL53L1X on each of mux channels 0–5, and a MaxBotix sonar on each of channels 6 and 7. The mux is needed because every VL53L1X is 0x29. It also lets both sonars keep their default 0x70, since only one channel is open at a time. Six distance sensors: front centre, front-left and front-right at 45°, left side, right side, rear centre. Two sonars: front, angled slightly left and right. The existing IMU (0x68) and magnetometer (0x0D) share the bus with no address clashes.

**Connecting the mux to the Pi:** the Qwiic SHIM pushes onto the Pi's GPIO header (2.54 mm pins) as a friction fit, with no soldering. It takes 5 V from the header and has its own 3.3 V regulator, so it doesn't load the Pi's 3.3 V pin that powers the IMU. From the SHIM's Qwiic socket, one standard Qwiic cable runs to the INA228, and another from the INA228's second socket to one of the mux's two main-bus sockets. Both are among the ten Adafruit 5384 cables below. Check that the IMU's jumper wires on pins 1, 3 and 5 still grip with the SHIM underneath.

**Qwiic connector type:** JST SH, 4 pins at 1.0 mm pitch, polarised so it only fits one way. Board sockets are JST SM04B-SRSS-TB. Cables have the same plug at both ends, with black = GND, red = 3.3 V, blue = SDA, yellow = SCL. Qwiic is 3.3 V only. STEMMA QT (Adafruit) is the same connector.

**Mux address:** bridge the mux's A0 address jumper (solder pads marked ADR on the back) to move it from 0x70 to 0x71, so it doesn't clash with the sonars.

### Core upgrades

| Item | Part | Qty | Unit (R) | Total (R) | Stock | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Qwiic adapter for the Pi | [SparkFun 15794 Qwiic SHIM](https://www.digikey.co.za/en/products/detail/sparkfun-electronics/15794/11206004) | 1 | 31.56 | 31.56 | 335 | Slips over GPIO pins 1–6; the IMU's wires on those pins can stay |
| I²C multiplexer | [SparkFun 16784 Qwiic Mux, TCA9548A](https://www.digikey.co.za/en/products/detail/sparkfun-electronics/16784/12714950) | 1 | 242.93 | 242.93 | 464 | 8 channels: 6 used, 2 spare |
| Distance sensors | [Adafruit 3967 VL53L1X, STEMMA QT](https://www.digikey.co.za/en/products/result?keywords=adafruit%203967) | 6 | 241.96 | 1,451.76 | 495 | Half the price of SparkFun 14722 (R484.73), same chip, 4 m range |
| Battery monitor | [Adafruit 5832 INA228, STEMMA QT](https://www.digikey.co.za/en/products/result?keywords=adafruit%205832) | 1 | 241.96 | 241.96 | 1,120 | 15 mΩ shunt on board: up to 10 A and 85 V. Wired in series with the battery's + lead |
| Qwiic cables, 300 mm | [Adafruit 5384](https://www.digikey.co.za/en/products/result?keywords=adafruit%205384) | 10 | 20.23 | 202.30 | not checked | 8 needed (SHIM→INA228→mux, mux→6 sensors) + 2 spare |
| Bump switches | [Omron SS-5GL2, roller lever](https://www.digikey.co.za/en/products/detail/aratas-formerly-omron-components/SS-5GL2/137204) | 4 | 42.24 | 168.96 | 9,259 | Two front corners, two rear; wired to free GPIOs, not Qwiic |
| Jumper wires, female–female | [Adafruit 266, 40 × 150 mm](https://www.digikey.co.za/en/products/detail/adafruit-industries-llc/266/5629427) | 1 | 63.93 | 63.93 | 195 | Switches, optical flow SPI, moving M0–M2 |
| **Core total** |  | **23 + 1 pack** |  | **2,403.40** |  |  |

### Glass and mirrors (ultrasonic)

The VL53L1X uses infrared light like the lidar, so it misses clear glass and angled mirrors. Two sonars cover the front.

| Item | Part | Qty | Unit (R) | Total (R) | Stock | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Ultrasonic sensors | [MaxBotix MB1242-000, I2CXL-MaxSonar-EZ4](https://www.digikey.co.za/en/products/detail/maxbotix-inc/MB1242-000/7896782) | 2 | 646.57 | 1,293.14 | 25 | I²C, 20–765 cm (closer reads as 20 cm), 39° beam, up to 40 Hz. Default address 0x70. Read them one after the other, so one sonar doesn't hear the other's ping |
| Qwiic-to-jumper cables | [SparkFun 22726, Qwiic to female 0.1" jumpers, 165 mm](https://www.digikey.co.za/en/products/detail/sparkfun-electronics/22726/21443057) | 2 | 32.21 | 64.42 | 254 | Mux channels 6 and 7 to each sonar's header |
| Male header strip | [Sullins PRPC040SAAN-RC, 40 pins, 2.54 mm](https://www.digikey.co.za/en/products/result?keywords=PRPC040SAAN-RC) | 1 | 19.91 | 19.91 | not shown | Break off and solder 4-pin pieces onto each sonar's GND, V+, SDA and SCL pads |
| **Ultrasonic total** |  | **5** |  | **1,377.47** |  |  |

### Mounting on the metal base

The bottom deck is a black metal plate about 5–6 cm off the floor, below the lidar's scan plane, so the distance sensors and bump switches all go there. Use nylon screws, nuts and standoffs throughout, so no board's pads or underside can short on the metal. This hardware adds R662.45 to the cart.

- **ToF sensors.** Each board stands upright on two small brass L-brackets. One leg of each bracket screws to a hole in the plate (M3 nylon screw and nut), and the other holds the board through its M2.5 holes. Put the lens right at the plate edge, so the plate and wheels stay out of the 27° field of view.
- **Sonars.** Mount them the same way at the front edge, turned about 10–15° left and right. At this height the floor may echo; if it does, tilt them up a few degrees.
- **Mux and INA228.** These lie flat on the plate on 6–10 mm nylon standoffs: M3 for the Mux, M2.5 for the INA228.
- **Bump switches.** Lay one flat at each corner, with the roller lever sticking out past the curved edge. Each takes two M2 × 12 nylon screws and nuts, because the switch's mounting holes are too small for M2.5.
- **Holes.** Most of the plate's holes are about 3 mm. Where none lines up, drill 3.2 mm.

| Item | Part | Qty | Unit (R) | Total (R) | Stock | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| L-brackets | [Keystone 633](https://www.digikey.co.za/en/products/result?keywords=36-633-ND), brass, 3.56 mm holes | 20 | 3.74 | 74.78 | 88,744 | 2 per ToF sensor (12) and per sonar (4), plus 4 spare |
| M2.5 nylon kit | [Adafruit 3299](https://www.digikey.co.za/en/products/detail/adafruit-industries-llc/3299/6596885) | 1 | 274.33 | 274.33 | 325 | Screws, nuts and standoffs: sensors to brackets, INA228 standoffs |
| M3 nylon kit | [Adafruit 4685](https://www.digikey.co.za/en/products/result?keywords=adafruit%204685) | 1 | 274.33 | 274.33 | 68 | Brackets to the plate, Mux standoffs |
| M2 × 12 screws | [Essentra 50M020040P012B](https://www.digikey.co.za/en/products/result?keywords=50M020040P012B), nylon | 10 | 1.93 | 19.26 | not checked | Bump switches, 2 each + 2 spare |
| M2 nuts | [Essentra 04M020040HN](https://www.digikey.co.za/en/products/result?keywords=04M020040HN), nylon | 10 | 1.98 | 19.75 | 50,115 | For the M2 screws |
| **Mounting total** |  | **42 + 2 kits** |  | **662.45** |  |  |

### Not available right now

| Item | Part | Qty | Unit (R) | Total (R) | Stock | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Optical flow sensor | [Pimoroni PIM453 PMW3901](https://www.digikey.co.za/en/products/detail/pimoroni-ltd/PIM453/10246386) | 1 | 421.22 | 421.22 | 0, no backorders | SPI, not Qwiic. Only a 102-unit price shown. No other optical flow board is active on Digi-Key ZA; the PAA5100 isn't listed |
| Stepper drivers | [ADI TMC2209SILENTSTEPSTICK](https://www.digikey.co.za/en/products/result?keywords=TMC2209SILENTSTEPSTICK) | 4 (+1 spare) | 249.89 | 999.56 | 0, restock 15 Oct 2026 overdue | StepStick format, fits the existing sockets. Adafruit 6121 (R144.85, 414 in stock) is a different breakout, probably not socket-compatible |

### Bigger options

| Item | Part | Qty | Unit (R) | Total (R) | Stock | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Better lidar | [DFRobot DFR1138, RPLIDAR series](https://www.digikey.co.za/en/products/detail/dfrobot/DFR1138/23028657) | 1 | 1,132.92 | 1,132.92 | 94 | 5–1,200 cm, UART. Matches the RPLIDAR C1, but the page doesn't name the model: confirm before ordering |
| Depth camera | [RealSense D435 (961448)](https://www.digikey.co.za/en/products/detail/realsense/961448/9926002) | 1 | 5,401.58 | 5,401.58 | 263 | The camera-only 962305 is R5,219.50. Needs the powered USB 3 hub below |
| Powered USB 3 hub | [Raspberry Pi USB 3 Hub (PiShop)](https://www.pishop.co.za/store/raspberry-pi-usb-30-four-port-hub) | 1 | 239.90 | 239.90 | 68 | 4 × USB 3.0 into one Pi 4 USB 3 port. Optional USB-C power input, fed 5.1 V from the buck by a USB-C pigtail: cut the USB-A end off PiShop's 2.5A USB-A to USB-C cable (R64.89, 11 in stock) and use its red and black wires. Alternative: Waveshare USB 3.2 Gen1 Hub HAT, R349 at DIYElectronics, but it sits over the GPIO header |
| Pi camera | [Seeed 114992267, IMX219 77°](https://www.digikey.co.za/en/products/detail/seeed-technology-co-ltd/114992267/12396961) | 1 | 160.23 | 160.23 | 25 | Same sensor as the Pi camera v2 |

### Other project

| Item | Part | Qty | Unit (R) | Total (R) | Stock | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Qwiic adapter for the Pi | [SparkFun 15794 Qwiic SHIM](https://www.digikey.co.za/en/products/detail/sparkfun-electronics/15794/11206004) | 6 | 31.56 | 189.36 | 335 | Not for the car; in addition to the one in Core upgrades (7 in all) |

Not sourced here: SparkFun's Qwiic ultrasonic boards are obsolete there; the MaxBotix sonars above replace them.
