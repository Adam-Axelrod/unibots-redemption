# Battery

The pack that came with the Yahboom MicroROS-Pi5 car. It plugs into the
control board, which powers everything else, including the Pi.

## Specs

From Yahboom's documentation, not measured by us:

| | |
|---|---|
| Chemistry | Li-ion, 2x 18650 cells in series (2S) |
| Nominal | 7.4 V |
| Full | 8.4 V |
| Empty (cut-off) | 5.6 V |
| Capacity | 2000 mAh, about 15 Wh |
| Rated / max discharge | 15 A / 20 A |
| Protection | overcharge, over-discharge, over-current, short circuit |
| Storage | about 7 V |

## Runtime

Plenty for a match. A match is 3 minutes. Even assuming 30 W for the Pi,
motors and lidar together, which is pessimistic, 15 Wh lasts about 25 minutes.
Expect several matches per charge, but start each match with a full pack: the
motors are noticeably weaker as the voltage drops, which changes how far the
robot drives for the same command.

## Rules that apply

From the [rulebook](../unibots-rulebook-2026-27-v1.0.pdf); these are
checked at inspection:

- **R13** The battery must be held on securely but be removable, e.g. velcro,
  removable zip ties, or a latch.
- **R14** It must be protected from puncture. **Li-ion cells must not be
  visible from outside the robot**, unless through a transparent material. So
  the chassis needs an enclosed pocket for the pack.
- **R18** Store it in a fire-safe or Li-po bag when not in use. Chargers and
  wires must not be frayed, and the pack must not be swelling.
- **R19** Never charge it unattended. Charge on a fire-safe surface (fire brick
  or concrete block), with a charger that has been inspected. Bring the charger
  to inspection.

## Charging and care

- Charge through the **DC 8.4 V port on the control board**; the battery
  doesn't need to come out.
- The charger's light is **red while charging, green when full**. Unplug it
  soon after it turns green.
- Store it at about 7 V if it won't be used for a while.
- Check before an event:
  - no swelling, dents or torn wrap
  - voltage with a multimeter: about 8.4 V when full, never below about 6 V
- If it swells, gets hot when idle, or smells, stop using it.

## Monitoring from software

The control board measures the voltage and publishes it on `/battery` once a
second, as **volts x 10** (`std_msgs/UInt16`): `75` means 7.5 V. Check with
`ros2 topic echo /battery --once`. Above about 80 is nearly full; below about
65, charge it. The board also sounds its buzzer when the battery is low.

## To measure

Still unknown, needed to design the chassis pocket:

- Pack dimensions (L x W x H) and weight
- Connector type
