# maps/ — vendored static occupancy map

## What is here

| file | sha256 | bytes |
|---|---|---|
| `carter_warehouse_navigation.yaml` | `6898496a872b7831a91e867eb08b67eda07bd91c6f2313053268d7a591e6fd70` | 156 |
| `carter_warehouse_navigation.png` | `dd2f5e382a5f331866becaeaffb391a7e46b595873bf25c9cbb4e280ec261b8e` | 4563 |

Both files are **byte-identical to upstream** — that is the point of vendoring: run
`sha256sum` on them and the digests above must come back, which is a check anybody can
repeat without trusting this README. Provenance therefore lives here, *next to* the
files, instead of in a header comment that would change their bytes.

## Where they come from

| | |
|---|---|
| upstream repo | `https://github.com/isaac-sim/IsaacSim-ros_workspaces` |
| commit | `50de00358f220d790d17050c6368cfe9a9cb9f51` (tag `IsaacSim-5.1.0`) |
| path in repo | `jazzy_ws/src/navigation/carter_navigation/maps/carter_warehouse_navigation.{yaml,png}` |
| fetched | 2026-09-01, `raw.githubusercontent.com` at the commit above (HTTP 200) |
| licence | Apache-2.0 (upstream repo `LICENSE`) |

`image: carter_warehouse_navigation.png` in the yaml is a **bare relative filename**, which
`nav2_map_server` resolves against the directory of the yaml itself — so vendoring the pair
into one directory needs no path edit and none was made.

## Why the *carter* map is the right map for the Go2

The Go2 scene the platform composes (`go2_warehouse`, runner scene registry) references the
**same warehouse USD and the same extras layer** the carter sample references, both with an
identity transform, and only adds the robot. So the occupancy grid transfers unchanged —
map frame == world frame, no offset to reconcile.

That is not an inference from file names; the platform measured it (decision **AR-13**,
2026-09-01, on the live composed stage — `reports/runner-2026-09-01-go2-c1-scene.md` §4):

| property | measured |
|---|---|
| `resolution` | 0.05 m/px |
| `origin` | `[-11.975, -17.975, 0.0]` |
| png size | 480 x 776 px -> 24.0 m x 38.8 m |
| warehouse footprint (probe) | 24.0 m x 38.82 m |
| composed extras layer | 345 prims at **identity** |

## What this repo uses it for

`go2_nav.launch.py` passes it to `nav2_bringup` as the `map:=` argument, which starts
`map_server` + `amcl` (the `use_localization` path). The AMCL prior in
`../params/nav2_params.yaml` (`set_initial_pose` / `initial_pose`) is expressed in **this
map's frame**, and so are the `scenarios/*.yaml` goals — one coordinate system for the
whole fixture.

Free-space clearances quoted in the scenario headers were computed **from this png**
(trinary thresholds from the yaml, chamfer distance transform); e.g. the T0 lane
`x = -6.0, y in [-1, 5]` has 1.35 m of clearance at the start and 2.7-2.8 m along the rest.
