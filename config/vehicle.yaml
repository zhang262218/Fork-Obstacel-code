// ===== README.md =====
# forklift_avoid

Obstacle-avoidance module for a pallet / cage handling forklift AMR. It turns the written scheme (four lidars plus two fork-tip proximity lasers, three nested zones) into code:

- **Zone calculator**: stop / deceleration / sound-and-light alarm fields for every motion state (forward, reverse, left and right curve, spin in place, docking), speed gear and load (empty, pallet 1200x800, pallet 1200x1000, cage 1200x1000).
- **Supervisor**: classifies sensor points against the active fields, applies a hold time, outputs a speed limit, a safety-stop request, lamp and buzzer pattern, and fail-safe behaviour on sensor faults.
- **ROS 2 wrapper** (optional, `ros2/`).

C++17, no third-party dependencies in the core. Everything is in metres, kilograms, seconds, radians.

## Build and test

```
cmake -S . -B build && cmake --build build
./build/test_main          # unit tests
./build/zone_dump          # every zone extent as CSV (same numbers as the spreadsheet)
```

`docs/zones_reference.csv` is the output of `zone_dump`; its 34 rows match the spreadsheet `forklift_amr_obstacle_zones_v3.xlsx` to rounding.

## Frame and geometry

Origin = load-axle centre = spin pivot. +x towards the nose, +y left. The front steered wheel at angle alpha puts the instantaneous centre on the load-axle line at lateral distance `d = wheelbase / tan(alpha)`. At alpha = 90 deg `d = 0` and the vehicle spins about the load axle (far radius 1.603 m, set by the nose corner).

| Position (from the nose) | mm |
|---|---|
| fork root | 487 |
| steered drive wheel | 648 |
| load axle / pivot | 1525 |
| fork tip, proximity lasers | 1637 |

## Zone formulas (`ZoneCalculator::compute`)

```
S1 = v_r*t_resp + v_r^2/(2a) + Z                       stop
S2 = S1 + (v_r^2 - vc_r^2)/(2*a_service) + Z_B          deceleration
S3 = S2 + v_r*t_warn + Z_A                              alarm
a  = min(F_brake/m, a_cap, load cap) - g*slope (downhill)
```
`v_r` is the speed at the leading point (`k` converts drive speed to nose speed on a curve and to far-point speed when spinning). Zones are the swept union of the footprint (body, forks, load plus tolerance), extended by a lateral margin that grows with speed. Curves and spins are unions of rotated footprint copies about the instantaneous centre.

## Sensors (`sensors.hpp`)

| id | role | notes |
|---|---|---|
| 3D | mast-top 3D lidar, volume to 1.43 m | navigation layer only (`safety_rated=false`), ground returns below 50 mm are dropped |
| FL, FR | lower front corner 2D lidars, plane at 50 mm | safety rated, required: a missing or stale frame gives a safety stop |
| R | fork-root rear-facing 2D lidar, plane at 120 mm | ignored when loaded (the load blocks it) and `Output::degraded` is set |
| T | optional fork-tip lidar | `defaultLidars(..., true)` |
| P1, P2 | fork-tip proximity lasers, 0.30 m | active in reverse and docking. Reverse: hit means stop. Docking: hit means pallet, slow to 0.15 m/s |

## Supervisor outputs (`Output`)

`level` (Clear, Alarm, Decel, Stop), `safety_stop` (safety circuit only; the 3D lidar never sets it), `nav_stop`, `target_speed` and `max_decel` for the speed controller, `lamp` and `buzzer_hz` (yellow 1 Hz, orange 2.5 Hz, red continuous), `degraded`, `docking_target`, `sensor_fault`, `hits`, `field_set`.

## What is verified and what is not

- The core library builds without warnings and `test_main` passes, including the reference numbers from the simulation (forward high empty 1.43 / 3.97 / 6.97 m; left curve high loaded pallet 0.99 / 2.20 / 4.09 m; spin high loaded pallet 0.54 / 1.19 / 2.53 m).
- The ROS 2 node was written but **not compiled or run**.
- The braking force (1800 N), response time split, steering angles (20 / 40 deg), cage size and cage deceleration cap (0.7 m/s2), load tolerance and proximity laser position are assumptions. Replace them with measured values.
- The stop zone must be implemented in a certified safety scanner / safety PLC for ISO 3691-4. This module computes and checks fields; it is not a safety-certified implementation.

## Open points to confirm

- Spin pivot at the load axle gives a far radius of 1.603 m against the spec-sheet turning radius of 1.18 m.
- Spec value "fork surface to front 467 mm" differs by 20 mm from 1637 - 1150 = 487 mm; the code uses 487.
- The two proximity beams are 500 mm apart, so a centred 400 mm lying test piece passes between them. Reverse at more than about 0.4 to 0.5 m/s cannot stop within 0.30 m.


// ===== CMakeLists.txt =====
cmake_minimum_required(VERSION 3.10)
project(forklift_avoid LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

add_library(forklift_avoid src/geometry.cpp src/zone_calculator.cpp src/sensors.cpp src/supervisor.cpp)
target_include_directories(forklift_avoid PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

add_executable(zone_dump apps/zone_dump.cpp)
target_link_libraries(zone_dump forklift_avoid)

enable_testing()
add_executable(test_main tests/test_main.cpp)
target_link_libraries(test_main forklift_avoid)
add_test(NAME forklift_avoid COMMAND test_main)

# Optional ROS 2 wrapper: cmake -DBUILD_ROS2_NODE=ON (needs rclcpp, sensor_msgs, std_msgs, visualization_msgs)
option(BUILD_ROS2_NODE "Build the ROS 2 node" OFF)
if(BUILD_ROS2_NODE)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  add_executable(forklift_avoid_node ros2/forklift_avoid_node.cpp)
  target_link_libraries(forklift_avoid_node forklift_avoid)
  ament_target_dependencies(forklift_avoid_node rclcpp sensor_msgs std_msgs visualization_msgs)
  install(TARGETS forklift_avoid_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY ros2/launch config DESTINATION share/${PROJECT_NAME})
endif()


// ===== apps/zone_dump.cpp =====
// Prints the zone-extent table (the same numbers as the spreadsheet and the 3D page) as CSV.
#include <cstdio>
#include "forklift_avoid/zone_calculator.hpp"
using namespace fa;

int main() {
  ZoneCalculator calc;
  const Motion modes[] = {Motion::Forward, Motion::Reverse, Motion::CurveLeft, Motion::SpinLeft, Motion::Dock};
  const LoadType loads[] = {LoadType::Empty, LoadType::Pallet1200x800, LoadType::Pallet1200x1000, LoadType::Cage1200x1000};
  std::printf("motion,gear,load,v_nominal,alpha_deg,d_m,k,a_emerg,S_stop,S_decel,S_alarm,lat_stop,lat_decel,lat_alarm,width_stop,width_decel,width_alarm,theta_stop_deg,theta_decel_deg,theta_alarm_deg\n");
  for (Motion m : modes) for (Gear g : {Gear::High, Gear::Low}) for (LoadType l : loads) {
    if (m == Motion::Dock && l != LoadType::Empty) continue;
    ZoneRequest r; r.motion = m; r.gear = g; r.load = l; r.build_polygons = false;
    const ZoneSet z = calc.compute(r);
    std::printf("%s,%s,%s,%.3f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n", name(m), g == Gear::High ? "high" : "low",
                loadSpec(l).name, z.v_nominal, z.alpha_deg, z.d, z.k, z.decel.a, z.S[0], z.S[1], z.S[2], z.lat[0], z.lat[1], z.lat[2], z.width[0], z.width[1], z.width[2],
                z.theta[0] * 180 / kPi, z.theta[1] * 180 / kPi, z.theta[2] * 180 / kPi);
  }
}


// ===== config/vehicle.yaml =====
# Reference values (same as the defaults compiled into params.hpp). Units: m, kg, s, rad.
forklift_avoid_node:
  ros__parameters:
    vehicle:
      mass_net: 680.0
      rated_load: 1500.0
      wheelbase: 0.877
      overhang: 0.648          # nose to steered drive wheel
      length: 1.637            # includes forks
      width: 0.985
      fork_len: 1.15
      fork_out: 0.68
      mast_retracted: 1.3265
      v_loaded: 1.389          # 5 km/h
      v_empty: 1.667           # 6 km/h
      slope_loaded: 0.03
      slope_empty: 0.05
    tuning:                    # assumptions: replace with measured values
      t_resp: 0.35
      t_warn: 1.5
      f_brake: 1800.0
      a_cap: 2.0
      load_tol: 0.05
      alpha_hi_deg: 20.0       # front wheel angle on curves, high / low gear
      alpha_lo_deg: 40.0       # alpha = 90 deg: spin about the load-axle centre
      prox_len: 0.30           # fork-tip proximity lasers
      prox_y: 0.25
      prox_z: 0.057
      stop_release_s: 1.0
    fork_tip_lidar: false
    topics:
      scan_fl: /lidar_fl/points      # PointCloud2, any frame, transformed to base_link by the node's TF listener (see TODO)
      scan_fr: /lidar_fr/points
      scan_rear: /lidar_rear/points
      lidar_3d: /lidar_top/points
      prox_left: /prox_left/range    # sensor_msgs/Range
      prox_right: /prox_right/range


// ===== docs/zones_reference.csv =====
motion,gear,load,v_nominal,alpha_deg,d_m,k,a_emerg,S_stop,S_decel,S_alarm,lat_stop,lat_decel,lat_alarm,width_stop,width_decel,width_alarm,theta_stop_deg,theta_decel_deg,theta_alarm_deg
forward,high,empty,1.667,90,0.000,1.000,2.000,1.428,3.968,6.968,0.287,0.683,1.283,1.558,2.352,3.552,0.0,0.0,0.0
forward,high,pallet_1200x800,1.389,90,0.000,1.000,0.826,1.804,4.331,6.915,0.259,0.628,1.186,1.503,2.241,3.357,0.0,0.0,0.0
forward,high,pallet_1200x1000,1.389,90,0.000,1.000,0.826,1.804,4.331,6.915,0.259,0.628,1.186,1.618,2.356,3.472,0.0,0.0,0.0
forward,high,cage_1200x1000,1.389,90,0.000,1.000,0.700,2.014,4.941,7.524,0.259,0.628,1.186,1.618,2.356,3.472,0.0,0.0,0.0
forward,low,empty,0.500,90,0.000,1.000,2.000,0.388,0.821,2.071,0.170,0.450,0.875,1.325,1.885,2.735,0.0,0.0,0.0
forward,low,pallet_1200x800,0.500,90,0.000,1.000,0.826,0.476,0.970,2.220,0.170,0.450,0.875,1.325,1.885,2.735,0.0,0.0,0.0
forward,low,pallet_1200x1000,0.500,90,0.000,1.000,0.826,0.476,0.970,2.220,0.170,0.450,0.875,1.440,2.000,2.850,0.0,0.0,0.0
forward,low,cage_1200x1000,0.500,90,0.000,1.000,0.700,0.504,1.032,2.282,0.170,0.450,0.875,1.440,2.000,2.850,0.0,0.0,0.0
reverse,high,empty,0.833,90,0.000,1.000,2.000,0.665,1.469,3.219,0.203,0.517,0.992,1.392,2.018,2.968,0.0,0.0,0.0
reverse,high,pallet_1200x800,0.694,90,0.000,1.000,0.826,0.735,1.510,3.052,0.189,0.489,0.943,1.364,1.963,2.871,0.0,0.0,0.0
reverse,high,pallet_1200x1000,0.694,90,0.000,1.000,0.826,0.735,1.510,3.052,0.189,0.489,0.943,1.479,2.078,2.986,0.0,0.0,0.0
reverse,high,cage_1200x1000,0.694,90,0.000,1.000,0.700,0.788,1.648,3.190,0.189,0.489,0.943,1.479,2.078,2.986,0.0,0.0,0.0
reverse,low,empty,0.300,90,0.000,1.000,2.000,0.328,0.627,1.577,0.150,0.410,0.805,1.285,1.805,2.595,0.0,0.0,0.0
reverse,low,pallet_1200x800,0.300,90,0.000,1.000,0.826,0.359,0.659,1.609,0.150,0.410,0.805,1.285,1.805,2.595,0.0,0.0,0.0
reverse,low,pallet_1200x1000,0.300,90,0.000,1.000,0.826,0.359,0.659,1.609,0.150,0.410,0.805,1.400,1.920,2.710,0.0,0.0,0.0
reverse,low,cage_1200x1000,0.300,90,0.000,1.000,0.700,0.369,0.669,1.619,0.150,0.410,0.805,1.400,1.920,2.710,0.0,0.0,0.0
curve_left,high,empty,1.000,20,2.410,1.112,2.000,0.848,2.086,4.254,0.220,0.550,1.050,1.425,2.085,3.085,17.0,41.9,85.5
curve_left,high,pallet_1200x800,0.833,20,2.410,1.112,0.826,0.994,2.200,4.090,0.203,0.517,0.992,1.392,2.018,2.968,20.0,44.2,82.2
curve_left,high,pallet_1200x1000,0.833,20,2.410,1.112,0.826,0.994,2.200,4.090,0.203,0.517,0.992,1.507,2.133,3.083,20.0,44.2,82.2
curve_left,high,cage_1200x1000,0.833,20,2.410,1.112,0.700,1.088,2.456,4.346,0.203,0.517,0.992,1.507,2.133,3.083,21.9,49.3,87.3
curve_left,low,empty,0.400,40,1.045,1.355,2.000,0.413,0.820,2.133,0.160,0.430,0.840,1.305,1.845,2.665,12.8,25.4,66.1
curve_left,low,pallet_1200x800,0.400,40,1.045,1.355,0.826,0.518,0.973,2.286,0.160,0.430,0.840,1.305,1.845,2.665,16.0,30.2,70.9
curve_left,low,pallet_1200x1000,0.400,40,1.045,1.355,0.826,0.518,0.973,2.286,0.160,0.430,0.840,1.420,1.960,2.780,16.0,30.2,70.9
curve_left,low,cage_1200x1000,0.400,40,1.045,1.355,0.700,0.550,1.033,2.346,0.160,0.430,0.840,1.420,1.960,2.780,17.0,32.0,72.7
spin_left,high,empty,0.500,90,0.000,1.603,2.000,0.591,1.405,3.107,0.170,0.450,0.875,1.325,1.885,2.735,21.1,50.2,111.1
spin_left,high,pallet_1200x800,0.350,90,0.000,1.603,0.826,0.537,1.187,2.528,0.155,0.420,0.822,1.295,1.825,2.630,19.2,42.4,90.4
spin_left,high,pallet_1200x1000,0.350,90,0.000,1.603,0.826,0.537,1.187,2.528,0.155,0.420,0.822,1.410,1.940,2.745,19.2,42.4,90.4
spin_left,high,cage_1200x1000,0.350,90,0.000,1.603,0.700,0.571,1.284,2.625,0.155,0.420,0.822,1.410,1.940,2.745,20.4,45.9,93.9
spin_left,low,empty,0.200,90,0.000,1.603,2.000,0.288,0.652,1.633,0.140,0.390,0.770,1.265,1.765,2.525,10.3,23.3,58.4
spin_left,low,pallet_1200x800,0.200,90,0.000,1.603,0.826,0.324,0.718,1.698,0.140,0.390,0.770,1.265,1.765,2.525,11.6,25.7,60.7
spin_left,low,pallet_1200x1000,0.200,90,0.000,1.603,0.826,0.324,0.718,1.698,0.140,0.390,0.770,1.380,1.880,2.640,11.6,25.7,60.7
spin_left,low,cage_1200x1000,0.200,90,0.000,1.603,0.700,0.336,0.746,1.726,0.140,0.390,0.770,1.380,1.880,2.640,12.0,26.7,61.7
dock,high,empty,0.300,90,0.000,1.000,2.000,0.278,0.578,1.527,0.050,0.150,0.300,1.085,1.285,1.585,0.0,0.0,0.0
dock,low,empty,0.150,90,0.000,1.000,2.000,0.208,0.508,1.233,0.050,0.150,0.300,1.085,1.285,1.585,0.0,0.0,0.0


// ===== include/forklift_avoid/geometry.hpp =====
#pragma once
#include "forklift_avoid/types.hpp"

namespace fa {
Vec2 rotate(Vec2 p, Vec2 c, double a);
Polygon rectPoly(const Rect& r);
bool pointInPolygon(Vec2 p, const Polygon& poly);
bool pointInAny(Vec2 p, const std::vector<Polygon>& list);
double signedArea(const Polygon& p);
Polygon makeCcw(Polygon p);
bool pointInRect(Vec2 p, const Rect& r);
}  // namespace fa


// ===== include/forklift_avoid/params.hpp =====
#pragma once
#include "forklift_avoid/types.hpp"

namespace fa {

// Vehicle data supplied by the customer (SI units).
struct VehicleSpec {
  double mass_net = 680, rated_load = 1500;
  double wheelbase = 0.877, overhang = 0.648;      // overhang: nose to steered drive wheel
  double length = 1.637, width = 0.985;            // length includes forks
  double fork_len = 1.15, fork_h = 0.055, fork_out = 0.68;
  double mast_retracted = 1.3265, fork_lowered = 0.085;
  double v_loaded = 5.0 / 3.6, v_empty = 6.0 / 3.6;
  double slope_loaded = 0.03, slope_empty = 0.05;
  double min_turn_radius_wa = 1.18;                // spec-sheet value, only used for consistency checks
  double aisle = 2.0;                              // right-angle stacking aisle
  double s_face() const { return length - fork_len; }   // s: distance from the nose along the body
  double s_drive() const { return overhang; }
  double s_axle() const { return overhang + wheelbase; }
  double s_tip() const { return length; }
};

// Tunable assumptions. Replace with measured values before release.
struct Tuning {
  double t_resp = 0.35, t_warn = 1.5;
  double v_creep = 0.3, w_creep = 0.1;
  double z_buf_decel = 0.3, z_buf_alarm = 0.5;
  double z_general = 0.10, z_reflect = 0.05, z_rev_extra = 0.05;   // ISO 13855 general supplement 100 mm
  double load_tol = 0.05;
  double f_brake = 1800, a_cap = 2.0;
  double as_k = 0.5, as_cap = 0.6;
  double accel = 0.5, accel_loaded = 0.4;
  double fork_h_carry = 0.20;
  double h3d_margin = 0.10, spin_margin = 0.10;
  double alpha_hi_deg = 20, alpha_lo_deg = 40;     // front wheel angle used on curves; alpha=90 -> spin about the load axle
  double range_2d = 8.0;
  double plane_nose = 0.05, plane_rear = 0.12, plane_tip = 0.06;
  double prox_len = 0.30, prox_y = 0.25, prox_z = 0.057;   // two fork-tip point lasers facing rearward
  double dock_speed_hi = 0.3, dock_speed_lo = 0.15;        // ISO 3691-4: muted detection <= 0.3 m/s
  double stop_release_s = 1.0;                              // hold time after an obstacle leaves a field
};

struct LoadSpec {
  LoadType type; const char* name; bool loaded;
  double L, W, deck, cargo_h;
  double a_cap;        // emergency decel cap for this load, <=0 means none
};

inline const LoadSpec& loadSpec(LoadType t) {
  static const LoadSpec table[] = {
    {LoadType::Empty, "empty", false, 0, 0, 0, 0, 0},
    {LoadType::Pallet1200x800, "pallet_1200x800", true, 1.2, 0.8, 0.144, 1.0, 0},
    {LoadType::Pallet1200x1000, "pallet_1200x1000", true, 1.2, 1.0, 0.144, 1.0, 0},
    {LoadType::Cage1200x1000, "cage_1200x1000", true, 1.2, 1.0, 0.15, 0.95, 0.7},   // cage cap is an assumption
  };
  return table[static_cast<int>(t)];
}

}  // namespace fa


// ===== include/forklift_avoid/sensors.hpp =====
#pragma once
#include <string>
#include "forklift_avoid/params.hpp"

namespace fa {

enum class SensorType { Lidar2D, Lidar3D };

struct LidarConfig {
  std::string id;               // "3D", "FL", "FR", "R", "T"
  SensorType type;
  Vec2 pos{};                   // mount position in the vehicle frame
  double plane_z = 0;           // scan plane height (2D)
  double bisector = 0;          // rad, 0 = +x
  double fov_deg = 0, range = 8;
  bool blocked_when_loaded = false;   // rear lidar sits behind the load
  bool required = true;               // a missing or unhealthy required sensor stops the vehicle
  bool safety_rated = true;           // false: reaches Decel/Alarm and a navigation stop, never the safety stop
};

struct ProxConfig { std::string id; Vec2 pos; double z; double len; };

// Default layout: front 3D lidar on the mast top, FL/FR 2D lidars at the lower front corners,
// rear-facing 2D lidar at the fork root, two fork-tip proximity lasers (0.30 m), optional fork-tip lidar.
std::vector<LidarConfig> defaultLidars(const VehicleSpec& v, const Tuning& t, bool fork_tip_lidar = false);
std::vector<ProxConfig> defaultProx(const VehicleSpec& v, const Tuning& t);

}  // namespace fa


// ===== include/forklift_avoid/supervisor.hpp =====
#pragma once
#include <map>
#include "forklift_avoid/sensors.hpp"
#include "forklift_avoid/zone_calculator.hpp"

namespace fa {

struct SensorFrame { std::string id; std::vector<Point3> points; bool healthy = true; };   // points already in the vehicle frame
struct ProxFrame { std::string id; bool valid = false; double distance = 0; };               // distance to the first hit, m

struct VehicleState {
  Motion motion = Motion::Forward;
  Gear gear = Gear::High;
  LoadType load = LoadType::Empty;
  double speed = 0;               // current drive-axis speed (m/s, or rad/s for spin)
  bool downhill = false;
  double alpha_deg = 0;           // measured front wheel angle (curves); 0 = default
};

enum class Lamp { Green, Yellow, Orange, Red };

struct Output {
  Level level = Level::Clear;          // highest level seen by any sensor
  Level safety_level = Level::Clear;   // from safety-rated sensors only
  bool safety_stop = false;            // to the safety circuit (PLd): cut drive and brake
  bool nav_stop = false;               // navigation-layer stop (3D lidar / soft stop)
  double target_speed = 0;             // limit for the speed controller (same unit as VehicleState::speed)
  double max_decel = 0;                // decel / accel limit to apply while reaching target_speed, in drive-axis units/s^2
  Lamp lamp = Lamp::Green;
  double buzzer_hz = 0;                // 0 = off, <0 = continuous
  bool degraded = false;               // a coverage hole exists (e.g. rear lidar hidden behind the load)
  bool docking_target = false;         // proximity laser sees the pallet while docking
  bool sensor_fault = false;
  std::vector<std::string> hits;       // ids of sensors that currently see something in a field
  std::string field_set;               // e.g. "reverse/high/cage_1200x1000/v=0.35"
};

class Supervisor {
 public:
  Supervisor(VehicleSpec v = {}, Tuning t = {}, bool fork_tip_lidar = false);
  // One cycle. dt in seconds. Frames missing for a required sensor count as a fault.
  Output update(const VehicleState& s, const std::vector<SensorFrame>& frames, const std::vector<ProxFrame>& prox, double dt);
  const ZoneSet& zones() const { return zones_; }
  const ZoneCalculator& calculator() const { return calc_; }
  const std::vector<LidarConfig>& lidars() const { return lidars_; }
  void setLidars(std::vector<LidarConfig> l) { lidars_ = std::move(l); }

 private:
  ZoneCalculator calc_;
  std::vector<LidarConfig> lidars_;
  std::vector<ProxConfig> prox_;
  ZoneSet zones_;
  std::string key_;
  Level held_ = Level::Clear; double hold_t_ = 0;
  void refresh(const VehicleState& s);
};

}  // namespace fa


// ===== include/forklift_avoid/types.hpp =====
#pragma once
// Basic types. Vehicle frame: origin = load-axle centre (spin pivot), +x towards the nose, +y left, metres.
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace fa {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 { double x = 0, y = 0; };
using Polygon = std::vector<Vec2>;
struct Rect { double x0, y0, x1, y1; };          // axis-aligned, x0<x1, y0<y1
struct Point3 { double x, y, z; };

enum class Motion { Forward, Reverse, CurveLeft, CurveRight, SpinLeft, SpinRight, Dock };
enum class Gear { High, Low };
enum class LoadType { Empty, Pallet1200x800, Pallet1200x1000, Cage1200x1000 };
enum class Level { Clear = 0, Alarm = 1, Decel = 2, Stop = 3 };   // alarm zone / deceleration zone / stop zone
enum class Kind { Trans, Curve, Spin };

inline Kind kindOf(Motion m) {
  switch (m) {
    case Motion::CurveLeft: case Motion::CurveRight: return Kind::Curve;
    case Motion::SpinLeft: case Motion::SpinRight: return Kind::Spin;
    default: return Kind::Trans;
  }
}
inline int dirOf(Motion m) { return (m == Motion::Reverse || m == Motion::Dock) ? -1 : 1; }   // -1: forks lead
inline int sgnOf(Motion m) { return (m == Motion::CurveLeft || m == Motion::SpinLeft) ? 1 : -1; }
inline const char* name(Motion m) {
  static const char* n[] = {"forward", "reverse", "curve_left", "curve_right", "spin_left", "spin_right", "dock"};
  return n[static_cast<int>(m)];
}

}  // namespace fa


// ===== include/forklift_avoid/zone_calculator.hpp =====
#pragma once
#include "forklift_avoid/params.hpp"

namespace fa {

struct Footprint {
  Rect body, forks, load{0, 0, 0, 0};
  bool loaded = false;
  std::vector<Rect> plan;                 // body + forks (+ load inflated by tolerance)
  double nose_x = 0, rear_x = 0, r_far = 0, hw_max = 0;
};

struct Decel { double mass, a, a_service; };   // emergency / service deceleration, m/s^2

struct ZoneRequest {
  Motion motion = Motion::Forward;
  Gear gear = Gear::High;
  LoadType load = LoadType::Empty;
  bool downhill = false;
  double alpha_deg = 0;                  // front wheel angle on curves; 0 = use the tuning default for the gear
  double v_actual = -1;                  // current speed (m/s or rad/s); <0 = nominal. Shrinks only the stop zone.
  bool build_polygons = true;
};

// Three nested zones, index 0 = stop, 1 = decel, 2 = alarm. S = length from the leading edge.
struct ZoneSet {
  Motion motion; Gear gear; LoadType load;
  Footprint fp; Decel decel;
  std::array<double, 3> S{}, lat{}, width{}, theta{};   // theta: swept angle, rad (curve / spin only)
  double v_nominal = 0, v_actual = 0, k = 1;            // k: nose speed / drive speed (curve), far radius (spin)
  double alpha_deg = 90, d = 0, arm = 0, r_dw = 1;       // steering geometry, d = wheelbase / tan(alpha)
  Vec2 icr{0, 0};
  double h3d = 0;                                       // 3D lidar volume height
  std::array<std::vector<Polygon>, 3> polys;            // 2D fields (union of polygons)
  std::array<Polygon, 3> vol;                           // 3D lidar footprint of the volume (forward / curve only)
  Polygon vol_end;                                      // far edge of the 3D frustum
};

class ZoneCalculator {
 public:
  ZoneCalculator(VehicleSpec v = {}, Tuning t = {}) : v_(v), t_(t) {}
  const VehicleSpec& spec() const { return v_; }
  const Tuning& tuning() const { return t_; }

  Footprint footprint(LoadType load) const;
  Decel decel(LoadType load, bool downhill) const;
  double nominalSpeed(Motion m, Gear g, bool loaded) const;
  double creepSpeed(Motion m) const { return kindOf(m) == Kind::Spin ? t_.w_creep : t_.v_creep; }
  ZoneSet compute(const ZoneRequest& r) const;

  // Level 0..3 of a point in the vehicle frame; 3D version also checks height.
  static Level levelOf(Vec2 p, const ZoneSet& z);
  static Level level3d(Vec2 p, const ZoneSet& z);

 private:
  VehicleSpec v_; Tuning t_;
  double X(double s) const { return v_.s_axle() - s; }
};

}  // namespace fa


// ===== ros2/forklift_avoid_node.cpp =====
// ROS 2 wrapper around the dependency-free core. NOT compiled or run in the environment this project was written in:
// treat it as a starting point and build it on a machine with ROS 2 (Humble or newer).
//
// Inputs : PointCloud2 for each lidar (expected already in the base frame, origin = load-axle centre, x forward),
//          sensor_msgs/Range for the two fork-tip lasers, /vehicle_state (see below).
// Outputs: /avoid/level (Int8), /avoid/safety_stop (Bool), /avoid/speed_limit (Float32), /avoid/zones (MarkerArray).
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "forklift_avoid/supervisor.hpp"

using namespace std::chrono_literals;

class AvoidNode : public rclcpp::Node {
 public:
  AvoidNode() : Node("forklift_avoid_node") {
    fa::VehicleSpec v; fa::Tuning t;
    v.mass_net = declare_parameter("vehicle.mass_net", v.mass_net);
    v.rated_load = declare_parameter("vehicle.rated_load", v.rated_load);
    v.wheelbase = declare_parameter("vehicle.wheelbase", v.wheelbase);
    v.overhang = declare_parameter("vehicle.overhang", v.overhang);
    v.length = declare_parameter("vehicle.length", v.length);
    v.width = declare_parameter("vehicle.width", v.width);
    t.f_brake = declare_parameter("tuning.f_brake", t.f_brake);
    t.t_resp = declare_parameter("tuning.t_resp", t.t_resp);
    t.alpha_hi_deg = declare_parameter("tuning.alpha_hi_deg", t.alpha_hi_deg);
    t.alpha_lo_deg = declare_parameter("tuning.alpha_lo_deg", t.alpha_lo_deg);
    t.prox_len = declare_parameter("tuning.prox_len", t.prox_len);
    const bool tip = declare_parameter("fork_tip_lidar", false);
    sv_ = std::make_unique<fa::Supervisor>(v, t, tip);

    auto cloud = [this](const std::string& id, const std::string& topic) {
      subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS(),
        [this, id](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { onCloud(id, *m); }));
    };
    cloud("FL", declare_parameter("topics.scan_fl", "/lidar_fl/points"));
    cloud("FR", declare_parameter("topics.scan_fr", "/lidar_fr/points"));
    cloud("R", declare_parameter("topics.scan_rear", "/lidar_rear/points"));
    cloud("3D", declare_parameter("topics.lidar_3d", "/lidar_top/points"));
    auto prox = [this](const std::string& id, const std::string& topic) {
      prox_subs_.push_back(create_subscription<sensor_msgs::msg::Range>(topic, rclcpp::SensorDataQoS(),
        [this, id](sensor_msgs::msg::Range::ConstSharedPtr m) {
          fa::ProxFrame f{id, std::isfinite(m->range) && m->range >= m->min_range && m->range <= m->max_range, m->range};
          prox_[id] = f; }));
    };
    prox("P1", declare_parameter("topics.prox_left", "/prox_left/range"));
    prox("P2", declare_parameter("topics.prox_right", "/prox_right/range"));

    // State topics use simple std_msgs so no custom message package is needed. Replace with your own interface.
    mode_sub_ = create_subscription<std_msgs::msg::String>("/avoid/mode", 10, [this](std_msgs::msg::String::ConstSharedPtr m) { parseMode(m->data); });
    speed_sub_ = create_subscription<std_msgs::msg::Float32>("/avoid/drive_speed", 10, [this](std_msgs::msg::Float32::ConstSharedPtr m) { state_.speed = m->data; });

    level_pub_ = create_publisher<std_msgs::msg::Int8>("/avoid/level", 10);
    stop_pub_ = create_publisher<std_msgs::msg::Bool>("/avoid/safety_stop", 10);
    limit_pub_ = create_publisher<std_msgs::msg::Float32>("/avoid/speed_limit", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/avoid/zones", 1);
    timer_ = create_wall_timer(50ms, [this] { tick(); });   // 20 Hz
  }

 private:
  // /avoid/mode string: "<motion> <gear> <load>", e.g. "reverse low cage_1200x1000"
  void parseMode(const std::string& s) {
    static const char* mo[] = {"forward", "reverse", "curve_left", "curve_right", "spin_left", "spin_right", "dock"};
    static const char* lo[] = {"empty", "pallet_1200x800", "pallet_1200x1000", "cage_1200x1000"};
    std::istringstream is(s); std::string m, g, l; is >> m >> g >> l;
    for (int i = 0; i < 7; ++i) if (m == mo[i]) state_.motion = static_cast<fa::Motion>(i);
    state_.gear = g == "low" ? fa::Gear::Low : fa::Gear::High;
    for (int i = 0; i < 4; ++i) if (l == lo[i]) state_.load = static_cast<fa::LoadType>(i);
  }
  void onCloud(const std::string& id, const sensor_msgs::msg::PointCloud2& m) {
    fa::SensorFrame f; f.id = id; f.healthy = true;
    sensor_msgs::PointCloud2ConstIterator<float> ix(m, "x"), iy(m, "y"), iz(m, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) if (std::isfinite(*ix)) f.points.push_back({*ix, *iy, *iz});
    frames_[id] = {std::move(f), now()};
  }
  void tick() {
    std::vector<fa::SensorFrame> fr;
    for (auto& [id, p] : frames_) {
      fa::SensorFrame f = p.first;
      f.healthy = (now() - p.second).seconds() < 0.3;     // stale data = unhealthy
      fr.push_back(std::move(f));
    }
    std::vector<fa::ProxFrame> px;
    for (auto& [id, f] : prox_) px.push_back(f);
    const auto o = sv_->update(state_, fr, px, 0.05);
    std_msgs::msg::Int8 lv; lv.data = static_cast<int8_t>(o.level); level_pub_->publish(lv);
    std_msgs::msg::Bool st; st.data = o.safety_stop; stop_pub_->publish(st);
    std_msgs::msg::Float32 lim; lim.data = static_cast<float>(o.target_speed); limit_pub_->publish(lim);
    if (o.degraded) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "coverage degraded (rear lidar hidden behind the load)");
    publishZones();
  }
  void publishZones() {
    visualization_msgs::msg::MarkerArray arr;
    const auto& z = sv_->zones();
    const float col[3][3] = {{0.85f, 0.2f, 0.17f}, {0.94f, 0.54f, 0.14f}, {0.94f, 0.76f, 0.18f}};
    for (int i = 2; i >= 0; --i) {
      visualization_msgs::msg::Marker mk; mk.header.frame_id = "base_link"; mk.ns = "zones"; mk.id = i;
      mk.type = visualization_msgs::msg::Marker::LINE_LIST; mk.scale.x = 0.03; mk.color.a = 1.0;
      mk.color.r = col[i][0]; mk.color.g = col[i][1]; mk.color.b = col[i][2];
      for (const auto& poly : z.polys[i]) for (size_t a = 0; a < poly.size(); ++a) {
        const auto& p = poly[a]; const auto& q = poly[(a + 1) % poly.size()];
        geometry_msgs::msg::Point P, Q; P.x = p.x; P.y = p.y; Q.x = q.x; Q.y = q.y; mk.points.push_back(P); mk.points.push_back(Q);
      }
      arr.markers.push_back(mk);
    }
    marker_pub_->publish(arr);
  }
  std::unique_ptr<fa::Supervisor> sv_;
  fa::VehicleState state_;
  std::map<std::string, std::pair<fa::SensorFrame, rclcpp::Time>> frames_;
  std::map<std::string, fa::ProxFrame> prox_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> prox_subs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr level_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr limit_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidNode>());
  rclcpp::shutdown();
}


// ===== ros2/launch/avoid.launch.py =====
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('forklift_avoid'), 'config', 'vehicle.yaml')
    return LaunchDescription([Node(package='forklift_avoid', executable='forklift_avoid_node', parameters=[cfg], output='screen')])


// ===== src/geometry.cpp =====
#include "forklift_avoid/geometry.hpp"
#include <algorithm>

namespace fa {

Vec2 rotate(Vec2 p, Vec2 c, double a) {
  const double ca = std::cos(a), sa = std::sin(a), dx = p.x - c.x, dy = p.y - c.y;
  return {c.x + dx * ca - dy * sa, c.y + dx * sa + dy * ca};
}
Polygon rectPoly(const Rect& r) { return {{r.x0, r.y0}, {r.x1, r.y0}, {r.x1, r.y1}, {r.x0, r.y1}}; }

bool pointInPolygon(Vec2 p, const Polygon& poly) {   // even-odd ray casting
  bool in = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const Vec2 &a = poly[i], &b = poly[j];
    if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
  }
  return in;
}
bool pointInAny(Vec2 p, const std::vector<Polygon>& list) {
  return std::any_of(list.begin(), list.end(), [&](const Polygon& q) { return pointInPolygon(p, q); });
}
double signedArea(const Polygon& p) {
  double s = 0;
  for (size_t i = 0; i < p.size(); ++i) { const Vec2& a = p[i]; const Vec2& b = p[(i + 1) % p.size()]; s += a.x * b.y - b.x * a.y; }
  return s / 2;
}
Polygon makeCcw(Polygon p) { if (signedArea(p) < 0) std::reverse(p.begin(), p.end()); return p; }
bool pointInRect(Vec2 p, const Rect& r) { return p.x >= r.x0 && p.x <= r.x1 && p.y >= r.y0 && p.y <= r.y1; }

}  // namespace fa


// ===== src/sensors.cpp =====
#include "forklift_avoid/sensors.hpp"

namespace fa {

std::vector<LidarConfig> defaultLidars(const VehicleSpec& v, const Tuning& t, bool tip) {
  const double hw = v.width / 2, nose = v.s_axle();
  std::vector<LidarConfig> L;
  LidarConfig l3; l3.id = "3D"; l3.type = SensorType::Lidar3D; l3.safety_rated = false; l3.range = 8;
  l3.pos = {nose - v.s_face() + 0.035, 0};
  L.push_back(l3);
  LidarConfig fl; fl.id = "FL"; fl.type = SensorType::Lidar2D; fl.pos = {nose + 0.012, hw + 0.012}; fl.plane_z = t.plane_nose; fl.bisector = kPi / 4; fl.fov_deg = 270; fl.range = t.range_2d;
  LidarConfig fr = fl; fr.id = "FR"; fr.pos.y = -fr.pos.y; fr.bisector = -kPi / 4;
  LidarConfig r; r.id = "R"; r.type = SensorType::Lidar2D; r.pos = {v.s_axle() - v.s_face() - 0.01, 0}; r.plane_z = t.plane_rear; r.bisector = kPi; r.fov_deg = 200; r.range = t.range_2d; r.blocked_when_loaded = true; r.required = false;
  L.push_back(fl); L.push_back(fr); L.push_back(r);
  if (tip) { LidarConfig k; k.id = "T"; k.type = SensorType::Lidar2D; k.pos = {v.s_axle() - v.s_tip() - 0.02, 0}; k.plane_z = t.plane_tip; k.bisector = kPi; k.fov_deg = 180; k.range = t.range_2d; k.required = false; L.push_back(k); }
  return L;
}

std::vector<ProxConfig> defaultProx(const VehicleSpec& v, const Tuning& t) {
  const double x = v.s_axle() - v.s_tip();
  return {{"P1", {x, t.prox_y}, t.prox_z, t.prox_len}, {"P2", {x, -t.prox_y}, t.prox_z, t.prox_len}};
}

}  // namespace fa


// ===== src/supervisor.cpp =====
#include "forklift_avoid/supervisor.hpp"
#include "forklift_avoid/geometry.hpp"
#include <algorithm>
#include <cstdio>

namespace fa {

Supervisor::Supervisor(VehicleSpec v, Tuning t, bool tip)
    : calc_(v, t), lidars_(defaultLidars(v, t, tip)), prox_(defaultProx(v, t)) {}

// Field sets are re-computed only when the discretised state changes (speed in 0.05 steps).
void Supervisor::refresh(const VehicleState& s) {
  char buf[96];
  const double vq = std::round(s.speed * 20) / 20;
  std::snprintf(buf, sizeof buf, "%s/%s/%s/v=%.2f", name(s.motion), s.gear == Gear::High ? "high" : "low", loadSpec(s.load).name, vq);
  const std::string key = std::string(buf) + (s.downhill ? "/down" : "") + "/a=" + std::to_string(static_cast<int>(s.alpha_deg));
  if (key == key_) return;
  key_ = key;
  ZoneRequest r; r.motion = s.motion; r.gear = s.gear; r.load = s.load; r.downhill = s.downhill; r.alpha_deg = s.alpha_deg; r.v_actual = vq;
  zones_ = calc_.compute(r);
}

Output Supervisor::update(const VehicleState& s, const std::vector<SensorFrame>& frames, const std::vector<ProxFrame>& prox, double dt) {
  refresh(s);
  const ZoneSet& z = zones_;
  const Tuning& t = calc_.tuning();
  const bool loaded = loadSpec(s.load).loaded;
  Output o;
  o.field_set = key_;

  Level nav = Level::Clear, safe = Level::Clear;
  auto raise = [](Level& a, Level b) { if (static_cast<int>(b) > static_cast<int>(a)) a = b; };

  for (const LidarConfig& L : lidars_) {
    const auto it = std::find_if(frames.begin(), frames.end(), [&](const SensorFrame& f) { return f.id == L.id; });
    const bool hidden = L.blocked_when_loaded && loaded;
    if (hidden) { if (s.motion == Motion::Reverse || kindOf(s.motion) == Kind::Spin || s.motion == Motion::Dock) o.degraded = true; continue; }
    if (it == frames.end() || !it->healthy) {
      if (L.required) { o.sensor_fault = true; }
      continue;
    }
    Level lv = Level::Clear;
    for (const Point3& p : it->points) {
      bool self = false;                                   // self filter: ignore returns on the vehicle and its load
      for (const Rect& r : z.fp.plan) if (pointInRect({p.x, p.y}, {r.x0 - 0.03, r.y0 - 0.03, r.x1 + 0.03, r.y1 + 0.03})) { self = true; break; }
      if (self) continue;
      if (L.type == SensorType::Lidar2D) raise(lv, ZoneCalculator::levelOf({p.x, p.y}, z));
      else if (p.z > 0.05 && p.z < z.h3d) raise(lv, ZoneCalculator::level3d({p.x, p.y}, z));   // ground returns removed
    }
    if (lv != Level::Clear) o.hits.push_back(L.id);
    raise(nav, lv);
    if (L.safety_rated) raise(safe, lv);
  }

  // Fork-tip proximity lasers: reverse and docking only.
  if (s.motion == Motion::Reverse || s.motion == Motion::Dock) {
    for (const ProxConfig& pc : prox_) {
      const auto it = std::find_if(prox.begin(), prox.end(), [&](const ProxFrame& f) { return f.id == pc.id; });
      if (it == prox.end() || !it->valid || it->distance > pc.len) continue;
      o.hits.push_back(pc.id);
      if (s.motion == Motion::Dock) o.docking_target = true;   // the pallet itself: slow down, do not stop
      else { raise(nav, Level::Stop); raise(safe, Level::Stop); }
    }
  }

  if (o.sensor_fault) { raise(nav, Level::Stop); raise(safe, Level::Stop); }

  // Hold: a level drops only after stop_release_s without a higher one.
  if (static_cast<int>(nav) >= static_cast<int>(held_)) { held_ = nav; hold_t_ = t.stop_release_s; }
  else { hold_t_ -= dt; if (hold_t_ <= 0) { held_ = nav; hold_t_ = t.stop_release_s; } }
  o.level = held_;
  o.safety_level = safe;
  o.safety_stop = (safe == Level::Stop) || (o.sensor_fault);
  o.nav_stop = o.level == Level::Stop;

  // Speed command in drive-axis units. Conversion to nose / far-point speed uses k.
  const double nom = z.v_nominal, vc = calc_.creepSpeed(s.motion), k = z.k;
  double target = nom, rate = (loaded ? t.accel_loaded : t.accel) / k;
  if (o.level == Level::Stop) { target = 0; rate = z.decel.a / k; }
  else if (o.level == Level::Decel) { target = std::min(nom, vc); rate = z.decel.a_service / k; }
  if (s.motion == Motion::Dock && o.docking_target) target = std::min(target, t.dock_speed_lo);
  o.target_speed = target; o.max_decel = rate;

  switch (o.level) {
    case Level::Clear: o.lamp = Lamp::Green; o.buzzer_hz = 0; break;
    case Level::Alarm: o.lamp = Lamp::Yellow; o.buzzer_hz = 1.0; break;
    case Level::Decel: o.lamp = Lamp::Orange; o.buzzer_hz = 2.5; break;
    case Level::Stop: o.lamp = Lamp::Red; o.buzzer_hz = -1; break;
  }
  return o;
}

}  // namespace fa


// ===== src/zone_calculator.cpp =====
#include "forklift_avoid/zone_calculator.hpp"
#include "forklift_avoid/geometry.hpp"
#include <algorithm>

namespace fa {

Footprint ZoneCalculator::footprint(LoadType lt) const {
  const LoadSpec& ls = loadSpec(lt);
  Footprint f;
  const double hw = v_.width / 2, fo = v_.fork_out / 2;
  f.body = {X(v_.s_face()), -hw, X(0), hw};
  f.forks = {X(v_.s_tip()), -fo, X(v_.s_face()), fo};
  f.plan = {f.body, f.forks};
  f.loaded = ls.loaded;
  if (ls.loaded) {
    f.load = {X(v_.s_face() + ls.L), -ls.W / 2, X(v_.s_face()), ls.W / 2};
    f.plan.push_back({f.load.x0 - t_.load_tol, f.load.y0 - t_.load_tol, f.load.x1, f.load.y1 + t_.load_tol});   // load tolerance
  }
  f.nose_x = X(0);
  f.rear_x = 1e9;
  for (const Rect& r : f.plan) {
    f.rear_x = std::min(f.rear_x, r.x0);
    f.hw_max = std::max({f.hw_max, -r.y0, r.y1});
    for (double x : {r.x0, r.x1}) for (double y : {r.y0, r.y1}) f.r_far = std::max(f.r_far, std::hypot(x, y));
  }
  return f;
}

Decel ZoneCalculator::decel(LoadType lt, bool down) const {
  const LoadSpec& ls = loadSpec(lt);
  const double m = v_.mass_net + (ls.loaded ? v_.rated_load : 0);
  double a = std::min(t_.f_brake / m, t_.a_cap);
  if (ls.loaded && ls.a_cap > 0) a = std::min(a, ls.a_cap);
  if (down) a = std::max(0.15, a - 9.81 * (ls.loaded ? v_.slope_loaded : v_.slope_empty));
  return {m, a, std::min(t_.as_k * a, t_.as_cap)};
}

double ZoneCalculator::nominalSpeed(Motion m, Gear g, bool loaded) const {
  const double base = loaded ? v_.v_loaded : v_.v_empty;
  const bool hi = g == Gear::High;
  switch (m) {
    case Motion::Forward: return hi ? base : 0.5;
    case Motion::Reverse: return hi ? 0.5 * base : 0.3;
    case Motion::CurveLeft: case Motion::CurveRight: return hi ? 0.6 * base : 0.4;
    case Motion::SpinLeft: case Motion::SpinRight: return hi ? (loaded ? 0.35 : 0.5) : 0.2;   // rad/s
    case Motion::Dock: return hi ? t_.dock_speed_hi : t_.dock_speed_lo;
  }
  return 0;
}

ZoneSet ZoneCalculator::compute(const ZoneRequest& q) const {
  ZoneSet z;
  z.motion = q.motion; z.gear = q.gear; z.load = q.load;
  const Kind kind = kindOf(q.motion);
  const int dir = dirOf(q.motion), sgn = sgnOf(q.motion);
  const bool dock = q.motion == Motion::Dock, loaded = loadSpec(q.load).loaded;
  z.fp = footprint(q.load);
  z.decel = decel(q.load, q.downhill);
  const Footprint& g = z.fp;

  if (kind == Kind::Curve) {   // front wheel at alpha: ICR on the load-axle line at lateral distance d = L / tan(alpha)
    z.alpha_deg = q.alpha_deg > 0 ? q.alpha_deg : (q.gear == Gear::High ? t_.alpha_hi_deg : t_.alpha_lo_deg);
    z.d = v_.wheelbase / std::tan(z.alpha_deg * kPi / 180);
    z.arm = std::hypot(v_.s_axle(), z.d);
    z.r_dw = std::hypot(z.d, v_.wheelbase);
    z.icr = {X(v_.s_axle()), sgn * z.d};
  }                            // alpha = 90 deg (spin): d = 0, pivot = load-axle centre = origin
  z.k = kind == Kind::Spin ? g.r_far : (kind == Kind::Curve ? z.arm / z.r_dw : 1.0);
  z.v_nominal = nominalSpeed(q.motion, q.gear, loaded);
  z.v_actual = q.v_actual < 0 ? z.v_nominal : q.v_actual;

  const double vcm = creepSpeed(q.motion), vNr = z.v_nominal * z.k;
  const double vAr = std::min(z.v_nominal, std::max(z.v_actual, vcm)) * z.k, vcr = vcm * z.k;
  const double Z = t_.z_general + t_.z_reflect + ((kind == Kind::Trans && dir < 0 && !dock) ? t_.z_rev_extra : 0);
  auto s1 = [&](double v) { return v * t_.t_resp + v * v / (2 * z.decel.a) + Z; };
  const double s1n = s1(vNr);
  const double s2 = s1n + std::max(0.0, vNr * vNr - vcr * vcr) / (2 * z.decel.a_service) + t_.z_buf_decel;
  const double s3 = s2 + vNr * t_.t_warn + t_.z_buf_alarm;
  z.S = {s1(vAr), s2, s3};
  const double vAm = vAr / z.k, vNm = vNr / z.k;
  z.lat = dock ? std::array<double, 3>{0.05, 0.15, 0.30}
               : std::array<double, 3>{0.12 + 0.1 * vAm, 0.35 + 0.2 * vNm, 0.70 + 0.35 * vNm};
  for (int i = 0; i < 3; ++i) {
    z.width[i] = 2 * (g.hw_max + z.lat[i]);
    if (kind == Kind::Curve) z.theta[i] = z.S[i] / z.arm;
    if (kind == Kind::Spin) z.theta[i] = z.S[i] / g.r_far;
  }
  z.h3d = v_.mast_retracted + t_.h3d_margin;
  if (!q.build_polygons) return z;

  if (kind == Kind::Trans) {
    for (int i = 0; i < 3; ++i) for (const Rect& r : g.plan) {
      Rect b{0, r.y0 - z.lat[i], 0, r.y1 + z.lat[i]};
      if (dir > 0) { b.x0 = r.x0 - z.lat[i]; b.x1 = r.x1 + z.S[i]; } else { b.x0 = r.x0 - z.S[i]; b.x1 = r.x1 + z.lat[i]; }
      z.polys[i].push_back(rectPoly(b));
    }
    if (dir > 0) {
      for (int i = 0; i < 3; ++i) z.vol[i] = rectPoly({g.nose_x, -(g.body.y1 + z.lat[i]), g.nose_x + z.S[i], g.body.y1 + z.lat[i]});
      z.vol_end = {{g.nose_x + z.S[2], g.body.y1 + z.lat[2]}, {g.nose_x + z.S[2], -(g.body.y1 + z.lat[2])}};
    }
  } else if (kind == Kind::Curve) {
    double maxR = 0;
    for (const Rect& r : g.plan) for (double x : {r.x0, r.x1}) for (double y : {r.y0, r.y1}) maxR = std::max(maxR, std::hypot(x - z.icr.x, y - z.icr.y));
    for (int i = 0; i < 3; ++i) {
      const double th = z.S[i] / z.arm;
      const int n = std::min(160, std::max(2, static_cast<int>(std::ceil(th * (maxR + z.lat[i]) / 0.08))));
      std::vector<Polygon> base;
      for (const Rect& r : g.plan) base.push_back(rectPoly({r.x0 - z.lat[i], r.y0 - z.lat[i], r.x1, r.y1 + z.lat[i]}));
      for (int j = 0; j <= n; ++j) {
        const double a = sgn * th * j / n;
        for (const Polygon& p : base) { Polygon c; for (const Vec2& v : p) c.push_back(rotate(v, z.icr, a)); z.polys[i].push_back(c); }
      }
      const double hv = g.body.y1 + z.lat[i];
      Polygon L, R;
      for (int j = 0; j <= n; ++j) { const double a = sgn * th * j / n; L.push_back(rotate({g.nose_x, hv}, z.icr, a)); R.push_back(rotate({g.nose_x, -hv}, z.icr, a)); }
      if (i == 2) z.vol_end = {L.back(), R.front()};
      std::reverse(R.begin(), R.end());
      L.insert(L.end(), R.begin(), R.end());
      z.vol[i] = makeCcw(L);
    }
  } else {
    for (int i = 0; i < 3; ++i) {
      const double th = z.S[i] / g.r_far, m = t_.spin_margin;
      const int n = std::min(160, std::max(2, static_cast<int>(std::ceil(th * g.r_far / 0.08))));
      std::vector<Polygon> base;
      for (const Rect& r : g.plan) base.push_back(rectPoly({r.x0 - m, r.y0 - m, r.x1 + m, r.y1 + m}));
      for (int j = 0; j <= n; ++j) {
        const double a = sgn * th * j / n;
        for (const Polygon& p : base) { Polygon c; for (const Vec2& v : p) c.push_back(rotate(v, {0, 0}, a)); z.polys[i].push_back(c); }
      }
    }
  }
  return z;
}

Level ZoneCalculator::levelOf(Vec2 p, const ZoneSet& z) {
  if (pointInAny(p, z.polys[0])) return Level::Stop;
  if (pointInAny(p, z.polys[1])) return Level::Decel;
  if (pointInAny(p, z.polys[2])) return Level::Alarm;
  return Level::Clear;
}
Level ZoneCalculator::level3d(Vec2 p, const ZoneSet& z) {
  if (z.vol[0].empty()) return Level::Clear;
  if (pointInPolygon(p, z.vol[0])) return Level::Stop;
  if (pointInPolygon(p, z.vol[1])) return Level::Decel;
  if (pointInPolygon(p, z.vol[2])) return Level::Alarm;
  return Level::Clear;
}

}  // namespace fa


// ===== tests/test_main.cpp =====
// Dependency-free tests. Reference numbers come from the verified simulation / spreadsheet.
#include <cstdio>
#include <cstdlib>
#include "forklift_avoid/supervisor.hpp"
using namespace fa;

static int fails = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)
#define NEAR(a, b, tol) do { double _a = (a), _b = (b); if (std::fabs(_a - _b) > (tol)) { std::printf("FAIL %s:%d  %s=%.4f expected %.4f\n", __FILE__, __LINE__, #a, _a, _b); ++fails; } } while (0)

static ZoneSet Z(Motion m, Gear g, LoadType l, bool poly = false) {
  ZoneCalculator c; ZoneRequest r; r.motion = m; r.gear = g; r.load = l; r.build_polygons = poly; return c.compute(r);
}

static void testReferenceNumbers() {
  auto a = Z(Motion::Forward, Gear::High, LoadType::Empty);
  NEAR(a.S[0], 1.43, .006); NEAR(a.S[1], 3.97, .006); NEAR(a.S[2], 6.97, .006);
  auto b = Z(Motion::CurveLeft, Gear::High, LoadType::Pallet1200x800);
  NEAR(b.S[0], 0.99, .006); NEAR(b.S[1], 2.20, .006); NEAR(b.S[2], 4.09, .006);
  NEAR(b.d, 2.41, .006); NEAR(b.k, 1.112, .002);
  auto c = Z(Motion::SpinLeft, Gear::High, LoadType::Pallet1200x800);
  NEAR(c.S[0], 0.54, .006); NEAR(c.S[1], 1.19, .006); NEAR(c.S[2], 2.53, .006);
  NEAR(c.fp.r_far, 1.603, .001);                        // pivot at the load axle: far point is the nose corner
}
static void testSteeringGeometry() {
  ZoneCalculator calc; ZoneRequest r; r.motion = Motion::CurveLeft; r.build_polygons = false;
  r.alpha_deg = 89.999; NEAR(calc.compute(r).d, 0.0, 1e-4);   // alpha -> 90 deg: d -> 0, ICR at the load-axle centre
  r.alpha_deg = 40; NEAR(calc.compute(r).d, 1.045, .002);
}
static void testLoadInflation() {
  const double e = Z(Motion::Reverse, Gear::High, LoadType::Empty).S[0];
  const double p = Z(Motion::Reverse, Gear::High, LoadType::Pallet1200x800).S[0];
  const double c = Z(Motion::Reverse, Gear::High, LoadType::Cage1200x1000).S[0];
  CHECK(p > e); CHECK(c > p);                                    // heavier + weaker brake -> longer stop zone
  NEAR(Z(Motion::Forward, Gear::High, LoadType::Cage1200x1000).fp.hw_max, 0.55, .001);   // 1.0 m cage + tolerance
}
static void testPolygons() {
  auto z = Z(Motion::Forward, Gear::High, LoadType::Empty, true);
  CHECK(ZoneCalculator::levelOf({z.fp.nose_x + 1.0, 0}, z) == Level::Stop);
  CHECK(ZoneCalculator::levelOf({z.fp.nose_x + 3.0, 0}, z) == Level::Decel);
  CHECK(ZoneCalculator::levelOf({z.fp.nose_x + 5.0, 0}, z) == Level::Alarm);
  CHECK(ZoneCalculator::levelOf({z.fp.nose_x + 8.0, 0}, z) == Level::Clear);
  CHECK(ZoneCalculator::levelOf({-3.0, 0}, z) == Level::Clear);   // behind the vehicle is not monitored when driving forward
  auto s = Z(Motion::SpinLeft, Gear::High, LoadType::Empty, true);
  CHECK(ZoneCalculator::levelOf({0.0, 0.2}, s) == Level::Stop);
}

static std::vector<SensorFrame> frameWith(const std::string& id, Point3 p) {
  std::vector<SensorFrame> f{{"FL", {}, true}, {"FR", {}, true}, {"3D", {}, true}};
  for (auto& x : f) if (x.id == id) x.points.push_back(p);
  return f;
}
static void testSupervisor() {
  Supervisor sv;
  VehicleState s; s.motion = Motion::Forward; s.speed = 1.67;
  const double nose = sv.calculator().spec().s_axle();
  auto o = sv.update(s, frameWith("FL", {nose + 0.8, 0.0, 0.05}), {}, 0.05);
  CHECK(o.level == Level::Stop); CHECK(o.safety_stop); CHECK(o.lamp == Lamp::Red); CHECK(o.target_speed == 0);
  Supervisor sv2; s.speed = 1.67;
  o = sv2.update(s, frameWith("FL", {nose + 3.0, 0, 0.05}), {}, 0.05);
  CHECK(o.level == Level::Decel); CHECK(!o.safety_stop); NEAR(o.target_speed, 0.3, 1e-9);
  o = sv2.update(s, frameWith("FL", {nose + 8.5, 0, 0.05}), {}, 0.05);   // outside every field
  CHECK(o.level == Level::Decel);                                          // still held
  for (int i = 0; i < 30; ++i) o = sv2.update(s, frameWith("FL", {nose + 8.5, 0, 0.05}), {}, 0.05);
  CHECK(o.level == Level::Clear);                                          // released after the hold time
  // 3D lidar only influences the navigation layer
  Supervisor sv3;
  o = sv3.update(s, frameWith("3D", {nose + 0.8, 0, 1.2}), {}, 0.05);
  CHECK(o.level == Level::Stop); CHECK(o.nav_stop); CHECK(!o.safety_stop);
  // ground returns are ignored by the 3D lidar
  Supervisor sv4;
  o = sv4.update(s, frameWith("3D", {nose + 0.8, 0, 0.02}), {}, 0.05);
  CHECK(o.level == Level::Clear);
  // missing required sensor -> fail-safe stop
  Supervisor sv5;
  o = sv5.update(s, {{"FL", {}, true}}, {}, 0.05);
  CHECK(o.sensor_fault); CHECK(o.safety_stop);
}
static void testReverseAndProx() {
  Supervisor sv;
  VehicleState s; s.motion = Motion::Reverse; s.speed = 0.3; s.gear = Gear::Low; s.load = LoadType::Cage1200x1000;
  std::vector<SensorFrame> f{{"FL", {}, true}, {"FR", {}, true}, {"3D", {}, true}};
  auto o = sv.update(s, f, {{"P1", true, 0.20}}, 0.05);
  CHECK(o.safety_stop); CHECK(o.degraded);            // rear lidar hidden behind the cage
  Supervisor sv2; s.load = LoadType::Empty;
  o = sv2.update(s, f, {{"P2", true, 0.45}}, 0.05);    // beyond the 0.30 m range
  CHECK(!o.safety_stop);
  // docking: prox hit means "pallet", slow to 0.15 m/s instead of stopping
  Supervisor sv3; VehicleState d; d.motion = Motion::Dock; d.gear = Gear::High; d.speed = 0.3;
  o = sv3.update(d, f, {{"P1", true, 0.25}}, 0.05);
  CHECK(o.docking_target); CHECK(!o.safety_stop); NEAR(o.target_speed, 0.15, 1e-9);
}

int main() {
  testReferenceNumbers(); testSteeringGeometry(); testLoadInflation(); testPolygons(); testSupervisor(); testReverseAndProx();
  std::printf(fails ? "%d FAILED\n" : "all tests passed\n", fails);
  return fails ? 1 : 0;
}
