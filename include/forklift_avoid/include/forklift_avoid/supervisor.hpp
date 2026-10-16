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
