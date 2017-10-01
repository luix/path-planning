//===--- SelfDrivingCar--.cpp -----------------------------------*- C++ -*-===//
//
// This source file is part of the Bosch Challenge Path Planner Project
//
// Author: Luis Vivero <luis.a.vivero@gmail.com>
//
//===----------------------------------------------------------------------===//
//  Udacity Open Source Self-Driving Car
//  Copyright (C) 2016, 2017 Udacity
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//===----------------------------------------------------------------------===//

#include "SelfDrivingCar.h"


using namespace std;


Path SelfDrivingCar::Planner(const Path &previous_path,
                             const Vehicle &ego_car,
                             const vector<RaceCar> &racers) {

  auto n_consumed = previous_s_path.size() - previous_path.size();
  previous_s_path.erase(previous_s_path.begin(),
                        previous_s_path.begin() + n_consumed);

  if (in_lane_change) {
    if (previous_path.size() <= kPathLength) { // < kPathLength
      in_lane_change = false;
    }
    return previous_path;
  }

  if (previous_path.size() == 0) {
    return FastStart(previous_path, ego_car, racers);
  }

//  bool is_previous_plan_available = (previous_path.size() >= 2);
//  if (! is_previous_plan_available) {
//    return FastStart(previous_path, ego_car, racers);
//  }

  int ego_car_lane = road.FindLane(ego_car.d);
  double ego_car_speed = ego_car.speed * kMilesPerHourToMetersPerSecond;
  vector<double> frontcar_speeds;
  vector<double> frontcar_dists;
  for (auto lane = 0; lane <= road.kRightMostLane; ++lane) {
    auto frontcar_idx = road.FindFrontCarInLane(ego_car, racers, lane);
    double frontcar_speed = kInfinity;
    double frontcar_dist = kInfinity;
    if (frontcar_idx != -1) {
      const RaceCar & frontcar(racers[frontcar_idx]);
      frontcar_speed = sqrt(frontcar.vx*frontcar.vx + frontcar.vy*frontcar.vy);
      frontcar_dist = frontcar.s - ego_car.s;
    }
    frontcar_speeds.push_back(frontcar_speed);
    frontcar_dists.push_back(frontcar_dist);
  }

  if (frontcar_speeds[ego_car_lane] <= 0.95 * kMaxSpeed
      && frontcar_dists[ego_car_lane] <= 100) {

    int target_a = 0;
    int target_b = 2;
    if (ego_car_lane == 0) {
      target_a = 1;
    } else if (ego_car_lane == 2) {
      target_b = 1;
    }

    double cost_a = 0;
    double cost_b = 0;

    cout << "eval changing from " << ego_car_lane;
    cout << " to " << target_a;
    cout << " or to " << target_b;
    cout << endl;

    bool safe_a = SafeToChangeLane(ego_car, racers, target_a);
    bool safe_b = SafeToChangeLane(ego_car, racers, target_b);

    if (safe_a) cost_a++;
    if (safe_b) cost_b++;

    if (safe_a && frontcar_speeds[target_a] > ego_car_speed) cost_a++;
    if (safe_b && frontcar_speeds[target_b] > ego_car_speed) cost_b++;
    cout << "[compared speeds between ego and target] cost_a: " << cost_a << " , cost_b:" << cost_b << endl;

    (frontcar_speeds[target_a] > frontcar_speeds[target_b]) ? cost_a++ : cost_b++;
    cout << "[compared speeds between target lanes] cost_a: " << cost_a << " , cost_b:" << cost_b << endl;

    if (safe_a && frontcar_speeds[target_a] >= frontcar_speeds[ego_car_lane] * 1.1) cost_a++;
    if (safe_b && frontcar_speeds[target_b] >= frontcar_speeds[ego_car_lane] * 1.1) cost_b++;
    cout << "[compared speeds on target lanes] cost_a: " << cost_a << " , cost_b:" << cost_b << endl;

    if (safe_a && frontcar_dists[target_a] >= kSafeCarDistance + frontcar_dists[ego_car_lane]) cost_a++;
    if (safe_b && frontcar_dists[target_b] >= kSafeCarDistance + frontcar_dists[ego_car_lane]) cost_b++;
    cout << "[compared safe minimal distance on target lanes] cost_a: " << cost_a << " , cost_b:" << cost_b << endl;

    if (safe_a && frontcar_dists[target_a] >= (kSafeCarDistance * 3) + frontcar_dists[ego_car_lane]) cost_a++;
    if (safe_b && frontcar_dists[target_b] >= (kSafeCarDistance * 3) + frontcar_dists[ego_car_lane]) cost_b++;
    cout << "[compared long distance on target lanes] cost_a: " << cost_a << " , cost_b:" << cost_b << endl;

    if (safe_a && frontcar_dists[target_a] + kSafeCarDistance >= frontcar_dists[ego_car_lane]) cost_a++;
    if (safe_b && frontcar_dists[target_b] + kSafeCarDistance >= frontcar_dists[ego_car_lane]) cost_b++;
    cout << "[compared short distance on target lanes] cost_a: " << cost_a << " , cost_b:" << cost_b << endl;

    if (safe_a && cost_a >= 4 && cost_a > cost_b) {
      cout << "change to line:" << target_a << endl;
      return ChangeLane(previous_path, ego_car, racers, target_a); // - ego_car_lane);
    }
    else if (safe_b && cost_b >= 4) {
      cout << "change to line:" << target_b << endl;
      return ChangeLane(previous_path, ego_car, racers, target_b); // - ego_car_lane);
    }
  }
  return KeepLane(previous_path, ego_car, racers);
}

Path SelfDrivingCar::FastStart(const Path &previous_path,
                               const Vehicle &ego_car,
                               const vector<RaceCar> &racers) {
  cout << "do a fast and smooth start..." << endl;
  int ego_car_lane = road.FindLane(ego_car.d);

  const auto & lane_s2x{road.lane_s2x[ego_car_lane]};
  const auto & lane_s2y{road.lane_s2y[ego_car_lane]};

  Spline ego_to_lane_s2x;
  Spline ego_to_lane_s2y;

  WayPoints fit_lane_s;
  WayPoints fit_lane_x;
  WayPoints fit_lane_y;

  fit_lane_s.push_back(ego_car.s);
  fit_lane_x.push_back(ego_car.x);
  fit_lane_y.push_back(ego_car.y);
  for (auto s = ego_car.s + 30; s < (ego_car.s + 90); s++) {  //180
    fit_lane_s.push_back(s);
    fit_lane_x.push_back(lane_s2x(s));
    fit_lane_y.push_back(lane_s2y(s));
  }
  ego_to_lane_s2x.set_points(fit_lane_s, fit_lane_x);
  ego_to_lane_s2y.set_points(fit_lane_s, fit_lane_y);

  WayPoints s_path;

  int frontcar_idx = road.FindFrontCarInLane(ego_car, racers, ego_car_lane);

  bool is_free_to_go = true;
  if (frontcar_idx != -1) {
    const RaceCar & car(racers[frontcar_idx]);
    double response_time = kTimeInterval * kPathLength;
    auto safe_dist = ego_car.speed * kMilesPerHourToMetersPerSecond
                     * response_time + kSafeCarDistance;
    auto dist = car.s - ego_car.s;
    is_free_to_go = (dist >= safe_dist);
  }
  double target_speed = kMaxSpeed;
  if (! is_free_to_go ) {
    const RaceCar & car(racers[frontcar_idx]);
    double carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
    target_speed = carspeed * 0.7;
  }
  cout << "target speed:" << target_speed << endl;
  double inc_s = 0.11;
  //double speed = 2;
  int ii = 0;
  //double s_speed = ;
  for (double speed = 0.5; speed < target_speed; speed += 0.175) { // 9.25
    inc_s += (speed * kTimeInterval);
    cout << " [" << ii++ << "] s:" << (ego_car.s + inc_s) << ", speed:" << speed << endl;
    s_path.push_back(ego_car.s + inc_s);
  }
//  for (int i = 0; i < 36; i++) {
//    inc_s += (speed * kTimeInterval);
//    speed += 0.5;
//    cout << " [" << i << "] s:" << (ego_car.s + inc_s) << ", speed:" << speed << endl;
//    s_path.push_back(ego_car.s + inc_s);
//  }
  for (int i = 1; i < 180; i++) {  //240
    double ss = ego_car.s + inc_s + (0.4 * i);  //0.44
    cout << " [" << i << "] ss:" << ss << endl;
    s_path.push_back(ss);
  }

  WayPoints x_path;
  WayPoints y_path;

  for ( int i = 0 ; i < 120 ; i++ ) { //160
    const auto & s = s_path[i];
    cout << " [" << i << "] s:" << s << endl;
    x_path.push_back(ego_to_lane_s2x(s));
    y_path.push_back(ego_to_lane_s2y(s));
  }

  const auto & s2x(road.lane_s2x[ego_car_lane]);
  const auto & s2y(road.lane_s2y[ego_car_lane]);
  for ( int i = 120 ; i < s_path.size() ; i++ ) { //160
    const auto & s = s_path[i];
    cout << " [" << i << "] s:" << s << endl;
    x_path.push_back(s2x(s));
    y_path.push_back(s2y(s));
  }
  previous_s_path = s_path;
  in_lane_change = true;
  return Path(x_path, y_path);
}

Path SelfDrivingCar::KeepLane(const Path &previous_path,
                              const Vehicle &ego_car,
                              const vector<RaceCar> &racers) {
  cout << "keep lane" << endl;
  WayPoints s_path;

  int ego_car_lane = road.FindLane(ego_car.d);
  int frontcar_idx = road.FindFrontCarInLane(ego_car, racers, ego_car_lane);

  bool is_free_to_go = true;
  if (frontcar_idx != -1) {
    const RaceCar & car(racers[frontcar_idx]);
    double response_time = kTimeInterval * kPathLength;
    auto safe_dist = ego_car.speed * kMilesPerHourToMetersPerSecond
                     * response_time + kSafeCarDistance;
    auto dist = car.s - ego_car.s;
    is_free_to_go = (dist >= safe_dist);
  }
  double target_speed = kMaxSpeed;
  if (! is_free_to_go ) {
    const RaceCar & car(racers[frontcar_idx]);
    double carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
    target_speed = carspeed * 0.9; // 0.7
  }

  bool is_previous_plan_available = (previous_path.size() >= 2);
  double last_speed;
  int newplan_start;
  if (! is_previous_plan_available) {
    s_path.push_back(ego_car.s + 0.25);
    last_speed = ego_car.speed * kMilesPerHourToMetersPerSecond;
    newplan_start = 1;
  } else {
    auto n = previous_s_path.size();
    for (auto i = 0; i < min(kMaxPreviousPathSteps, n); ++i) {
      s_path.push_back(previous_s_path[i]);
    }
    last_speed = (s_path.back() - s_path[s_path.size()-2]) / kTimeInterval;
    newplan_start = s_path.size();
  }
  double speed = last_speed;
  for (auto i = newplan_start; i < kPathLength; ++i) {
    speed = Accelerate(speed, target_speed);
    s_path.push_back(s_path.back() + speed * kTimeInterval);
  }

  WayPoints x_path;
  WayPoints y_path;
  const auto & s2x(road.lane_s2x[ego_car_lane]);
  const auto & s2y(road.lane_s2y[ego_car_lane]);
  for (const auto & s: s_path) {
    double sx = s2x(s); // - 0.404;
    double sy = s2y(s); // - 0.404;
    x_path.push_back(sx);
    y_path.push_back(sy);
  }

  previous_s_path = s_path;

  return Path(x_path, y_path);
}

Path SelfDrivingCar::ChangeLane(const Path &previous_path,
                                const Vehicle &ego_car,
                                const vector<RaceCar> &racers,
                                int target_lane) {

  int ego_car_lane = road.FindLane(ego_car.d);
  //int target_lane = ego_car_lane + lane_change;
  int target_frontcar_idx = road.FindFrontCarInLane(ego_car, racers, target_lane);
  int target_rearcar_idx = road.FindRearCarInLane(ego_car, racers, target_lane);
  int current_frontcar_idx = road.FindFrontCarInLane(ego_car, racers, ego_car_lane);

  bool two_lane_change = fabs(ego_car_lane - target_lane) == 2;

  if (two_lane_change &&
      (!SafeToChangeLane(ego_car, racers, 1))){
    cout << "NOT safe to make a two lanes change" << endl;
    return KeepLane(previous_path, ego_car, racers);
  }

  if (!SafeToChangeLane(ego_car, racers, target_lane)) {
    return KeepLane(previous_path, ego_car, racers);
  } else {
    const auto & current_s2x{road.lane_s2x[ego_car_lane]};
    const auto & current_s2y{road.lane_s2y[ego_car_lane]};
    const auto & target_s2x{road.lane_s2x[target_lane]};
    const auto & target_s2y{road.lane_s2y[target_lane]};

    Spline lanechange_s2x;
    Spline lanechange_s2y;

    WayPoints lanechange_s;
    WayPoints lanechange_x;
    WayPoints lanechange_y;

    double start_s = ego_car.s - 20;
    cout << "start_s:" << start_s << endl;
    double change_s = ego_car.s + 15;
    if (target_frontcar_idx != -1) {
      change_s = min(change_s, racers[target_frontcar_idx].s);
    }
    cout << "change_s:" << change_s << endl;
    for (auto s = start_s; s <= change_s; s++) {
      lanechange_s.push_back(s);
      lanechange_x.push_back(current_s2x(s));
      lanechange_y.push_back(current_s2y(s));
    }
    int change_length = two_lane_change ? 50 * 2 : 50;
    cout << "change_length:" << change_length << endl;
    double target_start_s = change_s + change_length; //100
    cout << "target_start_s:" << target_start_s << endl;
    double target_end_s = change_s + change_length + 50; //100
    cout << "target_end_s:" << target_end_s << endl;
    for (auto s = target_start_s; s <= target_end_s; ++s) { //50
      lanechange_s.push_back(s);
      lanechange_x.push_back(target_s2x(s));
      lanechange_y.push_back(target_s2y(s));
    }
    lanechange_s2x.set_points(lanechange_s, lanechange_x);
    lanechange_s2y.set_points(lanechange_s, lanechange_y);

    //vector<double> s_path;

    /*
    bool is_previous_plan_available = (previous_path.size() >= 2);
    double last_speed = -1;
    int newplan_start  = -1;
    if (! is_previous_plan_available) {
      s_path.push_back(ego_car.s + 0.25);
      last_speed = ego_car.speed * kMilesPerHourToMetersPerSecond;
      newplan_start = 1;
    } else {
      auto n = previous_s_path.size();
      for (auto i = 0; i < min(kMaxPreviousPathSteps, n); ++i) {
        s_path.push_back(previous_s_path[i]);
      }
      last_speed = (s_path.back() - s_path[s_path.size()-2]) / kTimeInterval;
      newplan_start = s_path.size();
    }
    double target_speed = min(last_speed * 1.02, kMaxSpeed - 1.5); //0.5
    double speed = last_speed;
    cout << "target_speed:" << target_speed << endl;
    int lane_change_length = (abs(target_lane - ego_car_lane) == 2) ? 8 : 6;
    cout << "lane_change_length:" << lane_change_length << endl;
    for (auto i = newplan_start; i < kPathLength * lane_change_length; ++i) {
      speed = Accelerate(speed, target_speed);
      s_path.push_back(s_path.back() + speed * kTimeInterval);
    }
    */

    WayPoints s_path;

//    bool is_previous_plan_available = (previous_path.size() >= 2);
//    double last_speed;
//    int newplan_start;
//    if (is_previous_plan_available) {
//      auto n = previous_s_path.size();
//      for (auto i = 0; i < min(kMaxPreviousPathSteps, n); ++i) {
//        double prev_s = previous_s_path[i];
//        cout << " [" << i << "] prev_s:" << prev_s << endl;
//        s_path.push_back(prev_s);
//      }
//      last_speed = (s_path.back() - s_path[s_path.size()-2]) / kTimeInterval;
//      newplan_start = s_path.size();
//    }


//    double current_speed = ego_car.speed * kMilesPerHourToMetersPerSecond;
//    cout << "current_speed:" << current_speed << endl;
//    double target_speed = current_speed * .95; //min(current_speed * 1.02, kMaxSpeed - 2);
//    cout << "target_speed:" << target_speed << endl;
//    double lane_change_steps = change_length / (target_speed * kTimeInterval);
//    cout << "lane_change_steps:" << lane_change_steps << endl;
//    double speed = current_speed;
//    double inc_s = ego_car.s;
////    for (auto i = 0; i < 10; ++i) {
////      inc_s += (speed * kTimeInterval);
////      cout << " [" << i << "] s:" << inc_s << ", speed:" << speed << endl;
////      s_path.push_back(inc_s);
////      //s_path.push_back(s_path.back() + speed * kTimeInterval);
////      speed = Accelerate(speed, target_speed);
////    }
//
//    for (auto i = 0; i < lane_change_steps + kPathLength; ++i) {
//      speed = Accelerate(speed, target_speed);
//      inc_s += (speed * kTimeInterval);
//      cout << " [" << i << "] s:" << inc_s << ", speed:" << speed << endl;
//      s_path.push_back(inc_s);
//    }


    auto n = previous_s_path.size();
    for (auto i = 0; i < min(kMaxPreviousPathSteps, n); ++i) {
      s_path.push_back(previous_s_path[i]);
    }
    double last_speed = (s_path.back() - s_path[s_path.size()-2]) / kTimeInterval;
    int newplan_start = s_path.size();
    // continue to build new path - it is a longer plan
    // than usual because it usually takes longer time
    // to change lanes
    double target_speed = min(last_speed * 1.02, kMaxSpeed);
    double speed = last_speed;
    for (auto i = newplan_start; i < (kPathLength * 5 + change_length); ++i) {
      speed = Accelerate(speed, target_speed);
      s_path.push_back(s_path.back() + speed * kTimeInterval);
    }

    WayPoints x_path;
    WayPoints y_path;
    for (const auto & s: s_path) {
      double sx = lanechange_s2x(s); // - 0.404;
      double sy = lanechange_s2y(s); // - 0.404;
      x_path.push_back(sx);
      y_path.push_back(sy);
    }

    cout << "start changing lanes..." << endl;

    previous_s_path = s_path;

    in_lane_change = true;
    return Path(x_path, y_path);
  }
}

double SelfDrivingCar::Accelerate(double current_speed,
                                  double target_speed) const {
  double diff = target_speed - current_speed;
  double delta = diff * 0.005;
  if (fabs(diff) <= 0.01) delta = 0;
  return current_speed + delta;
}

bool SelfDrivingCar::SafeToChangeLane(const Vehicle &ego_car,
                                      const vector<RaceCar> &racers,
                                      int target_lane) const {
  int ego_lane = road.FindLane(ego_car.d);
  int target_frontcar_idx = road.FindFrontCarInLane(ego_car, racers, target_lane);
  int target_rearcar_idx = road.FindRearCarInLane(ego_car, racers, target_lane);
  int current_frontcar_idx = road.FindFrontCarInLane(ego_car, racers, ego_lane);

  bool is_safe_to_change = (target_lane >= 0)
                           && (target_lane <= road.kRightMostLane);

  is_safe_to_change &= (ego_car.s >= 50) && (ego_car.s <= 6800);
  if (!is_safe_to_change) {
    cout << " its NOT safe to change to lane " << target_lane;
    cout << endl;
    return false;
  }

  const double ego_car_speed = ego_car.speed *kMilesPerHourToMetersPerSecond;

  if (target_frontcar_idx != -1) {
    const RaceCar & car(racers[target_frontcar_idx]);
    auto carspeed = sqrt(car.vx * car.vx + car.vy + car.vy);
    double response_time = 4; // 4 seconds
    double diff = carspeed - ego_car_speed;
    auto safe_distance = (diff >= 0) ? kSafeCarDistance :
                         fabs(diff * response_time) + kSafeCarDistance;
//        fabs(ego_car.speed * kMilesPerHourToMetersPerSecond - carspeed)
//        * response_time + kSafeCarDistance;
    auto dist = car.s - ego_car.s;
    is_safe_to_change &= (dist >= safe_distance);
//    if (dist < safe_distance) {
      cout << " distance to front car in target lane " << dist;
      cout << " safe dist:" << safe_distance;
      cout << endl;
//    }
    if (!is_safe_to_change) {
      cout << " its NOT safe to change to lane " << target_lane;
      cout << endl;
      return false;
    }
  }

  if (target_rearcar_idx != -1) {
    const RaceCar & car(racers[target_rearcar_idx]);
    auto carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
    double response_time = 4; // 4 seconds
    double diff = carspeed - ego_car_speed;
    auto safe_distance = (diff <= 0) ? kSafeCarDistance :
                         fabs(diff * response_time) + kSafeCarDistance;
//    auto safe_distance =
//        fabs(carspeed - ego_car.speed *kMilesPerHourToMetersPerSecond)
//        * response_time + kSafeCarDistance;
    auto dist = ego_car.s - car.s;
    is_safe_to_change &= (dist >= safe_distance);
//    if (dist < safe_distance) {
      cout << " distance to rear car in target lane " << dist;
      cout << " safe dist:" << safe_distance;
      cout << endl;
//    }
    if (!is_safe_to_change) {
      cout << " its NOT safe to change to lane " << target_lane;
      cout << endl;
      return false;
    }
  }

  if (current_frontcar_idx != -1) {
    const RaceCar & car(racers[current_frontcar_idx]);
    auto carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
    double response_time = 3; // 2 seconds
    double diff = carspeed - ego_car_speed;
    auto safe_distance = (diff >= 0) ? kSafeCarDistance / 3 :
                         fabs(diff * response_time) + kSafeCarDistance / 3;
//    auto safe_distance =
//        fabs(ego_car.speed*kMilesPerHourToMetersPerSecond - carspeed)
//        * response_time + kSafeCarDistance;
    auto dist = car.s - ego_car.s;
    is_safe_to_change &= (dist >= safe_distance);
//    if (dist < safe_distance) {
      cout << " distance to front car " << dist;
      cout << " safe dist:" << safe_distance;
      cout << endl;
//    }
    if (!is_safe_to_change) {
      cout << " its NOT safe to change to lane " << target_lane;
      cout << endl;
      return false;
    }
  }

  /*
  for (auto i = 0; i < racers.size(); i++ ) {
    const RaceCar & racer = racers[i];
    auto racer_lane = road.FindLane(racer.d);
    if (racer_lane == target_lane) {
      if (racer.s >= (ego_car.s - kSafeCarDistance) &&
          (racer.s <= (ego_car.s + kSafeCarDistance))) {
        cout << "race car id [" << i << "] is near ego car ";
        cout << " racer.s:" << racer.s << " , s:" << ego_car.s;
        cout << " its NOT safe to change to lane " << target_lane;
        cout << endl;
        //is_safe_to_change = false;
        return false;
      }
      else if (racer.s >= (ego_car.s - (kSafeCarDistance * 3)) &&
                 (racer.s <= (ego_car.s - kSafeCarDistance))) {
        auto racer_speed = sqrt(racer.vx * racer.vx + racer.vy * racer.vy);
        if (racer_speed >= ego_car_speed * 1.15) {
          cout << "race car id [" << i << "] is behind ego car ";
          cout << " racer.s:" << racer.s << " , s:" << ego_car.s;
          cout << " racer.speed:" << racer_speed << " , speed:" << ego_car_speed;
          cout << " its NOT safe to change to lane " << target_lane;
          cout << endl;
          //is_safe_to_change = false;
          return false;
        }
      }
    }
  }
  */
  if (!is_safe_to_change) {
    cout << " its NOT safe to change to lane " << target_lane;
    cout << endl;
    return false;
  } else {
    cout << " its SAFE to change to lane " << target_lane;
    cout << endl;
    return true;
  }

  /*
  if (target_frontcar_idx != -1) {
    const RaceCar & car(racers[target_frontcar_idx]);
    auto carspeed = sqrt(car.vx * car.vx + car.vy + car.vy);
    double response_time = 4; // 4 seconds
    auto safe_distance =
        fabs(ego_car.speed * kMilesPerHourToMetersPerSecond - carspeed)
        * response_time + kSafeCarDistance;
    auto dist = car.s - ego_car.s;
    is_safe_to_change &= (dist >= safe_distance);
    if (dist < safe_distance) {
      cout << " distance to front car in target lane " << dist;
      cout << " safe dist:" << safe_distance;
      cout << endl;
    }
  }
  if (target_rearcar_idx != -1) {
    const RaceCar & car(racers[target_rearcar_idx]);
    auto carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
    double response_time = 4; // 4 seconds
    auto safe_distance =
        fabs(carspeed - ego_car.speed *kMilesPerHourToMetersPerSecond)
        * response_time + kSafeCarDistance;
    auto dist = ego_car.s - car.s;
    is_safe_to_change &= (dist >= safe_distance);
    if (dist < safe_distance) {
      cout << " distance to rear car in target lane " << dist;
      cout << " safe dist:" << safe_distance;
      cout << endl;
    }
  }
  if (current_frontcar_idx != -1) {
    const RaceCar & car(racers[current_frontcar_idx]);
    auto carspeed = sqrt(car.vx * car.vx + car.vy * car.vy);
    double response_time = 3; // 2 seconds
    auto safe_distance =
        fabs(ego_car.speed*kMilesPerHourToMetersPerSecond - carspeed)
        * response_time + kSafeCarDistance;
    auto dist = car.s - ego_car.s;
    is_safe_to_change &= (dist >= safe_distance);
    if (dist < safe_distance) {
      cout << " distance to front car " << dist;
      cout << " safe dist:" << safe_distance;
      cout << endl;
    }
  }
  */

  //return is_safe_to_change;
}
