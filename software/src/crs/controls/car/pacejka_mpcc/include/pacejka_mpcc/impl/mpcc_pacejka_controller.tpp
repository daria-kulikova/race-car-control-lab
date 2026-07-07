#pragma once

#include <chrono>
#include <iomanip>

/**
 * Implementations of the template PacejkaMpccController class.
 */

namespace crs_controls::pacejka_mpcc
{
template <typename SolverType>
PacejkaMpccController<SolverType>::PacejkaMpccController(
    mpcc_pacejka_config config, std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
    std::shared_ptr<StaticTrackTrajectory> track)
  : MpcController(model, std::static_pointer_cast<crs_controls::Trajectory>(track))
{
  setConfig(config);
}

template <typename SolverType>
std::optional<std::vector<std::vector<double>>> PacejkaMpccController<SolverType>::getPlannedTrajectory()
{
  std::vector<std::vector<double>> traj;
  for (int i = 0; i < solver_.N; i++)
  {
    const auto global_coords = convertToGlobalCoordinates(last_solution_.x[i]);
    std::vector<double> row = {
        global_coords.pos_x,
        global_coords.pos_y,
        global_coords.vel_x,
        global_coords.vel_y,
        global_coords.yaw,
        last_reference_on_track_[i].x,
        last_reference_on_track_[i].y,
        last_reference_on_track_[i].yaw,
    };
    if (inducing_gp_.isLoaded() && !mlp_.isLoaded() && config_.inducing_gp_uncertainty_scaling)
    {
      const double vx    = last_solution_.x[i][static_cast<int>(pacejka_vars::VX)];
      const double vy    = last_solution_.x[i][static_cast<int>(pacejka_vars::VY)];
      const double omega = last_solution_.x[i][static_cast<int>(pacejka_vars::DYAW)];
      const double delta = last_solution_.x[i][static_cast<int>(pacejka_vars::STEER)];
      const double T     = last_solution_.x[i][static_cast<int>(pacejka_vars::TORQUE)];
      auto res = inducing_gp_.predict(vx, vy, omega, delta, T,
                                      config_.inducing_gp_uncertainty_scaling);
      row.push_back((res.scale_vx + res.scale_vy + res.scale_omega) / 3.0);
    }
    traj.push_back(std::move(row));
  }

  return traj;
}

template <typename SolverType>
std::optional<std::vector<std::vector<double>>> PacejkaMpccController<SolverType>::getUncorrectedTrajectory()
{
  std::vector<std::vector<double>> traj;
  auto state = last_virtual_state_;
  const double Ts = config_.sample_period;
  traj.push_back({ state.pos_x, state.pos_y, state.vel_x, state.vel_y, state.yaw,
                   last_reference_on_track_[0].x, last_reference_on_track_[0].y,
                   last_reference_on_track_[0].yaw });
  for (int i = 0; i < solver_.N - 1; i++)
  {
    crs_models::pacejka_model::pacejka_car_input input{
        last_solution_.x[i][static_cast<int>(pacejka_vars::TORQUE)],
        last_solution_.x[i][static_cast<int>(pacejka_vars::STEER)]
    };
    state = model_->applyModel(state, input, Ts);
    traj.push_back({ state.pos_x, state.pos_y, state.vel_x, state.vel_y, state.yaw,
                     last_reference_on_track_[i + 1].x, last_reference_on_track_[i + 1].y,
                     last_reference_on_track_[i + 1].yaw });
  }
  return traj;
}

template <typename SolverType>
typename PacejkaMpccController<SolverType>::MpcParameters
PacejkaMpccController<SolverType>::generateCurrentSolverParameters()
{
  auto track = getStaticTrack();
  const double max_arc_length = track->getMaxArcLength();

  MpcParameters parameters;

  for (int stage = 0; stage < solver_.N; stage++)
  {
    double distance_on_track = last_solution_.x[stage][static_cast<int>(pacejka_vars::THETA)] - laps_ * max_arc_length;
    int reference_track_index = distance_on_track * track->getDensity();

    // Update tracking point based on predicted distance on track
    const auto reference_track_point = track->operator[](reference_track_index);
    const auto reference_track_rate = track->getRate(reference_track_index);
    crs_controls::pacejka_mpcc::solvers::TrajectoryTrackPoint track_point = {
      reference_track_point.x(),
      reference_track_point.y(),
      reference_track_rate.x(),
      reference_track_rate.y(),
      distance_on_track + laps_ * max_arc_length,
      track->getTrackAngle(reference_track_index) + laps_ * 2 * M_PI,
    };

    // Update reference visualization
    last_reference_on_track_[stage] = { track_point.x, track_point.y, track_point.phi };

    parameters[stage] = { model_->getParams(),
                          { config_.Q1, config_.Q2, config_.R1, config_.R2, config_.R3, config_.q },
                          track_point };
  }
  return parameters;
}

template <typename SolverType>
void PacejkaMpccController<SolverType>::initialize(crs_models::pacejka_model::pacejka_car_state state)
{
  auto track = getStaticTrack();
  // Initial inputs.
  last_input_.steer = 0;
  // Initial torque input. Lets use 50% throttle
  last_input_.torque = 0.5;
  // Set initial arc length
  theta_ = track->getArcLength(track->getClosestTrackPointIdx(Eigen::Vector2d(state.pos_x, state.pos_y)));

  state.vel_x = std::max(0.01, state.vel_x);

  StateArray x_0 = convertToLocalCoordinates(state, last_input_, theta_);

  // Setup the initial guess (last_solution) of the solver for solver initialization.
  // NOTE(@naefjo): The initial solution of the solver is set using the track reference.
  // The track points are chosen by getting the distance that the car would drive if
  // the forward velocity was ramped up linearly from the initial condition to 1.5m/s along
  // the horizon. This distance (which is the arc length if we assume the car just drives along the
  // center-line) is then transformed into the corresponding track center-line points which make
  // up the initial guess.
  const double ts = solver_.getSamplePeriod();
  const double initial_vel = x_0[3];
  double driven_arclength = theta_;
  auto x_init = x_0;

  for (int stage = 0; stage < solver_.N + 1; stage++)
  {
    const double curr_vel = (1.5 - initial_vel) * stage / (float)(solver_.N + 1) + initial_vel;
    driven_arclength += ts * curr_vel;
    const int reference_track_index = driven_arclength * track->getDensity();
    x_init = generateInitialization(reference_track_index, curr_vel);

    // Set initial state
    last_solution_.x[stage] = x_init;
    // Set initial input (nothing to do, its zero)
  }

  if (!config_.mlp_model_path.empty())
  {
    const auto& p = model_->getParams();
    mlp_.setGeometry(p.lf, p.lr);
    mlp_.load(config_.mlp_model_path);
  }
  else if (!config_.inducing_gp_model_path.empty())
  {
    inducing_gp_.load(config_.inducing_gp_model_path);
    std::cout << "[InducingGP] uncertainty_scaling="
              << (config_.inducing_gp_uncertainty_scaling ? "ON" : "OFF") << std::endl;
  }
  else if (!config_.gp_model_path.empty())
    gp_.load(config_.gp_model_path);

  if (!config_.log_path.empty())
  {
    const bool log_exists = std::ifstream(config_.log_path).good();
    log_stream_.open(config_.log_path, std::ios::app);
    if (!log_stream_.is_open())
    {
      std::cout << "[MPCC] ERROR: could not open log file: " << config_.log_path << std::endl;
    }
    else
    {
      if (!log_exists)
      {
        log_stream_ << "t,vx,vy,omega,delta,T,eps_vx,eps_vy,eps_omega,eC,eL,pos_x,pos_y,ref_x,ref_y,lap,theta\n";
        log_stream_.flush();
      }
      std::cout << "[MPCC] logging → " << config_.log_path
                << (log_exists ? " (appending)" : " (new)") << std::endl;
    }
  }

  // Run solver
  // TODO(@naefjo): reduce cognitive complexity of this code
  for (int run = 0; run < config_.warmstart_iterations; run++)
  {
    MpcParameters parameters = generateCurrentSolverParameters();

    auto solution = solver_.solve_problem(x_0, last_solution_, parameters);
    if (solution.exit_code == crs_controls::control_commons::MpcExitCode::SUCCEEDED)
    {
      last_solution_ = solution.solution.value();
    }
    else
    {
      uint64_t now_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
              .count();
      std::cout << "Solver [ " << std::fixed << uint64_t((now_ms) / 100.0) / 10.0
                << " ]: MPCC exitflag: " << static_cast<int>(solution.exit_code) << " in warmstart iteration " << run
                << std::endl;
    }
  }
}

template <typename SolverType>
crs_models::pacejka_model::pacejka_car_input
PacejkaMpccController<SolverType>::getControlInput(crs_models::pacejka_model::pacejka_car_state state,
                                                   double timestamp [[maybe_unused]])
{
  const double max_arc_length = getStaticTrack()->getMaxArcLength();

  if (!is_initialized_)
  {
    initialize(state);
    is_initialized_ = true;
  }

  // Compute residual = actual_state - model_prediction; store for writing after solver
  const bool log_limit_reached = config_.log_max_points > 0 && log_point_count_ >= config_.log_max_points;
  log_pending_ = false;
  if (!config_.log_path.empty() && !log_limit_reached)
  {
    double t_now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (log_has_prev_)
    {
      double dt = t_now - log_t_prev_;
      if (dt > 0.005 && dt < 0.2)
      {
        auto predicted = model_->applyModel(log_state_prev_, last_input_, dt);
        log_pend_t_         = t_now;
        log_pend_vx_        = log_state_prev_.vel_x;
        log_pend_vy_        = log_state_prev_.vel_y;
        log_pend_omega_     = log_state_prev_.yaw_rate;
        log_pend_delta_     = last_input_.steer;
        log_pend_T_         = last_input_.torque;
        log_pend_eps_vx_    = state.vel_x    - predicted.vel_x;
        log_pend_eps_vy_    = state.vel_y    - predicted.vel_y;
        log_pend_eps_omega_ = state.yaw_rate - predicted.yaw_rate;
        log_pending_ = true;
      }
    }
    log_state_prev_ = state;
    log_t_prev_     = t_now;
    log_has_prev_   = true;
  }

  // initialize state to virtually advanced vehicle states and inputs
  auto virtual_state = model_->applyModel(state, last_input_, config_.lag_compensation_time);
  last_virtual_state_ = virtual_state;

  // Residual correction for lag compensation and zero-order solver disturbance.
  // MLP takes priority over GP when both model paths are configured.
  // Both predict velocity residuals [m/s]; divide by Ts to get acceleration [m/s²] for acados.
  // residual_scale ∈ [0,1] controls aggressiveness of the correction.
  const double Ts    = config_.sample_period;
  const double alpha = config_.residual_scale;

  auto apply_residual = [&](double eps_vx, double eps_vy, double eps_omega, const char* tag, unsigned int& counter) {
    const double a_vx    = alpha * eps_vx    / Ts;
    const double a_vy    = alpha * eps_vy    / Ts;
    const double a_omega = alpha * eps_omega / Ts;
    virtual_state.vel_x    += a_vx    * config_.lag_compensation_time;
    virtual_state.vel_y    += a_vy    * config_.lag_compensation_time;
    virtual_state.yaw_rate += a_omega * config_.lag_compensation_time;
    solver_.setGPResidual(a_vx, a_vy, a_omega);
    if (counter++ % 50 == 0)
      std::cout << "[" << tag << "] eps=(" << eps_vx << ", " << eps_vy << ", " << eps_omega
                << ")  scale=" << alpha
                << "  a=(" << a_vx << ", " << a_vy << ", " << a_omega << ")" << std::endl;
  };

  // Evaluates the loaded residual model at each stage's predicted state from the
  // previous solver iterate (last_solution_.x[stage], not yet shifted for this cycle),
  // overwriting the constant broadcast that apply_residual() set above per-stage.
  // This is a zero-order-hold approximation, same principle as RTI: the residual is
  // frozen to the previous iterate for the duration of this cycle's SQP solve(s).
  auto apply_residual_per_stage = [&]() {
    for (int stage = 0; stage < solver_.N; ++stage)
    {
      const double vx    = last_solution_.x[stage][static_cast<int>(pacejka_vars::VX)];
      const double vy    = last_solution_.x[stage][static_cast<int>(pacejka_vars::VY)];
      const double omega = last_solution_.x[stage][static_cast<int>(pacejka_vars::DYAW)];
      const double delta = last_solution_.x[stage][static_cast<int>(pacejka_vars::STEER)];
      const double T     = last_solution_.x[stage][static_cast<int>(pacejka_vars::TORQUE)];

      double eps_vx = 0.0, eps_vy = 0.0, eps_omega = 0.0;
      if (mlp_.isLoaded())
      {
        auto d = mlp_.predict(vx, vy, omega, delta, T);
        eps_vx = d.d_vx; eps_vy = d.d_vy; eps_omega = d.d_omega;
      }
      else if (inducing_gp_.isLoaded())
      {
        auto d = inducing_gp_.predict(vx, vy, omega, delta, T, config_.inducing_gp_uncertainty_scaling);
        eps_vx = d.d_vx; eps_vy = d.d_vy; eps_omega = d.d_omega;
      }
      else if (gp_.isLoaded())
      {
        auto d = gp_.predict(vx, vy, omega, delta, T);
        eps_vx = d.d_vx; eps_vy = d.d_vy; eps_omega = d.d_omega;
      }
      else
      {
        continue;
      }

      solver_.setGPResidual(stage, alpha * eps_vx / Ts, alpha * eps_vy / Ts, alpha * eps_omega / Ts);
    }
  };

  if (mlp_.isLoaded())
  {
    auto d = mlp_.predict(state.vel_x, state.vel_y, state.yaw_rate, last_input_.steer, last_input_.torque);
    apply_residual(d.d_vx, d.d_vy, d.d_omega, "MLP", mlp_log_counter_);
  }
  else if (inducing_gp_.isLoaded())
  {
    auto d = inducing_gp_.predict(state.vel_x, state.vel_y, state.yaw_rate, last_input_.steer, last_input_.torque,
                                   config_.inducing_gp_uncertainty_scaling);
    apply_residual(d.d_vx, d.d_vy, d.d_omega, "InducingGP", inducing_gp_log_counter_);
    if (inducing_gp_log_counter_ % 50 == 1 && config_.inducing_gp_uncertainty_scaling)
      std::cout << "[InducingGP] confidence=(" << d.scale_vx << ", " << d.scale_vy << ", " << d.scale_omega << ")"
                << std::endl;
  }
  else if (gp_.isLoaded())
  {
    auto d = gp_.predict(state.vel_x, state.vel_y, state.yaw_rate, last_input_.steer, last_input_.torque);
    apply_residual(d.d_vx, d.d_vy, d.d_omega, "GP", gp_log_counter_);
  }
  else
  {
    solver_.setGPResidual(0.0, 0.0, 0.0);
  }

  if (config_.residual_per_stage)
  {
    apply_residual_per_stage();
  }

  // theta also needs to be adjusted to account for the forward simulation. This is exact since dTheta is
  // piecewise constant.
  theta_ += config_.lag_compensation_time * last_solution_.u[0][pacejka_inputs::DTHETA];

  StateArray x_0 = convertToLocalCoordinates(virtual_state, last_input_, theta_);

  // if lap is done, increase lap counter
  if (theta_ > (laps_ + 1) * max_arc_length)
  {
    laps_++;
  }

  crs_controls::control_commons::shiftSequence(last_solution_.x);
  crs_controls::control_commons::shiftSequence(last_solution_.u);

  for (unsigned int iteration = 0; iteration < config_.max_sqp_iterations; ++iteration)
  {
    MpcParameters parameters = generateCurrentSolverParameters();

    auto solution = solver_.solve_problem(x_0, last_solution_, parameters);
    if (solution.exit_code == crs_controls::control_commons::MpcExitCode::SUCCEEDED)
    {
      last_solution_ = solution.solution.value();
    }
    else
    {
      uint64_t now_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
              .count();
      std::cout << "Solver [ " << std::fixed << uint64_t((now_ms) / 100.0) / 10.0
                << " ]: MPCC exitflag: " << static_cast<int>(solution.exit_code) << std::endl;
    }
  }

  last_input_.torque = last_solution_.x[1][static_cast<int>(pacejka_vars::TORQUE)];
  last_input_.steer = last_solution_.x[1][static_cast<int>(pacejka_vars::STEER)];
  theta_ = last_solution_.x[1][static_cast<int>(pacejka_vars::THETA)];

  if (log_stream_.is_open() && log_pending_)
  {
    const double ref_x = last_reference_on_track_[0].x;
    const double ref_y = last_reference_on_track_[0].y;
    const double phi   = last_reference_on_track_[0].yaw;
    const double dx    = state.pos_x - ref_x;
    const double dy    = state.pos_y - ref_y;
    const double eC    =  std::sin(phi) * dx - std::cos(phi) * dy;
    const double eL    = -std::cos(phi) * dx - std::sin(phi) * dy;
    log_stream_ << std::fixed << std::setprecision(6)
                << log_pend_t_       << ","
                << log_pend_vx_      << "," << log_pend_vy_      << "," << log_pend_omega_ << ","
                << log_pend_delta_   << "," << log_pend_T_        << ","
                << log_pend_eps_vx_  << "," << log_pend_eps_vy_  << "," << log_pend_eps_omega_ << ","
                << eC << "," << eL << ","
                << state.pos_x << "," << state.pos_y << ","
                << ref_x << "," << ref_y << "," << laps_ << "," << theta_ << "\n";
    log_stream_.flush();
    log_pending_ = false;

    log_point_count_++;
    if (config_.log_max_points > 0 && log_point_count_ >= config_.log_max_points)
    {
      log_stream_.close();
      std::cout << "[MPCC] Logging stopped: " << log_point_count_ << " points written." << std::endl;
    }
  }

  return last_input_;
}

template <typename SolverType>
mpcc_pacejka_config& PacejkaMpccController<SolverType>::getConfig()
{
  return config_;
}

template <typename SolverType>
void PacejkaMpccController<SolverType>::setConfig(mpcc_pacejka_config config)
{
  config_ = config;
}

template <typename SolverType>
bool PacejkaMpccController<SolverType>::isInitializing()
{
  return false;
}

template <typename SolverType>
std::shared_ptr<StaticTrackTrajectory> PacejkaMpccController<SolverType>::getStaticTrack()
{
  return std::static_pointer_cast<crs_controls::StaticTrackTrajectory>(trajectory_);
}

template <typename SolverType>
typename PacejkaMpccController<SolverType>::StateArray PacejkaMpccController<SolverType>::convertToLocalCoordinates(
    const crs_models::pacejka_model::pacejka_car_state& global_state,
    const crs_models::pacejka_model::pacejka_car_input& input, double theta)
{
  return StateArray{ global_state.pos_x, global_state.pos_y, global_state.yaw,
                     global_state.vel_x, global_state.vel_y, global_state.yaw_rate,
                     input.torque,       input.steer,        theta };
}

template <typename SolverType>
crs_models::pacejka_model::pacejka_car_state PacejkaMpccController<SolverType>::convertToGlobalCoordinates(
    const typename PacejkaMpccController<SolverType>::StateArray& local_state)
{
  // NOTE(@naefjo): standard MPCC is: x, y, yaw, vx, vy, dyaw.
  return { local_state[0], local_state[1], local_state[2], local_state[3], local_state[4], local_state[5] };
}

template <typename SolverType>
typename PacejkaMpccController<SolverType>::StateArray
PacejkaMpccController<SolverType>::generateInitialization(const int reference_track_index,
                                                          const double current_velocity)
{
  auto track = getStaticTrack();
  const Eigen::Vector2d current_centerline_reference = track->operator[](reference_track_index);
  return {
    current_centerline_reference.x(),
    current_centerline_reference.y(),
    track->getTrackAngle(reference_track_index),
    current_velocity,                                               // v_x
    0.0,                                                            // v_y
    track->getCurvature(reference_track_index) * current_velocity,  // dyaw
    0.0,
    0.0,
    track->getArcLength(reference_track_index),
  };
}
}  // namespace crs_controls::pacejka_mpcc
