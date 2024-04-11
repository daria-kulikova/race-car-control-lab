#include "lighthouse_calibration_solver.h"

#include <cmath>
#include <casadi/casadi.hpp>

namespace ros_lighthouse
{

bool LighthouseCalibrationSolver::solve(const BaseStation& base_station, const CalibrationDataset& calibration_data,
                                        const LighthousePose& initial_condition)
{
  auto point_coordinates = calibration_data.first;
  auto average_angles = calibration_data.second;

  // Check if the passed data is valid
  int point_count = point_coordinates.size();

  if (point_count != average_angles.size())
  {
    throw std::invalid_argument("The number of points in angles and positions don't match.");
  }

  // Calculate the light plane angles from the factory calibrated offsets.
  double light_plane_tilt1 = -M_PI / 6 - base_station.dt1.value();
  double light_plane_tilt2 = M_PI / 6 - base_station.dt2.value();

  // Set up the CasADi optimization problem
  casadi::Opti opti;

  // Set up the optimization variables
  casadi::MX bs_pos_x = opti.variable();
  casadi::MX bs_pos_y = opti.variable();
  casadi::MX bs_pos_z = opti.variable();

  casadi::MX bs_angle_z = opti.variable();
  casadi::MX bs_angle_y = opti.variable();
  casadi::MX bs_angle_x = opti.variable();

  // Calculate the rotation matrix as a function of the optimization variables
  Eigen::Matrix<casadi::MX, 3, 3> rot_bsZ;
  Eigen::Matrix<casadi::MX, 3, 3> rot_bsY;
  Eigen::Matrix<casadi::MX, 3, 3> rot_bsX;

  rot_bsZ << cos(bs_angle_z), -sin(bs_angle_z), 0, sin(bs_angle_z), cos(bs_angle_z), 0, 0, 0, 1;
  rot_bsY << cos(bs_angle_y), 0, sin(bs_angle_y), 0, 1, 0, -sin(bs_angle_y), 0, cos(bs_angle_y);
  rot_bsX << 1, 0, 0, 0, cos(bs_angle_x), -sin(bs_angle_x), 0, sin(bs_angle_x), cos(bs_angle_x);

  // rot = rotZ * rotY * rotX; pos = [pos_x; pos_y; pos_z]
  Eigen::Matrix<casadi::MX, 3, 3> rot_bs = rot_bsZ * rot_bsY * rot_bsX;
  Eigen::Matrix<casadi::MX, 3, 1> pos_bs = { bs_pos_x, bs_pos_y, bs_pos_z };

  // Convert the calibration data to casadi matrices
  Eigen::Matrix<casadi::MX, 3, -1> calib_pos = Eigen::Matrix<casadi::MX, 3, -1>(3, point_count);
  Eigen::Matrix<casadi::MX, 2, -1> calib_angle = Eigen::Matrix<casadi::MX, 2, -1>(2, point_count);

  for (size_t i = 0; i < point_count; i++)
  {
    calib_pos(0, i) = point_coordinates[i].first;
    calib_pos(1, i) = point_coordinates[i].second;
    calib_pos(2, i) = 0;
    calib_angle(0, i) = average_angles[i].first;
    calib_angle(1, i) = average_angles[i].second;
  }

  // Positions in base station reference frame
  Eigen::Matrix<casadi::MX, 3, -1> sbs = rot_bs.transpose() * (calib_pos.colwise() - pos_bs);

  // Calculate angles
  Eigen::Matrix<casadi::MX, 1, -1> alphas = (sbs.row(1).array() / sbs.row(0).array()).atan();
  Eigen::Matrix<casadi::MX, 1, -1> alpha1 =
      alphas.array() + ((sbs.row(2) * tan(light_plane_tilt1)).array() /
                        (sbs.row(0).array().square() + sbs.row(1).array().square()).sqrt())
                           .asin();
  Eigen::Matrix<casadi::MX, 1, -1> alpha2 =
      alphas.array() + ((sbs.row(2) * tan(light_plane_tilt2)).array() /
                        (sbs.row(0).array().square() + sbs.row(1).array().square()).sqrt())
                           .asin();

  // Optimization function: minimize error in angles
  casadi::MX f = 0;
  for (size_t i = 0; i < point_count; i++)
  {
    casadi::MX error1 = alpha1(i) - calib_angle(0, i);
    casadi::MX error2 = alpha2(i) - calib_angle(1, i);
    f += error1 * error1 + error2 * error2;
  }

  // Set objective
  opti.minimize(f);

  // Set up solver
  casadi::Dict solver_options = {};

  if (!print_debug_information_)
  {
    // Suppress output of solver when not in debug mode
    solver_options = {
      { "ipopt.print_level", 0 },  // don't print anything when iterating
      { "ipopt.sb", "yes" },       // don't print "this is IPOPT" banner
      { "print_time", 0 },         // don't print timing summary
    };
  }

  opti.solver("ipopt", solver_options);

  // Set up the initial guess of the solver
  opti.set_initial({
      bs_pos_x == initial_condition.position[0],
      bs_pos_y == initial_condition.position[1],
      bs_pos_z == initial_condition.position[2],
      bs_angle_z == initial_condition.angles[0],
      bs_angle_y == initial_condition.angles[1],
      bs_angle_x == initial_condition.angles[2],
  });

  // Solve the optimization problem
  try
  {
    casadi::OptiSol sol = opti.solve();  // will throw on failure

    std::vector<double> solution = {
      (double)sol.value(bs_angle_z), (double)sol.value(bs_angle_y), (double)sol.value(bs_angle_x),
      (double)sol.value(bs_pos_x),   (double)sol.value(bs_pos_y),   (double)sol.value(bs_pos_z),
    };

    // Calculate the rotation matrix from the solution.
    Eigen::Matrix3d rot_solutionZ;
    Eigen::Matrix3d rot_solutionY;
    Eigen::Matrix3d rot_solutionX;
    rot_solutionZ << cos(solution[0]), -sin(solution[0]), 0, sin(solution[0]), cos(solution[0]), 0, 0, 0, 1;
    rot_solutionY << cos(solution[1]), 0, sin(solution[1]), 0, 1, 0, -sin(solution[1]), 0, cos(solution[1]);
    rot_solutionX << 1, 0, 0, 0, cos(solution[2]), -sin(solution[2]), 0, sin(solution[2]), cos(solution[2]);

    rotation_ = rot_solutionZ * rot_solutionY * rot_solutionX;

    // Calculate the position from the solution.
    position_ = { solution[3], solution[4], solution[5] };

    return true;
  }
  catch (std::exception& e)
  {
    std::cout << "Solver did not converge: " << e.what() << std::endl;
    return false;
  }
}

}  // namespace ros_lighthouse
