#include <vector>
#include <cmath>
#include <algorithm>

float L_1 = 4; //servo arm
float L_2 = 9.8; //forearm
float x = 1.0; // Avstand fra tilkoplingspunkt plate til servo aktuator i x-planet
float L = 22.5;
double h = L * std::sqrt(3.0/ 4.0);
// Maxmin servo angles
float servo_angle_limit_max = 85;
float servo_angle_limit_min = -35;

double pos1_matrix[4] = {
    -L/2.0,
    -h/3.0,
    0.0,
    1.0};
double pos2_matrix[4] = {
    L/2.0,
    -h/3.0,
    0.0,
    1.0};
double pos3_matrix[4] = {
    0.0,
    2.0*h/3.0,
    0.0,
    1.0};

class PID
{
private:
  double kp, ki, kd;
  bool integral_limit = false;
  int integral_cap = 20;
  double integral; // Integral accumulator
  double previous_error;
  std::chrono::steady_clock::time_point last_time;

public:
  PID(double p, double i, double d) : kp(p), ki(i), kd(d), integral(0.0), previous_error(0.0)
  {
    last_time = std::chrono::steady_clock::now();
  }

  double compute(double setpoint, double ball_pos)
  {
    // Current time
    auto now = std::chrono::steady_clock::now();

    // Calculate dt
    std::chrono::duration<double> dt = now - last_time;
    double delta_time = dt.count();
    delta_time = 0.5;
    // Calculate the error
    double error = setpoint - ball_pos;

    // Integral term
    integral += error * delta_time;

    // Integral windup protection
    if (integral_limit)
    {
      if (integral > integral_cap)
      {
        integral = integral_cap;
      }
      else if (integral < -integral_cap)
      {
        integral = -integral_cap;
      }
    }
    // Derivative term
    double derivative = (error - previous_error) / delta_time;
    // PID output
    double output = kp * error + ki * integral + kd * derivative;

    // Update previous error and time
    previous_error = error;
    last_time = now;

    return output;
  }
};





void multiply(int m1, int m2, int mat1[][2], int n1, int n2,
              int mat2[][2])
{
    int x, i, j;
    int res[m1][n2];
    for (i = 0; i < m1; i++) {
        for (j = 0; j < n2; j++) {
            res[i][j] = 0;
            for (x = 0; x < m2; x++) {
                *(*(res + i) + j) += *(*(mat1 + i) + x)
                                     * *(*(mat2 + x) + j);
            }
        }
    }
    for (i = 0; i < m1; i++) {
        for (j = 0; j < n2; j++) {
            std::cout << *(*(res + i) + j) << " ";
        }
        std::cout << "\n";
    }
}


std::vector<std::vector<double>> newPos(float gamma_deg, float theta_deg, double pos[4], float z_){
    // GAMMA = PITCH   THETA = ROLL
    const double gamma_rad = M_PI/180 * gamma_deg;
    const double theta_rad = M_PI/180 * theta_deg;
    const double rotation_matrix[4][4] = {
        {std::cos(gamma_rad) , std::sin(gamma_rad) * std::sin(theta_rad), std::sin(gamma_rad) * std::cos(theta_rad), 0},
        {0.0, std::cos(theta_rad), -std::sin(theta_rad), 0.0},
        {-std::sin(gamma_rad), std::cos(gamma_rad) * std::sin(theta_rad), std::cos(gamma_rad)*std::cos(theta_rad), z_},
        {0.0, 0.0, 0.0, 1.0}};
    std::vector<std::vector<double>> newPositionVector = {{0},{0},{0},{0}};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newPositionVector[i][0] += rotation_matrix[i][j]*pos[j];
        }
    }

    // for (int i = 0; i < 4; i++)
    // {
    //     std::cout <<  i << ":   "   << newPositionVector[i][0] << std::endl;
    // }
    return newPositionVector;
}



std::vector<double> next_servo_pos(float pitch_deg, float roll_deg, float z_)
{
    const std::vector<std::vector<double>> a = newPos(pitch_deg, roll_deg, pos1_matrix, z_);
    const std::vector<std::vector<double>> b = newPos(pitch_deg, roll_deg, pos2_matrix, z_);
    const std::vector<std::vector<double>> c = newPos(pitch_deg, roll_deg, pos3_matrix, z_);


    double ball_heights[3] = {a[2][0], b[2][0], c[2][0]};
    std::cout << "Z height. Point: " << ball_heights[0] << " Point 2: " << ball_heights[1] << " Point 3: " << ball_heights[2] << std::endl;
    const float arm_l = L_1;
    const float forearm_l = L_2;
    // std::cout << "servo test angle: " << theta1_deg << std::endl;
    std::vector<double> servo_ang_rad_test;
    std::vector<double> servo_ang_deg_test;
    for (const double y : ball_heights)
    {

        const double theta2 = +std::acos((x*x+y*y-arm_l*arm_l-forearm_l*forearm_l)/(2*forearm_l*arm_l));
        double theta1 = std::atan2(y, x) - std::atan2(forearm_l * std::sin(theta2), arm_l + forearm_l * std::cos(theta2));
        double theta1_deg = theta1 * 180/M_PI;
        servo_ang_rad_test.push_back(theta1);
        servo_ang_deg_test.push_back(theta1_deg);
    };
    std::cout << "Servo angles radians, 1: " << servo_ang_rad_test[0] << " 2: " << servo_ang_rad_test[1] << " 3: " << servo_ang_rad_test[2] << std::endl;
    std::cout << "Servo angles degree :) 1: " << servo_ang_deg_test[0] << " 2: " << servo_ang_deg_test[1] << " 3: " << servo_ang_deg_test[2] << std::endl;
    return (servo_ang_deg_test);
}


std::vector<double> servo_angle_outputs(std::vector<double> angles_){
  std::vector<double> output;

  for (int i = 0; i < 3; i++){
    if (angles_[i] >= servo_angle_limit_max){
      angles_[i] = servo_angle_limit_max - 1;
    }
    else if (angles_[i] <= servo_angle_limit_min)
    {
      angles_[i] = servo_angle_limit_min + 1;
    }
  output.push_back(angles_[i]);
  }
  return output;
}