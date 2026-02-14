#ifndef WINDOW_ROS2_XBOX_JOYSTICK_HPP_
#define WINDOW_ROS2_XBOX_JOYSTICK_HPP_

#include <functional>
#include <windows.h>
#include <Xinput.h>
#pragma comment(lib, "Xinput9_1_0.lib") 

struct XboxState
{
  float lx, ly;
  float rx, ry;
  float lt, rt;
  bool auto_mode; // (false = manual mode)
  bool b_pressed;
};

class XboxJoystick
{
  using StateCallback = std::function<void(const XboxState&)>;

public:
  /// @brief constructor
  XboxJoystick() = default;
  /// @brief destructor
  ~XboxJoystick() = default;

  /// @brief callback hook
  void set_callback(StateCallback cb);

  /// @brief controller actions
  void poll();

  /// @brief xbox vibration motion
  void vibrate_motion(WORD leftMotor=35535, WORD rightMotor=35535);

protected:
  inline bool is_button_pressed(WORD current, WORD previous, WORD buttonMask)
  {
    return (current & buttonMask) && !(previous & buttonMask);
  }


private:
  /// @brief callback function to keep
  StateCallback callback_;
  /// @brief previous input state
  XINPUT_STATE prev_state_{};
  /// @brief current input state
  XINPUT_STATE current_state_{};
  /// @brief trigger flag
  bool trigger_{false};
  /// @brief controller id (default: 0)
  DWORD user_id_{0};
};

#endif /* WINDOW_ROS2_XBOX_JOYSTICK_HPP_ */