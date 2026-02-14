#include "xbox_controller/xbox_joystick.hpp"


void XboxJoystick::set_callback(StateCallback cb)
{
  callback_ = cb;
}

void XboxJoystick::poll()
{
  prev_state_ = current_state_;

  if (XInputGetState(user_id_, &current_state_) != ERROR_SUCCESS)
    return;

  auto prev_buttons = prev_state_.Gamepad.wButtons;
  auto curr_buttons = current_state_.Gamepad.wButtons;

  // edge detection for A button
  if (this->is_button_pressed(curr_buttons, prev_buttons, XINPUT_GAMEPAD_A))
  {
    trigger_ = !trigger_;
  }

  XboxState s;
  s.lx = current_state_.Gamepad.sThumbLX / 32767.0f;
  s.ly = current_state_.Gamepad.sThumbLY / 32767.0f;
  s.rx = current_state_.Gamepad.sThumbRX / 32767.0f;
  s.ry = current_state_.Gamepad.sThumbRY / 32767.0f;
  s.lt = current_state_.Gamepad.bLeftTrigger / 255.0f;
  s.rt = current_state_.Gamepad.bRightTrigger / 255.0f;

  s.auto_mode = trigger_;
  s.b_pressed = current_state_.Gamepad.wButtons & XINPUT_GAMEPAD_B;

  if (callback_)
    callback_(s);
}

void XboxJoystick::vibrate_motion(WORD leftMotor, WORD rightMotor)
{
  XINPUT_VIBRATION vibration{};
  ZeroMemory(&vibration, sizeof(XINPUT_VIBRATION));

  // 0 - 65535
  // vibrate
  vibration.wLeftMotorSpeed  = leftMotor;
  vibration.wRightMotorSpeed = rightMotor;
  XInputSetState(user_id_, &vibration);

  Sleep(300);

  // turn off
  vibration.wLeftMotorSpeed = 0;
  vibration.wRightMotorSpeed = 0;
  XInputSetState(user_id_, &vibration);
}