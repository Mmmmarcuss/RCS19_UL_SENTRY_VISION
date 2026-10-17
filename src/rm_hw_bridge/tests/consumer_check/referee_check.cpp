#include <iostream>
#include "referee_interface.hpp"
int main() {
  RefereeInterface input;
  RobotContext context;
  rm_interfaces::msg::GimbalStatus status;
  status.game_progress = 4;
  status.current_hp = 300;
  status.maximum_hp = 400;
  status.referee_data_valid = false;
  input.UpdateFromStatus(status);
  input.SyncToContext(context);
  if (context.referee_link_fresh || context.match_started) return 1;
  status.referee_data_valid = true;
  input.UpdateFromStatus(status);
  input.SyncToContext(context);
  if (!context.referee_link_fresh || !context.match_started) return 2;
  std::cout << "PASS: missing referee state stays unavailable; legacy full status remains valid\n";
}
