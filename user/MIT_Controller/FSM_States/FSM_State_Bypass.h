#ifndef FSM_STATE_BYPASS_H
#define FSM_STATE_BYPASS_H

#include <stdio.h>

#include "FSM_State.h"
#include <lcm-cpp.hpp>
#include <unistd.h>
#include "spi_data_t.hpp"
#include "spi_command_t.hpp"
#include <thread>
#include <vector>


/**
 *
 */
template <typename T>
class FSM_State_Bypass : public FSM_State<T> {
 public:
  FSM_State_Bypass(ControlFSMData<T>* _controlFSMData);

  void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                    const spi_command_t *msg);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  void setstate();

 private:
  // Keep track of the control iterations
  int iter = 0;
  lcm::LCM _bypassLCM;
  std::thread _bypassLCMThread;
  void bypassLCMThread() { while (true) { _bypassLCM.handle(); } }
  Vec4<T> _ini_q_des_abad;
  Vec4<T> _ini_q_des_hip;
  Vec4<T> _ini_q_des_knee;
  Vec4<T> _ini_qd_des_abad;
  Vec4<T> _ini_qd_des_hip;
  Vec4<T> _ini_qd_des_knee;
  Vec4<T> _ini_tau_abad_ff;
  Vec4<T> _ini_tau_hip_ff;
  Vec4<T> _ini_tau_knee_ff;  
  Vec4<T> _ini_kp_abad;
  Vec4<T> _ini_kp_hip;
  Vec4<T> _ini_kp_knee;
  Vec4<T> _ini_kd_abad;
  Vec4<T> _ini_kd_hip;
  Vec4<T> _ini_kd_knee;
};

#endif  // FSM_STATE_BYPASS_H
