/*============================== Bypass ==============================*/
/**
 * Bypass State collaborates with IHMC code that allows the user bypass the MIT Code
   and control the robot directly.
 */

#include "FSM_State_Bypass.h"


/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */


template <typename T>
FSM_State_Bypass<T>::FSM_State_Bypass(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::BYPASS, "BYPASS"),_bypassLCM(getLcmUrl(255)){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
  if (!_bypassLCM.good())
    return;

  // BPHandler handlerObject;
  // _BYPASSLCM.subscribe("spi_command_IHMC", &BPHandler::handleMessage, &handlerObject);
  _bypassLCM.subscribe("spi_command_IHMC", &FSM_State_Bypass<T>::handleMessage, this);
  _bypassLCMThread = std::thread(&FSM_State_Bypass<T>::bypassLCMThread, this);

}

template <typename T>
void FSM_State_Bypass<T>::handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                    const spi_command_t *msg)
{   (void)rbuf;
    printf("Received message on channel \"%s\":\n", chan.c_str());
    printf("  q_des_abad   = %f %f %f %f \n", (float) msg->q_des_abad[0],(float) msg->q_des_abad[1],(float) msg->q_des_abad[2],(float) msg->q_des_abad[3]);
    printf("  q_des_hip   = %f %f %f %f \n", (float) msg->q_des_hip[0],(float) msg->q_des_hip[1],(float) msg->q_des_hip[2],(float) msg->q_des_hip[3]);
    printf("  q_des_knee   = %f %f %f %f \n", (float) msg->q_des_knee[0],(float) msg->q_des_knee[1],(float) msg->q_des_knee[2],(float) msg->q_des_knee[3]);
    printf("  qd_des_abad   = %f %f %f %f \n", (float) msg->qd_des_abad[0],(float) msg->qd_des_abad[1],(float) msg->qd_des_abad[2],(float) msg->qd_des_abad[3]);
    printf("  qd_des_hip   = %f %f %f %f \n", (float) msg->qd_des_hip[0],(float) msg->qd_des_hip[1],(float) msg->qd_des_hip[2],(float) msg->qd_des_hip[3]);
    printf("  qd_des_knee   = %f %f %f %f \n", (float) msg->qd_des_knee[0],(float) msg->qd_des_knee[1],(float) msg->qd_des_knee[2],(float) msg->qd_des_knee[3]);
    printf("  kp_abad   = %f %f %f %f \n", (float) msg->kp_abad[0],(float) msg->kp_abad[1],(float) msg->kp_abad[2],(float) msg->kp_abad[3]);
    printf("  kp_hip   = %f %f %f %f \n", (float) msg->kp_hip[0],(float) msg->kp_hip[1],(float) msg->kp_hip[2],(float) msg->kp_hip[3]);
    printf("  kp_knee   = %f %f %f %f \n", (float) msg->kp_knee[0],(float) msg->kp_knee[1],(float) msg->kp_knee[2],(float) msg->kp_knee[3]);
    printf("  kd_abad   = %f %f %f %f \n", (float) msg->kd_abad[0],(float) msg->kd_abad[1],(float) msg->kd_abad[2],(float) msg->kd_abad[3]);
    printf("  kd_hip   = %f %f %f %f \n", (float) msg->kd_hip[0],(float) msg->kd_hip[1],(float) msg->kd_hip[2],(float) msg->kd_hip[3]);
    printf("  kd_knee   = %f %f %f %f \n", (float) msg->kd_knee[0],(float) msg->kd_knee[1],(float) msg->kd_knee[2],(float) msg->kd_knee[3]);
    printf("  tau_abad_ff   = %f %f %f %f \n", (float) msg->tau_abad_ff[0],(float) msg->tau_abad_ff[1],(float) msg->tau_abad_ff[2],(float) msg->tau_abad_ff[3]);
    printf("  tau_hip_ff   = %f %f %f %f \n", (float) msg->tau_hip_ff[0],(float) msg->tau_hip_ff[1],(float) msg->tau_hip_ff[2],(float) msg->tau_hip_ff[3]);
    printf("  tau_knee_ff   = %f %f %f %f \n", (float) msg->tau_knee_ff[0],(float) msg->tau_knee_ff[1],(float) msg->tau_knee_ff[2],(float) msg->tau_knee_ff[3]);
    printf("  flags   = %lf %lf %lf %lf \n", (float) msg->flags[0],(float) msg->flags[1],(float) msg->flags[2],(float) msg->flags[3]);

    _ini_q_des_abad.setZero();
    _ini_q_des_hip.setZero();
    _ini_q_des_knee.setZero();
    _ini_qd_des_abad.setZero();
    _ini_qd_des_hip.setZero();
    _ini_qd_des_knee.setZero();
    _ini_tau_abad_ff.setZero();
    _ini_tau_hip_ff.setZero();
    _ini_tau_knee_ff.setZero();
    _ini_kp_abad.setZero();
    _ini_kp_hip.setZero();
    _ini_kp_knee.setZero();
    _ini_kd_abad.setZero();
    _ini_kd_hip.setZero();
    _ini_kd_knee.setZero();

    for (int leg = 0; leg < 4; ++leg) {
      _ini_q_des_abad[leg] = msg->q_des_abad[leg];
      _ini_q_des_hip[leg] = msg->q_des_hip[leg];
      _ini_q_des_knee[leg] = msg->q_des_knee[leg];
      _ini_qd_des_abad[leg] = msg->qd_des_abad[leg];
      _ini_qd_des_hip[leg] = msg->qd_des_hip[leg];
      _ini_qd_des_knee[leg] = msg->qd_des_knee[leg];
      _ini_tau_abad_ff[leg] = msg->tau_abad_ff[leg];
      _ini_tau_hip_ff[leg] = msg->tau_hip_ff[leg];
      _ini_tau_knee_ff[leg] = msg->tau_knee_ff[leg];
      _ini_kp_abad[leg] = msg->kp_abad[leg];
      _ini_kp_hip[leg] = msg->kp_hip[leg];
      _ini_kp_knee[leg] = msg->kp_knee[leg];
      _ini_kd_abad[leg] = msg->kd_abad[leg];
      _ini_kd_hip[leg] = msg->kd_hip[leg];
      _ini_kd_knee[leg] = msg->kd_knee[leg];
    }

}


template <typename T>
void FSM_State_Bypass<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;
  // printf("  hehehehehehhehhehehehehehehehehee_des_abad   = %f \n", _ini_q_des_abad[0]);
  // Reset the transition data
  this->transitionData.zero();
  printf("[FSM BYPASS] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
   Keep listening if there is any incoming packet
 */
template <typename T>
void FSM_State_Bypass<T>::run() {
  // Listen and Do nothing
  setstate();
}

/**
 * Handles LCM Message
 * Returns true when the LCM is received
 *
 * @return true 
 */
template <typename T>
void FSM_State_Bypass<T>::setstate() {
  for(size_t leg(0); leg<4; ++leg){
    this->_data->_legController->commands[leg].qDes[0] = _ini_q_des_abad[leg];
    this->_data->_legController->commands[leg].qDes[1] = _ini_q_des_hip[leg];
    this->_data->_legController->commands[leg].qDes[2] = _ini_q_des_knee[leg];
    this->_data->_legController->commands[leg].qdDes[0] = _ini_qd_des_abad[leg];
    this->_data->_legController->commands[leg].qdDes[1] = _ini_qd_des_hip[leg];
    this->_data->_legController->commands[leg].qdDes[2] = _ini_qd_des_knee[leg];
    this->_data->_legController->commands[leg].tauFeedForward[0] = _ini_tau_abad_ff[leg];
    this->_data->_legController->commands[leg].tauFeedForward[1] = _ini_tau_hip_ff[leg];
    this->_data->_legController->commands[leg].tauFeedForward[2] = _ini_tau_knee_ff[leg];
    this->_data->_legController->commands[leg].kpJoint(0,0) = _ini_kp_abad[leg];
    this->_data->_legController->commands[leg].kpJoint(1,1) = _ini_kp_hip[leg];
    this->_data->_legController->commands[leg].kpJoint(2,2) = _ini_kp_knee[leg];
    this->_data->_legController->commands[leg].kdJoint(0,0) = _ini_kd_abad[leg];
    this->_data->_legController->commands[leg].kdJoint(1,1) = _ini_kd_hip[leg];
    this->_data->_legController->commands[leg].kdJoint(2,2) = _ini_kd_knee[leg];
  }
    // commands elements
    // Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
    // Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;

}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Bypass<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_BYPASS:  // normal c (0)
      // Normal operation for state based transitions
      break;

    case K_PASSIVE:  
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_BYPASS << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */

template <typename T>
TransitionData<T> FSM_State_Bypass<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Bypass<T>::onExit() {
  // nothing to clean up
}

// template class FSM_State_Bypass<double>;
template class FSM_State_Bypass<float>;

