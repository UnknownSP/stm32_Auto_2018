#include "app.h"
#include "DD_Gene.h"
#include "DD_RCDefinition.h"
#include "SystemTaskManager.h"
#include <stdlib.h>
#include <stdbool.h>
#include "MW_GPIO.h"
#include "MW_IWDG.h"
#include "message.h"
#include "MW_flash.h"
#include "constManager.h"
#include "trapezoid_ctrl.h"

static
SuspensionMoveState_t cal_omni_value(TargetMoveDirection_t direction, SuspensionStopType_t type, int32_t target_value, int32_t now_value, int offset_x, int offset_y, int offset_z);

static
ShotMechaResetState_t pull_mecha_reset(void);

static
ShotMechaResetState_t release_mecha_reset(void);

static
ShotMechaResetState_t degree_mecha_reset(void);

static
ShotMechaResetState_t pull_rele_mecha_reset(void);

static
ShotMechaSetState_t release_mecha_set_target(int32_t target_value, int32_t now_value, int reset);

static
ShotMechaSetState_t degree_mecha_set_target(int32_t target_value, int32_t now_value, int reset);

static
ShotMechaSetState_t shotmecha_set_waitposition(void);

static
AdjustLineState_t move_linecenter(int32_t straight_duty, int32_t time);

static
ShotState_t auto_shot(const int32_t power, const int32_t degree, int reset);

static
ShotState_t shot(void);

static
int Pull_mecha_stop(void);

static
int release_mecha_stop(void);

static
int suspension_stop(void);

static
int all_motor_stop(void);

static
int change_buttonflag(uint8_t *panel1, uint8_t *panel2);

static
int change_binary(uint16_t value, int *arr);

static
LineTrace_State_t find_robotdirection(uint8_t *front, uint8_t *behind, Robot_Direction_t *robot_direction, int reset ,Direction_t reset_direction);

static
SuspensionMoveState_t go_to_target(Desk_t target, Desk_t now_place, int32_t distance_to_wall, GameZone_t zone, int reset);

static
int linetrace_duty_decide(LineTrace_State_t state, int cal_duty);

static
int32_t I2C_Encoder(int32_t encoder_num, EncoderOperation_t operation);

static
int Led_brink(LedMode_t mode, int brink_interval ,int duty);

static
int manual_suspensionsystem(void);

int cal_abs(int x,int y);
double cal_root(double s);

static int32_t distancetowall_desk_a = RESET_DESK_A_DISTANCE;
static int32_t distancetowall_desk_b = RESET_DESK_B_DISTANCE;
static int32_t distancetowall_desk_c = RESET_DESK_C_DISTANCE;
static int32_t distancetowall_desk_d = 7000;

static int32_t desk_a_degree = -20200;
static int32_t desk_a_power = 200;

static int32_t desk_b_degree = -26500;
static int32_t desk_b_power = 385;

static int32_t desk_c_degree = -37300;
static int32_t desk_c_power = 650;

static int32_t desk_d_top_degree = -28500;
static int32_t desk_d_top_power = 50;

static int32_t desk_d_bottom_degree = -37300;
static int32_t desk_d_bottom_power = 650;

static int32_t desk_e_degree = -36000;
static int32_t desk_e_power = 700;

static uint16_t sensor_area_a = 0;
static uint16_t sensor_area_b = 0;
static uint16_t sensor_area_c = 0;
static uint16_t sensor_area_emergency = 0;

static bool panel_sw[32];

/*メモ
 *g_ab_h...ABのハンドラ
 *g_md_h...MDのハンドラ
 *
 *g_rc_data...RCのデータ
 */


static char *linetrace_state_name[] = {
  "RECENT_DATA","CURRENT_DATA","RESET_DATA",
};

static char *linetrace_direction_name[] = {
  "RIGHT","LEFT","FRONT_RIGHT","FRONT_LEFT","MIDDLE","NO_LINE",
};

static char *testmode_name[] = {
  "MOVE_PULLMECHA","MOVE_RELEASEMECHA","MOVE_DEGREEMECHA","RESET_SHOTMECHA",
  "RESET_DEGREEMECHA","SET_DEGREEMECHA",
  "AUTO_SHOT","MANUAL_SUSPENSION",
  "AUTO_SUS_FRONT","AUTO_SUS_BACK","AUTO_SUS_LEFT","AUTO_SUS_RIGHT",
  "MOVE_SUS_LF","MOVE_SUS_LB","MOVE_SUS_RB","MOVE_SUS_RF",
  "TRACE_LINE","TRACE_UNTIL_FRONT_TOUCH",
  "MOVE_LEFT_LINE_C","MOVE_LEFT_LINE_S","MOVE_LEFT_LINE_M","MOVE_LEFT_LINE_L",
  "ALL_MOVING_DESK_A","ALL_MOVING_DESK_B","ALL_MOVING_DESK_C",
  "ALL_MOVING_DESK_D_TOP","ALL_MOVING_DESK_D_BOTTOM","ALL_MOVING_DESK_E",
  "NO_OPERATION",
  "STOP_EVERYTHING",
};

int appInit(void){

  ad_init();

  /*GPIO の設定などでMW,GPIOではHALを叩く*/
  return EXIT_SUCCESS;
}

/*application tasks*/
int appTask(void){
  static GameZone_t zone = RED;
  
  static SuspensionMoveState_t sus_state = MAXSPEED;
  static ShotMechaResetState_t pullmecha = FIRSTRESET;
  static ShotMechaResetState_t releasemecha = FIRSTRESET;
  static ShotMechaResetState_t pull_rele_mecha = FIRSTRESET;
  static ShotMechaResetState_t degreemecha = FIRSTRESET;
  static ShotMechaSetState_t releasemecha_set = SET_READY;
  static ShotMechaSetState_t degreemecha_set = SET_READY;
  static ShotMechaSetState_t waitposition_set = SET_READY;
  static ShotState_t shot_state = SHOTMECHA_RESET;
  static ShotActionState_t shot_action = RESET_MECHA;
  static int32_t releasemecha_encoder = 0;
  static int32_t degreemecha_encoder = 0;
  static AdjustLineState_t movecenterline = ADJUST_READY;
  static int32_t x_encoder = 0;
  static int32_t y_encoder = 0;
  int linetrace_duty = 0;
  int linetrace_strate_duty = 0;

  static bool encoder_move_end = false;

  static bool test_flag = false;
  static int testmode_target[2] = {0,0};
  static int bairitu = 1;
  static int index = 0;
  
  static bool circle_flag = true;
  static bool triangle_flag = true;
  static bool cross_flag = true;
  static bool sqare_flag = true;
  static bool up_flag = true;
  static bool down_flag = true;
  static bool right_flag = true;
  static bool left_flag = true;
  static bool r1_flag = true;
  static bool r2_flag = true;
  static bool l1_flag = true;
  static bool l2_flag = true;
  static bool l1_turn_flag = false;
  static bool l2_turn_flag = false;
  
  static bool test_move_continue = false;

  static int stop_robotturn_count = 0;

  static Robot_Direction_t recent_robot_direction = {
    .gap_degree = 0,
    .direction = NO_LINE,
  };

  static Robot_Direction_t robot_direction = {
    .gap_degree = 0,
    .direction = NO_LINE,
  };
  static LineTrace_State_t linetrace_state = RESET_DATA;

  static TestMode_t test_mode = STOP_EVERYTHING;
  
  int ret=0;
  const tc_const_t testmode_tcon = {
    .inc_con = 750,
    .dec_con = 750,
  };

  static bool move_go_end = false;
  static bool move_return_end = false;
  static bool shot_end = false;
  static bool adjust_line = false;
  static bool wait_receive = false;

  static int wait_count = 0;
  static int count = 0;

  static GameZone_t gamezone = RED;
  static TestMode_t real_mode = STOP_EVERYTHING;
  static bool real_flag = false;
  static int spin_direction = 1;
  const tc_const_t realmode_tcon = {
    .inc_con = 750,
    .dec_con = 750,
  };

  static bool power_up_flag = false;
  static bool power_down_flag = false;
  static bool start_flag = false;
  static bool manual_move = false;
  
  count++;
  
  if(__RC_ISPRESSED_R1(g_rc_data)&&__RC_ISPRESSED_R2(g_rc_data)&&
     __RC_ISPRESSED_L1(g_rc_data)&&__RC_ISPRESSED_L2(g_rc_data)){
    while(__RC_ISPRESSED_R1(g_rc_data)||__RC_ISPRESSED_R2(g_rc_data)||
	  __RC_ISPRESSED_L1(g_rc_data)||__RC_ISPRESSED_L2(g_rc_data))
        SY_wait(10);
    ad_main();
  }

#if USE_CONTROL_PANEL

  change_buttonflag(g_ss_h[CONTROL_PANEL_1].data,g_ss_h[CONTROL_PANEL_2].data);

  if(panel_sw[SW_FUN]){
    g_md_h[8].duty = 6000;
    g_md_h[8].mode = D_MMOD_FORWARD;
  }else{
    g_md_h[8].duty = 0;
    g_md_h[8].mode = D_MMOD_FREE;
  }
  
  if(panel_sw[SW_START]){
    real_flag = true;
  }

  if(panel_sw[SW_STOP]){
    real_mode = STOP_EVERYTHING;
    all_motor_stop();
    real_flag = false;
  }
  
  if(panel_sw[SW_MV_FORWARD]){
    spin_direction = 1;
  }else if(panel_sw[SW_MV_BACKWARD]){
    spin_direction = -1;
  }

  if(!real_flag){

    if(panel_sw[SW_ZONE_RED]){
      gamezone = RED;
    }else if(panel_sw[SW_ZONE_BLUE]){
      gamezone = BLUE;
    }

    if(!panel_sw[SW_POWER_UP]){
      power_up_flag = true;
    }
    if(!panel_sw[SW_POWER_DOWN]){
      power_down_flag = true;
    }

    if(panel_sw[SW_LED_BRINK]){
      Led_brink(ON,LED_BRINK_INTERVAL,LED_BRINK_DUTY);
    }else{
      Led_brink(OFF,LED_BRINK_INTERVAL,LED_BRINK_DUTY);
    }
    
    if(panel_sw[SW_MV_1]){
      trapezoidCtrl(PULLMECHA_MAX_DUTY*spin_direction, &g_md_h[PULLMECHA], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[PULLMECHA], &realmode_tcon);
    }

    if(panel_sw[SW_MV_2]){
      trapezoidCtrl(RELEASEMECHA_MAX_DUTY*spin_direction, &g_md_h[RELEASEMECHA], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[RELEASEMECHA], &realmode_tcon);
    }

    if(panel_sw[SW_MV_3]){
      trapezoidCtrl(DEGREEMECHA_MAX_DUTY*spin_direction, &g_md_h[DEGREEMECHA], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[DEGREEMECHA], &realmode_tcon);
    }

    if(panel_sw[SW_MV_4]){
      trapezoidCtrl(REALMODE_SUS_MAXDUTY*spin_direction, &g_md_h[KUDO_LF], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[KUDO_LF], &realmode_tcon);
    }

    if(panel_sw[SW_MV_5]){
      trapezoidCtrl(REALMODE_SUS_MAXDUTY*spin_direction, &g_md_h[KUDO_LB], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[KUDO_LB], &realmode_tcon);
    }

    if(panel_sw[SW_MV_6]){
      trapezoidCtrl(REALMODE_SUS_MAXDUTY*spin_direction, &g_md_h[KUDO_RB], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[KUDO_RB], &realmode_tcon);
    }

    if(panel_sw[SW_MV_7]){
      trapezoidCtrl(REALMODE_SUS_MAXDUTY*spin_direction, &g_md_h[KUDO_RF], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[KUDO_RF], &realmode_tcon);
    }

    if(panel_sw[SW_MV_1] || panel_sw[SW_MV_2] || panel_sw[SW_MV_3] || panel_sw[SW_MV_4] || panel_sw[SW_MV_5] || panel_sw[SW_MV_6] || panel_sw[SW_MV_7]){
      real_mode = NO_OPERATION;
      manual_move = true;
    }else{
      manual_move = false;
    }    
  
    if(panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_A;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_B;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_C;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_D_TOP;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_D_BOTTOM;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && (panel_sw[SW_TAR_FIX_RIGHT] || panel_sw[SW_TAR_FIX_LEFT])){
      real_mode = ALL_MOVING_DESK_E;
    }/* else{ */
    /*   real_mode = STOP_EVERYTHING; */
    /* } */

    if(panel_sw[SW_POWER_UP] && power_up_flag){
      switch(real_mode){
      case ALL_MOVING_DESK_A:
	desk_a_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_B:
	desk_b_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_C:
	desk_c_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_D_TOP:
	desk_d_top_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_D_BOTTOM:
	desk_d_bottom_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_E:
	desk_e_power += SW_UP_VALUE;
	break;
      default:
	break;
      }
      power_up_flag = false;
    }
    if(panel_sw[SW_POWER_DOWN] && power_down_flag){
      switch(real_mode){
      case ALL_MOVING_DESK_A:
	desk_a_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_B:
	desk_b_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_C:
	desk_c_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_D_TOP:
	desk_d_top_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_D_BOTTOM:
	desk_d_bottom_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_E:
	desk_e_power -= SW_DOWN_VALUE;
	break;
      default:
	break;
      }
      power_down_flag = false;
    }
    
  }

  if(panel_sw[SW_RESET]){
    real_mode = RESET_SHOTMECHA;
    encoder_move_end = false;
    linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,NO_LINE);
    degree_mecha_set_target(0,0,1);
    auto_shot(0, 0, 1);
    DD_encoder1reset();
    DD_encoder2reset();
    real_flag = true;
  }

#if USE_SENSOR_AREA

  if(!real_flag){
    if(panel_sw[SW_SENSOR_AREA]){
      if(panel_sw[SW_ZONE_RED]){
	sensor_area_a = sensor_area_rcv[0] + sensor_area_rcv[1]*256;
	sensor_area_b = sensor_area_rcv[2] + sensor_area_rcv[3]*256;
	sensor_area_c = sensor_area_rcv[4] + sensor_area_rcv[5]*256;
      }else if(panel_sw[SW_ZONE_BLUE]){
	sensor_area_c = sensor_area_rcv[0] + sensor_area_rcv[1]*256;
	sensor_area_b = sensor_area_rcv[2] + sensor_area_rcv[3]*256;
	sensor_area_a = sensor_area_rcv[4] + sensor_area_rcv[5]*256;
      }
    
      distancetowall_desk_a = (int32_t)(((double)sensor_area_a/(double)ENCODER_OMNI_DISTANCE_PER_REVOLUTION)*(double)ENCODER_SMALL_PULSE_PER_REVOLUTION);
      distancetowall_desk_b = (int32_t)(((double)sensor_area_b/(double)ENCODER_OMNI_DISTANCE_PER_REVOLUTION)*(double)ENCODER_SMALL_PULSE_PER_REVOLUTION);
      distancetowall_desk_c = (int32_t)(((double)sensor_area_c/(double)ENCODER_OMNI_DISTANCE_PER_REVOLUTION)*(double)ENCODER_SMALL_PULSE_PER_REVOLUTION);
    }else{
      distancetowall_desk_a = RESET_DESK_A_DISTANCE;
      distancetowall_desk_b = RESET_DESK_B_DISTANCE;
      distancetowall_desk_c = RESET_DESK_C_DISTANCE;
    }
  }
  
  sensor_area_emergency = sensor_area_rcv[6] + sensor_area_rcv[7]*256;
  if(sensor_area_emergency == 26265){
    real_mode = STOP_EVERYTHING;
    real_flag = false;
    all_motor_stop();
  }
#endif
  
  if(real_flag){
    switch(real_mode){
    case ALL_MOVING_DESK_A:
    case ALL_MOVING_DESK_B:
    case ALL_MOVING_DESK_C:
    case ALL_MOVING_DESK_D_TOP:
    case ALL_MOVING_DESK_D_BOTTOM:
    case ALL_MOVING_DESK_E:
      
      if(!move_go_end && !adjust_line && !wait_receive && !shot_end && !move_return_end ){
	if(real_mode == ALL_MOVING_DESK_D_TOP || real_mode == ALL_MOVING_DESK_D_BOTTOM || real_mode == ALL_MOVING_DESK_E){
	  if(shot_state == SHOTMECHA_RESET){
	    if(real_mode == ALL_MOVING_DESK_A){
	      shot_state = auto_shot(desk_a_power, desk_a_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_B){
	      shot_state = auto_shot(desk_b_power, desk_b_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_C){
	      shot_state = auto_shot(desk_c_power, desk_c_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_D_TOP){
	      shot_state = auto_shot(desk_d_top_power, desk_d_top_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_D_BOTTOM){
	      shot_state = auto_shot(desk_d_bottom_power, desk_d_bottom_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_E){
	      shot_state = auto_shot(desk_e_power, desk_e_degree, 0);
	    }
	  }else if((shot_state == PULLING || shot_state == WAIT_PERFECT_RELEASE) && !panel_sw[SW_WAIT_SHOT]){
	    shot_state = shot();
	  }else if(shot_state == COMPLETED_SHOT){
	    auto_shot(0,0,1);
	    //shot_state = SHOTMECHA_RESET;
	    Led_brink(OFF,LED_BRINK_INTERVAL,0);
	    //shot_end = true;
	  }
	}else{
	  if(shot_state == SHOTMECHA_RESET){
	    if(real_mode == ALL_MOVING_DESK_A){
	      shot_state = auto_shot(desk_a_power, desk_a_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_B){
	      shot_state = auto_shot(desk_b_power, desk_b_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_C){
	      shot_state = auto_shot(desk_c_power, desk_c_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_D_TOP){
	      shot_state = auto_shot(desk_d_top_power, desk_d_top_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_D_BOTTOM){
	      shot_state = auto_shot(desk_d_bottom_power, desk_d_bottom_degree, 0);
	    }else if(real_mode == ALL_MOVING_DESK_E){
	      shot_state = auto_shot(desk_e_power, desk_e_degree, 0);
	    }
	  }
	}
	
	if(real_mode == ALL_MOVING_DESK_A){
	  sus_state = go_to_target(MV_FRONT, START_ZONE, distancetowall_desk_a, gamezone, 0);
	}else if(real_mode == ALL_MOVING_DESK_B){
	  sus_state = go_to_target(MV_CENTER, START_ZONE, distancetowall_desk_b, gamezone, 0);
	}else if(real_mode == ALL_MOVING_DESK_C){
	  sus_state = go_to_target(MV_BACK, START_ZONE, distancetowall_desk_c, gamezone, 0);
	}else if(real_mode == ALL_MOVING_DESK_D_TOP || real_mode == ALL_MOVING_DESK_D_BOTTOM){
	  sus_state = go_to_target(FIX_TWO, START_ZONE, distancetowall_desk_d, gamezone, 0);
	}else if(real_mode == ALL_MOVING_DESK_E){
	  sus_state = go_to_target(FIX_RIGHT, START_ZONE, 0, gamezone, 0);
	}
	if(sus_state == MOVE_END){
	  move_go_end = true;
	  count = 0;
	}
      }else if(move_go_end && !adjust_line && !wait_receive && !shot_end && !move_return_end ){
	if(!panel_sw[SW_WAIT_SHOT]){
	  Led_brink(ON,LED_BRINK_INTERVAL,LED_BRINK_DUTY);
	}else{
	  Led_brink(ON,LED_BRINK_INTERVAL-15,LED_BRINK_DUTY-35);
	}
	
	if(shot_state == SHOTMECHA_RESET){
	  if(real_mode == ALL_MOVING_DESK_A){
	    shot_state = auto_shot(desk_a_power, desk_a_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_B){
	    shot_state = auto_shot(desk_b_power, desk_b_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_C){
	    shot_state = auto_shot(desk_c_power, desk_c_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_D_TOP){
	    shot_state = auto_shot(desk_d_top_power, desk_d_top_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_D_BOTTOM){
	    shot_state = auto_shot(desk_d_bottom_power, desk_d_bottom_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_E){
	    shot_state = auto_shot(desk_e_power, desk_e_degree, 0);
	  }
	}else if( (shot_state == PULLING || shot_state == WAIT_PERFECT_RELEASE) && !panel_sw[SW_WAIT_SHOT]){
	  shot_state = shot();
	}else if(shot_state == COMPLETED_SHOT){
	  auto_shot(0,0,1);
	  //shot_state = SHOTMECHA_RESET;
	  Led_brink(OFF,LED_BRINK_INTERVAL,0);
	  //shot_end = true;
	}
	
	movecenterline = move_linecenter(ADJUST_LINECENTER_STRAIGHTDUTY, ADJUST_COUNT);
	if(movecenterline == ADJUST_OK){
	  suspension_stop();
	  adjust_line = true;
	}
      }else if(move_go_end && adjust_line && !wait_receive && !shot_end && !move_return_end){
	if(!panel_sw[SW_WAIT_SHOT]){
	  wait_receive = true;
	}else{
	  Led_brink(ON,LED_BRINK_INTERVAL-15,LED_BRINK_DUTY-35);
	  
	  if(wait_count <= RECEIVE_WAIT_TIME){
	    wait_count++;
	  }else if(wait_count > RECEIVE_WAIT_TIME){
	    Led_brink(OFF,LED_BRINK_INTERVAL,0);
	    wait_count = 0;
	    wait_receive = true;
	  }
	}
      }else if(move_go_end && adjust_line && wait_receive && !shot_end && !move_return_end ){
	Led_brink(ON,LED_BRINK_INTERVAL,LED_BRINK_DUTY);

	if(shot_state == SHOTMECHA_RESET){
	  if(real_mode == ALL_MOVING_DESK_A){
	    shot_state = auto_shot(desk_a_power, desk_a_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_B){
	    shot_state = auto_shot(desk_b_power, desk_b_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_C){
	    shot_state = auto_shot(desk_c_power, desk_c_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_D_TOP){
	    shot_state = auto_shot(desk_d_top_power, desk_d_top_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_D_BOTTOM){
	    shot_state = auto_shot(desk_d_bottom_power, desk_d_bottom_degree, 0);
	  }else if(real_mode == ALL_MOVING_DESK_E){
	    shot_state = auto_shot(desk_e_power, desk_e_degree, 0);
	  }
	}else if(shot_state == PULLING || shot_state == WAIT_PERFECT_RELEASE){
	  shot_state = shot();
	}else if(shot_state == COMPLETED_SHOT){
	  auto_shot(0,0,1);
	  shot_state = SHOTMECHA_RESET;
	  Led_brink(OFF,LED_BRINK_INTERVAL,0);
	  shot_end = true;
	}
      }else if(move_go_end && adjust_line && wait_receive && shot_end && !move_return_end){
	if(pull_rele_mecha != FINISH){
	  pull_rele_mecha = pull_rele_mecha_reset();
	}
	
	if(real_mode == ALL_MOVING_DESK_A){
	  sus_state = go_to_target(START_ZONE, MV_FRONT, distancetowall_desk_a, gamezone, 0);
	}else if(real_mode == ALL_MOVING_DESK_B){
	  sus_state = go_to_target(START_ZONE, MV_CENTER, distancetowall_desk_b, gamezone, 0);
	}else if(real_mode == ALL_MOVING_DESK_C){
	  sus_state = go_to_target(START_ZONE, MV_BACK, distancetowall_desk_c, gamezone, 0);
	}else if(real_mode == ALL_MOVING_DESK_D_TOP || real_mode == ALL_MOVING_DESK_D_BOTTOM){
	  sus_state = go_to_target(START_ZONE, FIX_TWO, distancetowall_desk_d, gamezone, 0);
	}else if(real_mode == ALL_MOVING_DESK_E){
	  sus_state = go_to_target(START_ZONE, FIX_RIGHT, 0, gamezone, 0);
	}
	
	if(sus_state == MOVE_END){
	  move_return_end = true;
	}
      }else if(move_go_end && adjust_line && wait_receive && shot_end && move_return_end){
	wait_count = 0;
	waitposition_set = SET_READY;
	shot_state = SHOTMECHA_RESET;
	pull_rele_mecha = FIRSTRESET;
	move_go_end = false;
	adjust_line = false;
	shot_end = false;
	wait_receive = false;
	move_return_end = false;
	real_flag = false;
	all_motor_stop();
      }
      
      break;

    case RESET_SHOTMECHA:
      if(pullmecha != FINISH){
	pullmecha = pull_mecha_reset();
      }
      if(releasemecha != FINISH){
	releasemecha = release_mecha_reset();
      }
      if(degreemecha != FINISH){
	degreemecha = degree_mecha_reset();
      }
      if(degreemecha == FINISH){
	I2C_Encoder(1,RESET_ENCODER_VALUE);
      }

      if(pullmecha == FINISH && releasemecha == FINISH && degreemecha == FINISH){
	degreemecha = FIRSTRESET;
	pullmecha = FIRSTRESET;
	releasemecha = FIRSTRESET;
	real_flag = false;
      }
      break;
      
    case STOP_EVERYTHING:
      all_motor_stop();
      break;
    }
  }else{
    wait_count = 0;
    degreemecha = FIRSTRESET;
    shot_state = SHOTMECHA_RESET;
    pullmecha = FIRSTRESET;
    releasemecha = FIRSTRESET;
    pull_rele_mecha = FIRSTRESET;
    encoder_move_end = false;
    linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,NO_LINE);
    degree_mecha_set_target(0,0,1);
    auto_shot(0, 0, 1);
    go_to_target(MV_FRONT, START_ZONE, 0, RED, 1);
    DD_encoder1reset();
    DD_encoder2reset();
    move_go_end = false;
    adjust_line = false;
    wait_receive = false;
    shot_end = false;
    move_return_end = false;
    if(!manual_move){
      all_motor_stop();
    }
  }

  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    if(gamezone == RED){
      MW_printf("gamezone [RED ]\n");
    }else{
      MW_printf("gamezone [BLUE]\n");
    }
    MW_printf("realmode       :[%30s]\n",testmode_name[real_mode]);

    if(panel_sw[SW_SENSOR_AREA]){
      MW_printf("sensor_area [YES]\n");
    }else{
      MW_printf("sensor_area [NO ]\n");
    }
    
    if(panel_sw[SW_WAIT_SHOT]){
      MW_printf("receivewait [YES]\n");
    }else{
      MW_printf("receivewait [NO ]\n");
    }
    //MW_printf("testmode_target[0]:[%6d]\ntestmode_target[1]:[%6d]\n",testmode_target[0],testmode_target[1]);
  
    switch(real_mode){
    case ALL_MOVING_DESK_A:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_a_degree,desk_a_power,distancetowall_desk_a);
      break;
    case ALL_MOVING_DESK_B:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_b_degree,desk_b_power,distancetowall_desk_b);
      break;
    case ALL_MOVING_DESK_C:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_c_degree,desk_c_power,distancetowall_desk_c);
      break;
    case ALL_MOVING_DESK_D_TOP:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_d_top_degree,desk_d_top_power,0);
      break;
    case ALL_MOVING_DESK_D_BOTTOM:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_d_bottom_degree,desk_d_bottom_power,0);
      break;
    case ALL_MOVING_DESK_E:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_e_degree,desk_e_power,0);
      break;
    default:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",0,0,0);
      break;
    }
    
    x_encoder = DD_encoder1Get_int32();
    MW_printf("x_encoder:[%7d]\n",x_encoder);
    y_encoder = DD_encoder2Get_int32();
    MW_printf("y_encoder:[%7d]\n",y_encoder);
    releasemecha_encoder = I2C_Encoder(0,GET_ENCODER_VALUE);
    MW_printf("releasemecha_encoder:[%7d]\n",releasemecha_encoder);
    degreemecha_encoder = I2C_Encoder(1,GET_ENCODER_VALUE);
    MW_printf("degreemecha_encoder :[%7d]\n",degreemecha_encoder);
    MW_printf("linetrace : state[%12s] direction[%11s] gap_degree[%2d]\n",linetrace_state_name[linetrace_state],linetrace_direction_name[robot_direction.direction],robot_direction.gap_degree);
    if(_IS_PRESSED_FRONT_FOOTSW()){
      MW_printf("FRONT_FOOTSWITCH [ ON  ] \n");
    }else{
      MW_printf("FRONT_FOOTSWITCH [ OFF ] \n");
    }
    if(_IS_PRESSED_BACK_FOOTSW()){
      MW_printf("BACK_FOOTSWITCH [ ON  ] \n");
    }else{
      MW_printf("BACK_FOOTSWITCH [ OFF ] \n");
    }
  }
  
#endif 

#if USE_CONTROL_PANEL_IN_NOCOLOR
  change_buttonflag(g_ss_h[CONTROL_PANEL_1].data,g_ss_h[CONTROL_PANEL_2].data);

  if(panel_sw[SW_FUN]){
    g_md_h[8].duty = 6000;
    g_md_h[8].mode = D_MMOD_FORWARD;
  }else{
    g_md_h[8].duty = 0;
    g_md_h[8].mode = D_MMOD_FREE;
  }
  
  if(panel_sw[SW_START]){
    real_flag = true;
  }

  if(panel_sw[SW_STOP]){
    real_mode = STOP_EVERYTHING;
    all_motor_stop();
    real_flag = false;
  }
  
  if(panel_sw[SW_MV_FORWARD]){
    spin_direction = 1;
  }else if(panel_sw[SW_MV_BACKWARD]){
    spin_direction = -1;
  }

  if(!real_flag){

    if(panel_sw[SW_ZONE_RED]){
      gamezone = RED;
    }else if(panel_sw[SW_ZONE_BLUE]){
      gamezone = BLUE;
    }

    if(!panel_sw[SW_POWER_UP]){
      power_up_flag = true;
    }
    if(!panel_sw[SW_POWER_DOWN]){
      power_down_flag = true;
    }

    if(panel_sw[SW_LED_BRINK]){
      Led_brink(ON,LED_BRINK_INTERVAL,LED_BRINK_DUTY);
    }else{
      Led_brink(OFF,LED_BRINK_INTERVAL,LED_BRINK_DUTY);
    }
    
    if(panel_sw[SW_MV_1]){
      trapezoidCtrl(PULLMECHA_MAX_DUTY*spin_direction, &g_md_h[PULLMECHA], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[PULLMECHA], &realmode_tcon);
    }

    if(panel_sw[SW_MV_2]){
      trapezoidCtrl(RELEASEMECHA_MAX_DUTY*spin_direction, &g_md_h[RELEASEMECHA], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[RELEASEMECHA], &realmode_tcon);
    }

    if(panel_sw[SW_MV_3]){
      trapezoidCtrl(DEGREEMECHA_MAX_DUTY*spin_direction, &g_md_h[DEGREEMECHA], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[DEGREEMECHA], &realmode_tcon);
    }

    if(panel_sw[SW_MV_4]){
      trapezoidCtrl(REALMODE_SUS_MAXDUTY*spin_direction, &g_md_h[KUDO_LF], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[KUDO_LF], &realmode_tcon);
    }

    if(panel_sw[SW_MV_5]){
      trapezoidCtrl(REALMODE_SUS_MAXDUTY*spin_direction, &g_md_h[KUDO_LB], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[KUDO_LB], &realmode_tcon);
    }

    if(panel_sw[SW_MV_6]){
      trapezoidCtrl(REALMODE_SUS_MAXDUTY*spin_direction, &g_md_h[KUDO_RB], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[KUDO_RB], &realmode_tcon);
    }

    if(panel_sw[SW_MV_7]){
      trapezoidCtrl(REALMODE_SUS_MAXDUTY*spin_direction, &g_md_h[KUDO_RF], &realmode_tcon);
    }else{
      trapezoidCtrl(0, &g_md_h[KUDO_RF], &realmode_tcon);
    }

    if(panel_sw[SW_MV_1] || panel_sw[SW_MV_2] || panel_sw[SW_MV_3] || panel_sw[SW_MV_4] || panel_sw[SW_MV_5] || panel_sw[SW_MV_6] || panel_sw[SW_MV_7]){
      real_mode = NO_OPERATION;
      manual_move = true;
    }else{
      manual_move = false;
    }    
  
    if(panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_A;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_B;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_C;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_D_TOP;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && panel_sw[SW_TAR_FIX_BOTTOM] && !panel_sw[SW_TAR_FIX_RIGHT] && !panel_sw[SW_TAR_FIX_LEFT]){
      real_mode = ALL_MOVING_DESK_D_BOTTOM;
    }else if(!panel_sw[SW_TAR_MV_FRONT] && !panel_sw[SW_TAR_MV_CENTER] && !panel_sw[SW_TAR_MV_BACK] && !panel_sw[SW_TAR_FIX_TOP] && !panel_sw[SW_TAR_FIX_BOTTOM] && (panel_sw[SW_TAR_FIX_RIGHT] || panel_sw[SW_TAR_FIX_LEFT])){
      real_mode = ALL_MOVING_DESK_E;
    }/* else{ */
    /*   real_mode = STOP_EVERYTHING; */
    /* } */

    if(panel_sw[SW_POWER_UP] && power_up_flag){
      switch(real_mode){
      case ALL_MOVING_DESK_A:
	desk_a_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_B:
	desk_b_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_C:
	desk_c_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_D_TOP:
	desk_d_top_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_D_BOTTOM:
	desk_d_bottom_power += SW_UP_VALUE;
	break;
      case ALL_MOVING_DESK_E:
	desk_e_power += SW_UP_VALUE;
	break;
      default:
	break;
      }
      power_up_flag = false;
    }
    if(panel_sw[SW_POWER_DOWN] && power_down_flag){
      switch(real_mode){
      case ALL_MOVING_DESK_A:
	desk_a_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_B:
	desk_b_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_C:
	desk_c_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_D_TOP:
	desk_d_top_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_D_BOTTOM:
	desk_d_bottom_power -= SW_DOWN_VALUE;
	break;
      case ALL_MOVING_DESK_E:
	desk_e_power -= SW_DOWN_VALUE;
	break;
      default:
	break;
      }
      power_down_flag = false;
    }
    
  }

  if(panel_sw[SW_RESET]){
    real_mode = RESET_SHOTMECHA;
    encoder_move_end = false;
    linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,NO_LINE);
    degree_mecha_set_target(0,0,1);
    auto_shot(0, 0, 1);
    DD_encoder1reset();
    DD_encoder2reset();
    real_flag = true;
  }

  if(real_flag){
    switch(real_mode){
    case ALL_MOVING_DESK_A:
      shot_state = auto_shot(desk_a_power, desk_a_degree, 0);
      if(shot_state == COMPLETED_SHOT){
	shot_state = SHOTMECHA_RESET;
	real_flag = false;
      }
      break;
      
    case ALL_MOVING_DESK_B:
      shot_state = auto_shot(desk_b_power, desk_b_degree, 0);
      if(shot_state == COMPLETED_SHOT){
	shot_state = SHOTMECHA_RESET;
	real_flag = false;
      }
      break;
      
    case ALL_MOVING_DESK_C:
      shot_state = auto_shot(desk_c_power, desk_c_degree, 0);
      if(shot_state == COMPLETED_SHOT){
	shot_state = SHOTMECHA_RESET;
	real_flag = false;
      }
      break;
      
    case ALL_MOVING_DESK_D_TOP:
      shot_state = auto_shot(desk_d_top_power, desk_d_top_degree, 0);
      if(shot_state == COMPLETED_SHOT){
	shot_state = SHOTMECHA_RESET;
	real_flag = false;
      }
      break;
      
    case ALL_MOVING_DESK_D_BOTTOM:
      shot_state = auto_shot(desk_d_bottom_power, desk_d_bottom_degree, 0);
      if(shot_state == COMPLETED_SHOT){
	shot_state = SHOTMECHA_RESET;
	real_flag = false;
      }
      break;
      
    case ALL_MOVING_DESK_E:
      shot_state = auto_shot(desk_e_power, desk_e_degree, 0);
      if(shot_state == COMPLETED_SHOT){
	shot_state = SHOTMECHA_RESET;
	real_flag = false;
      }
      break;
      
    case RESET_SHOTMECHA:
      if(pullmecha != FINISH){
	pullmecha = pull_mecha_reset();
      }
      if(releasemecha != FINISH){
	releasemecha = release_mecha_reset();
      }
      if(degreemecha != FINISH){
	degreemecha = degree_mecha_reset();
      }
      if(degreemecha == FINISH){
	I2C_Encoder(1,RESET_ENCODER_VALUE);
      }

      if(pullmecha == FINISH && releasemecha == FINISH && degreemecha == FINISH){
	degreemecha = FIRSTRESET;
	pullmecha = FIRSTRESET;
	releasemecha = FIRSTRESET;
	real_flag = false;
      }
      break;
    case STOP_EVERYTHING:
      all_motor_stop();
      break;
    }
  }else{
    wait_count = 0;
    degreemecha = FIRSTRESET;
    shot_state = SHOTMECHA_RESET;
    pullmecha = FIRSTRESET;
    releasemecha = FIRSTRESET;
    pull_rele_mecha = FIRSTRESET;
    encoder_move_end = false;
    linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,NO_LINE);
    degree_mecha_set_target(0,0,1);
    auto_shot(0, 0, 1);
    go_to_target(MV_FRONT, START_ZONE, 0, RED, 1);
    DD_encoder1reset();
    DD_encoder2reset();
    move_go_end = false;
    adjust_line = false;
    wait_receive = false;
    shot_end = false;
    move_return_end = false;
    if(!manual_move){
      all_motor_stop();
    }
  }

  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    if(gamezone == RED){
      MW_printf("gamezone [RED ]\n");
    }else{
      MW_printf("gamezone [BLUE]\n");
    }
    MW_printf("realmode       :[%30s]\n",testmode_name[real_mode]);

    if(panel_sw[SW_SENSOR_AREA]){
      MW_printf("sensor_area [YES]\n");
    }else{
      MW_printf("sensor_area [NO ]\n");
    }
    
    if(panel_sw[SW_WAIT_SHOT]){
      MW_printf("receivewait [YES]\n");
    }else{
      MW_printf("receivewait [NO ]\n");
    }
    //MW_printf("testmode_target[0]:[%6d]\ntestmode_target[1]:[%6d]\n",testmode_target[0],testmode_target[1]);
  
    switch(real_mode){
    case ALL_MOVING_DESK_A:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_a_degree,desk_a_power,distancetowall_desk_a);
      break;
    case ALL_MOVING_DESK_B:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_b_degree,desk_b_power,distancetowall_desk_b);
      break;
    case ALL_MOVING_DESK_C:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_c_degree,desk_c_power,distancetowall_desk_c);
      break;
    case ALL_MOVING_DESK_D_TOP:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_d_top_degree,desk_d_top_power,0);
      break;
    case ALL_MOVING_DESK_D_BOTTOM:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_d_bottom_degree,desk_d_bottom_power,0);
      break;
    case ALL_MOVING_DESK_E:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",desk_e_degree,desk_e_power,0);
      break;
    default:
      MW_printf("target : degree[%6d] power[%4d] distance[%5d]\n",0,0,0);
      break;
    }
    
    x_encoder = DD_encoder1Get_int32();
    MW_printf("x_encoder:[%7d]\n",x_encoder);
    y_encoder = DD_encoder2Get_int32();
    MW_printf("y_encoder:[%7d]\n",y_encoder);
    releasemecha_encoder = I2C_Encoder(0,GET_ENCODER_VALUE);
    MW_printf("releasemecha_encoder:[%7d]\n",releasemecha_encoder);
    degreemecha_encoder = I2C_Encoder(1,GET_ENCODER_VALUE);
    MW_printf("degreemecha_encoder :[%7d]\n",degreemecha_encoder);
    MW_printf("linetrace : state[%12s] direction[%11s] gap_degree[%2d]\n",linetrace_state_name[linetrace_state],linetrace_direction_name[robot_direction.direction],robot_direction.gap_degree);
    if(_IS_PRESSED_FRONT_FOOTSW()){
      MW_printf("FRONT_FOOTSWITCH [ ON  ] \n");
    }else{
      MW_printf("FRONT_FOOTSWITCH [ OFF ] \n");
    }
    if(_IS_PRESSED_BACK_FOOTSW()){
      MW_printf("BACK_FOOTSWITCH [ ON  ] \n");
    }else{
      MW_printf("BACK_FOOTSWITCH [ OFF ] \n");
    }
  }
#endif
  
#if DD_USE_RC
  
  if(!__RC_ISPRESSED_CIRCLE(g_rc_data)){
    circle_flag = true;
  }
  if(!__RC_ISPRESSED_CROSS(g_rc_data)){
    cross_flag = true;
  }
  if(!__RC_ISPRESSED_SQARE(g_rc_data)){
    sqare_flag = true;
  }
  if(!__RC_ISPRESSED_TRIANGLE(g_rc_data)){
    triangle_flag = true;
  }
  if(!__RC_ISPRESSED_UP(g_rc_data)){
    up_flag = true;
  }
  if(!__RC_ISPRESSED_DOWN(g_rc_data)){
    down_flag = true;
  }
  if(!__RC_ISPRESSED_RIGHT(g_rc_data)){
    right_flag = true;
  }
  if(!__RC_ISPRESSED_LEFT(g_rc_data)){
    left_flag = true;
  }
  if(!__RC_ISPRESSED_R1(g_rc_data)){
    r1_flag = true;
  }
  if(!__RC_ISPRESSED_R2(g_rc_data)){
    r2_flag = true;
  }
  if(!__RC_ISPRESSED_L1(g_rc_data)){
    l1_flag = true;
  }
  if(!__RC_ISPRESSED_L2(g_rc_data)){
    l2_flag = true;
  }

  if(__RC_ISPRESSED_RIGHT(g_rc_data) && right_flag){
    if(!test_flag){
      if(test_mode == STOP_EVERYTHING){
	test_mode = MOVE_PULLMECHA;
      }else{
	test_mode++;
      }
    }
    right_flag = false;
  }
  if(__RC_ISPRESSED_LEFT(g_rc_data) && left_flag){
    if(!test_flag){
      if(test_mode == MOVE_PULLMECHA){
	test_mode = STOP_EVERYTHING;
      }else{
	test_mode--;
      }
    }
    left_flag = false;
  }

  if(__RC_ISPRESSED_UP(g_rc_data) && up_flag){
    testmode_target[index] += bairitu*5;
    up_flag = false;
  }

  if(__RC_ISPRESSED_DOWN(g_rc_data) && down_flag){
    testmode_target[index] -= bairitu*5;
    down_flag = false;
  }

  if(__RC_ISPRESSED_CIRCLE(g_rc_data) && circle_flag){
    test_flag = true;
    circle_flag = false;
  }

  if(__RC_ISPRESSED_CROSS(g_rc_data) && circle_flag){
    test_flag = false;
    all_motor_stop();
    cross_flag = false;
  }
  
  if(__RC_ISPRESSED_SQARE(g_rc_data)){
    /* MW_SetGPIOPin(GPIO_PIN_4); */
    /* MW_SetGPIOMode(GPIO_MODE_OUTPUT_PP); */
    /* MW_SetGPIOSpeed(GPIO_SPEED_FREQ_LOW); */
    /* MW_GPIOInit(GPIOCID); */
    MW_GPIOWrite(GPIOCID,GPIO_PIN_4,GPIO_PIN_SET);
    sqare_flag = false;
  }

  if(!__RC_ISPRESSED_SQARE(g_rc_data)){
    MW_GPIOWrite(GPIOCID,GPIO_PIN_4,GPIO_PIN_RESET);
  }

  if(__RC_ISPRESSED_TRIANGLE(g_rc_data) && circle_flag){
    testmode_target[0] = 0;
    testmode_target[1] = 0;
    triangle_flag = false;
  }

  if(__RC_ISPRESSED_R1(g_rc_data) && r1_flag){
    bairitu = bairitu*10;
    r1_flag = false;
  }

  if(__RC_ISPRESSED_R2(g_rc_data) && r2_flag){
    if(bairitu == 1){
      bairitu = 1;
    }else{
      bairitu = bairitu / 10;
    }
    r2_flag = false;
  }

  if(__RC_ISPRESSED_L1(g_rc_data) && l1_flag){
    l1_turn_flag = !l1_turn_flag;
    if(l1_turn_flag){
      g_md_h[8].duty = 6000;
      g_md_h[8].mode = D_MMOD_FORWARD;
    }else{
      g_md_h[8].duty = 0;
      g_md_h[8].mode = D_MMOD_FREE;
    }
  }

  if(__RC_ISPRESSED_L2(g_rc_data) && l2_flag){
    l2_turn_flag = !l2_turn_flag;
    if(!l2_turn_flag){
      index = 0;
    }else{
      index = 1;
    }
  }
  
  if(test_flag){
    switch(test_mode){
      
    case MOVE_PULLMECHA:
      trapezoidCtrl(-DD_RCGetLX(g_rc_data)*(TESTMODE_MAX_DUTY/RC_MAX_VALUE), &g_md_h[PULLMECHA], &testmode_tcon);
      break;
      
    case MOVE_RELEASEMECHA:
      trapezoidCtrl(-DD_RCGetLX(g_rc_data)*(TESTMODE_MAX_DUTY/RC_MAX_VALUE), &g_md_h[RELEASEMECHA], &testmode_tcon);
      break;
      
    case MOVE_DEGREEMECHA:
      trapezoidCtrl(-DD_RCGetLX(g_rc_data)*(TESTMODE_MAX_DUTY/RC_MAX_VALUE), &g_md_h[DEGREEMECHA], &testmode_tcon);
      break;

    case RESET_SHOTMECHA:
      if(pullmecha != FINISH){
	pullmecha = pull_mecha_reset();
      }
      if(releasemecha != FINISH){
	releasemecha = release_mecha_reset();
      }
      if(degreemecha != FINISH){
	degreemecha = degree_mecha_reset();
      }
      if(degreemecha == FINISH){
	I2C_Encoder(1,RESET_ENCODER_VALUE);
      }

      if(pullmecha == FINISH && releasemecha == FINISH && degreemecha == FINISH){
	degreemecha = FIRSTRESET;
	pullmecha = FIRSTRESET;
  	releasemecha = FIRSTRESET;
	test_flag = false;
      }
      break;
      
    case RESET_DEGREEMECHA:
      if(degreemecha != FINISH){
	degreemecha = degree_mecha_reset();
      }
      if(degreemecha == FINISH){
	I2C_Encoder(1,RESET_ENCODER_VALUE);
	degreemecha = FIRSTRESET;
	test_flag = false;
      }
      break;

    case SET_DEGREEMECHA:
      degreemecha_encoder = I2C_Encoder(1,GET_ENCODER_VALUE);
      degreemecha_set = degree_mecha_set_target(testmode_target[0],degreemecha_encoder,0);
      if(degreemecha_set == SET_OK){
	test_flag = false;
      }
      break;
      
    case AUTO_SHOT:
      shot_state = auto_shot(testmode_target[0], testmode_target[1], 0);
      if(shot_state == COMPLETED_SHOT){
	test_flag = false;
      }
      break;
      
    case MANUAL_SUSPENSION:
      ret = manual_suspensionsystem();
      if(ret){
	return ret;
      }
      break;
      
    case AUTO_SUS_FRONT:
      y_encoder = DD_encoder2Get_int32();
      
      linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,NO_LINE);
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;

      if(linetrace_duty <= 15){
	linetrace_duty = 2;
      }else if(linetrace_duty >= 17){
	linetrace_duty = 15;
      }

      switch(robot_direction.direction){
      case D_RIGHT:
	sus_state = cal_omni_value(FRONT, MOVE_STOP, testmode_target[0], y_encoder, linetrace_duty, 0, 0);
	break;
      case D_LEFT:
	sus_state = cal_omni_value(FRONT, MOVE_STOP, testmode_target[0], y_encoder, -linetrace_duty, 0, 0);
	break;
      case D_FRONT_RIGHT:
	sus_state = cal_omni_value(FRONT, MOVE_STOP, testmode_target[0], y_encoder, 0, 0, -linetrace_duty);
	break;
      case D_FRONT_LEFT:
	sus_state = cal_omni_value(FRONT, MOVE_STOP, testmode_target[0], y_encoder, 0, 0, linetrace_duty);
	break;
      case D_MIDDLE:
	sus_state = cal_omni_value(FRONT, MOVE_STOP, testmode_target[0], y_encoder, 0, 0, 0);
	break;
      case NO_LINE:
	sus_state = cal_omni_value(FRONT, MOVE_STOP, testmode_target[0], y_encoder, 0, 0, 0);
	break;
      }
      
      if(sus_state == MOVE_END){
	test_flag = false;
      }
      break;
      
    case AUTO_SUS_BACK:
      y_encoder = DD_encoder2Get_int32();
      
      linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,NO_LINE);
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;

      if(linetrace_duty <= 15){
	linetrace_duty = 2;
      }else if(linetrace_duty >= 17){
	linetrace_duty = 15;
      }

      switch(robot_direction.direction){
      case D_RIGHT:
	sus_state = cal_omni_value(BACK, MOVE_STOP, testmode_target[0], y_encoder, linetrace_duty, 0, 0);
	break;
      case D_LEFT:
	sus_state = cal_omni_value(BACK, MOVE_STOP, testmode_target[0], y_encoder, -linetrace_duty, 0, 0);
	break;
      case D_FRONT_RIGHT:
	sus_state = cal_omni_value(BACK, MOVE_STOP, testmode_target[0], y_encoder, 0, 0, -linetrace_duty);
	break;
      case D_FRONT_LEFT:
	sus_state = cal_omni_value(BACK, MOVE_STOP, testmode_target[0], y_encoder, 0, 0, linetrace_duty);
	break;
      case D_MIDDLE:
	sus_state = cal_omni_value(BACK, MOVE_STOP, testmode_target[0], y_encoder, 0, 0, 0);
	break;
      case NO_LINE:
	sus_state = cal_omni_value(BACK, MOVE_STOP, testmode_target[0], y_encoder, 0, 0, 0);
	break;
      }
      
      if(sus_state == MOVE_END){
	test_flag = false;
      }
      break;
      
    case AUTO_SUS_LEFT:
      x_encoder = DD_encoder1Get_int32(); 
      sus_state = cal_omni_value(LEFT, MOVE_STOP, testmode_target[0], x_encoder, 0, 0, 0);
      if(sus_state == MOVE_END){
	test_flag = false;
      }
      break;
      
    case AUTO_SUS_RIGHT:
      x_encoder = DD_encoder1Get_int32(); 
      sus_state = cal_omni_value(RIGHT, MOVE_STOP, testmode_target[0], x_encoder, 0, 0, 0);
      if(sus_state == MOVE_END){
	test_flag = false;
      }
      break;
      
    case MOVE_SUS_LF:
      trapezoidCtrl(-DD_RCGetLX(g_rc_data)*(TESTMODE_MAX_DUTY/RC_MAX_VALUE), &g_md_h[KUDO_LF], &testmode_tcon);
      break;
      
    case MOVE_SUS_LB:
      trapezoidCtrl(-DD_RCGetLX(g_rc_data)*(TESTMODE_MAX_DUTY/RC_MAX_VALUE), &g_md_h[KUDO_LB], &testmode_tcon);
      break;
      
    case MOVE_SUS_RB:
      trapezoidCtrl(-DD_RCGetLX(g_rc_data)*(TESTMODE_MAX_DUTY/RC_MAX_VALUE), &g_md_h[KUDO_RB], &testmode_tcon);
      break;
      
    case MOVE_SUS_RF:
      trapezoidCtrl(-DD_RCGetLX(g_rc_data)*(TESTMODE_MAX_DUTY/RC_MAX_VALUE), &g_md_h[KUDO_RF], &testmode_tcon);
      break;
      
    case TRACE_LINE:
      linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,NO_LINE);
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;
      linetrace_duty = linetrace_duty_decide(linetrace_state,linetrace_duty);
      
      switch(robot_direction.direction){
      case D_RIGHT:
	cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, linetrace_duty, linetrace_strate_duty, 0);
	break;
      case D_LEFT:
	cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -linetrace_duty, linetrace_strate_duty, 0);
	break;
      case D_FRONT_RIGHT:
	cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, linetrace_strate_duty, -linetrace_duty);
	break;
      case D_FRONT_LEFT:
	cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, linetrace_strate_duty, linetrace_duty);
	break;
      case D_MIDDLE:
	cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, linetrace_strate_duty, 0);
	break;
      case NO_LINE:
	cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, linetrace_strate_duty, 0);
	break;
      }
      break;

    case TRACE_UNTIL_FRONT_TOUCH:
      y_encoder = DD_encoder2Get_int32();
      
      linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,NO_LINE);
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;

      if(linetrace_state == CURRENT_DATA){
	if(linetrace_duty <= 16){
	  linetrace_duty = 1;
	}else if(linetrace_duty >= 30){
	  linetrace_duty = 20;
	}else if(linetrace_duty >= 17){
	  linetrace_duty = 14;
	}
      }else if(RECENT_DATA){
	if(linetrace_duty <= 11){
	  linetrace_duty = 5;
	}else if(linetrace_duty >= 12){
	  linetrace_duty = 15;
	}
      }
      
      /* if(linetrace_duty <= 15){ */
      /* 	linetrace_duty = 2; */
      /* }else if(linetrace_duty >= 16){ */
      /* 	linetrace_duty = 15; */
      /* } */

      if(_IS_PRESSED_FRONT_FOOTSW()){
	all_motor_stop();
	encoder_move_end = false;
	test_flag = false;
      }else{

	if(!encoder_move_end){
	  switch(robot_direction.direction){
	  case D_RIGHT:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, testmode_target[0], y_encoder, linetrace_duty, 0, 0);
	    break;
	  case D_LEFT:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, testmode_target[0], y_encoder, -linetrace_duty, 0, 0);
	    break;
	  case D_FRONT_RIGHT:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, testmode_target[0], y_encoder, 0, 0, -linetrace_duty);
	    break;
	  case D_FRONT_LEFT:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, testmode_target[0], y_encoder, 0, 0, linetrace_duty);
	    break;
	  case D_MIDDLE:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, testmode_target[0], y_encoder, 0, 0, 0);
	    break;
	  case NO_LINE:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, testmode_target[0], y_encoder, 0, 0, 0);
	    break;
	  }
	  if(sus_state == MOVE_END){
	    encoder_move_end = true;
	  }
	  
	}else{
	  
	  switch(robot_direction.direction){
	  case D_RIGHT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, linetrace_duty, MOVEFRONT_MIN_DUTY, 0);
	    break;
	  case D_LEFT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -linetrace_duty, MOVEFRONT_MIN_DUTY, 0);
	    break;
	  case D_FRONT_RIGHT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, -linetrace_duty);
	    break;
	  case D_FRONT_LEFT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, linetrace_duty);
	    break;
	  case D_MIDDLE:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, 0);
	    break;
	  case NO_LINE:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, 0);
	    break;
	  }
	  
	}
      }
      break;

    case MOVE_LEFT_LINE_C:
      x_encoder = DD_encoder1Get_int32();
      linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,D_RIGHT);
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;

      if(linetrace_state == CURRENT_DATA){
	if(linetrace_duty <= 16){
	  linetrace_duty = 1;
	}else if(linetrace_duty >= 30){
	  linetrace_duty = 20;
	}else if(linetrace_duty >= 17){
	  linetrace_duty = 14;
	}
      }else if(RECENT_DATA){
	if(linetrace_duty <= 11){
	  linetrace_duty = 5;
	}else if(linetrace_duty >= 12){
	  linetrace_duty = 15;
	}
      }
      
      /* if(linetrace_duty <= 15){ */
      /* 	linetrace_duty = 2; */
      /* }else if(linetrace_duty >= 16){ */
      /* 	linetrace_duty = 15; */
      /* } */
      
      if(robot_direction.direction == NO_LINE && !encoder_move_end){
	sus_state = cal_omni_value(LEFT, MOVE_CONTINUE, -MOVEGO_LINE_D_VALUE, x_encoder, 0, 0, 0);
	if(sus_state == MOVE_END){
	  encoder_move_end = true;
	}
      }else if(robot_direction.direction == NO_LINE && encoder_move_end){
	sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVELEFT_MIN_DUTY, 0, 0);
      }else if(robot_direction.direction != NO_LINE){
	switch(robot_direction.direction){
	case D_RIGHT:
	  cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, linetrace_duty, 0, 0);
	  break;
	case D_LEFT:
	  cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -linetrace_duty, 0, 0);
	  break;
	case D_FRONT_RIGHT:
	  cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, 0, -linetrace_duty);
	  break;
	case D_FRONT_LEFT:
	  cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, 0, linetrace_duty);
	  break;
	case D_MIDDLE:
	  cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, 0, 0);
	  break;
	case NO_LINE:
	  cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, 0, 0);
	  break;
	}
	
	if(recent_robot_direction.direction == robot_direction.direction && recent_robot_direction.gap_degree == robot_direction.gap_degree){
	  stop_robotturn_count++;
	}else {
	  stop_robotturn_count = 0;
	}
	
	recent_robot_direction.direction = robot_direction.direction;
	recent_robot_direction.gap_degree = robot_direction.gap_degree;

	if(stop_robotturn_count >= 75){
	  recent_robot_direction.direction = NO_LINE;
	  recent_robot_direction.gap_degree = 0;
	  stop_robotturn_count = 0;
	  encoder_move_end = false;
	  test_flag = false;  
	}
      }
      
      break;

    case ALL_MOVING_DESK_A:
    case ALL_MOVING_DESK_B:
    case ALL_MOVING_DESK_C:
    case ALL_MOVING_DESK_D_TOP:
    case ALL_MOVING_DESK_D_BOTTOM:
       
      if(!move_go_end && !adjust_line && !shot_end && !move_return_end ){
	if(shot_state == SHOTMECHA_RESET){
	  if(test_mode == ALL_MOVING_DESK_A){
	    shot_state = auto_shot(desk_a_power, desk_a_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_B){
	    shot_state = auto_shot(desk_b_power, desk_b_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_C){
	    shot_state = auto_shot(desk_c_power, desk_c_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_D_TOP){
	    shot_state = auto_shot(desk_d_top_power, desk_d_top_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_D_BOTTOM){
	    shot_state = auto_shot(desk_d_bottom_power, desk_d_bottom_degree, 0);
	  }
	}
	
	if(test_mode == ALL_MOVING_DESK_A){
	  sus_state = go_to_target(MV_FRONT, START_ZONE, distancetowall_desk_a, zone, 0);
	}else if(test_mode == ALL_MOVING_DESK_B){
	  sus_state = go_to_target(MV_CENTER, START_ZONE, distancetowall_desk_b, zone, 0);
	}else if(test_mode == ALL_MOVING_DESK_C){
	  sus_state = go_to_target(MV_BACK, START_ZONE, distancetowall_desk_c, zone, 0);
	}else if(test_mode == ALL_MOVING_DESK_D_TOP || test_mode == ALL_MOVING_DESK_D_BOTTOM){
	  sus_state = go_to_target(FIX_TWO, START_ZONE, distancetowall_desk_d, zone, 0);
	}
	if(sus_state == MOVE_END){
	  move_go_end = true;
	  count = 0;
	}
      }else if(move_go_end && !adjust_line && !shot_end && !move_return_end ){
	Led_brink(ON,LED_BRINK_INTERVAL,LED_BRINK_DUTY);

	if(shot_state == SHOTMECHA_RESET){
	  if(test_mode == ALL_MOVING_DESK_A){
	    shot_state = auto_shot(desk_a_power, desk_a_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_B){
	    shot_state = auto_shot(desk_b_power, desk_b_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_C){
	    shot_state = auto_shot(desk_c_power, desk_c_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_D_TOP){
	    shot_state = auto_shot(desk_d_top_power, desk_d_top_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_D_BOTTOM){
	    shot_state = auto_shot(desk_d_bottom_power, desk_d_bottom_degree, 0);
	  }
	}
	
	movecenterline = move_linecenter(ADJUST_LINECENTER_STRAIGHTDUTY, ADJUST_COUNT);
	if(movecenterline == ADJUST_OK){
	  adjust_line = true;
	  suspension_stop();
	}
      }else if(move_go_end && adjust_line && !shot_end && !move_return_end ){
	Led_brink(ON,LED_BRINK_INTERVAL,LED_BRINK_DUTY);
	
	if(shot_state == SHOTMECHA_RESET){
	  if(test_mode == ALL_MOVING_DESK_A){
	    shot_state = auto_shot(desk_a_power, desk_a_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_B){
	    shot_state = auto_shot(desk_b_power, desk_b_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_C){
	    shot_state = auto_shot(desk_c_power, desk_c_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_D_TOP){
	    shot_state = auto_shot(desk_d_top_power, desk_d_top_degree, 0);
	  }else if(test_mode == ALL_MOVING_DESK_D_BOTTOM){
	    shot_state = auto_shot(desk_d_bottom_power, desk_d_bottom_degree, 0);
	  }
	}else if(shot_state == PULLING || shot_state == WAIT_PERFECT_RELEASE){
	  shot_state = shot();
	}else if(shot_state == COMPLETED_SHOT){
	  Led_brink(OFF,LED_BRINK_INTERVAL,0);
	  shot_end = true;
	}
      }else if(move_go_end && adjust_line && shot_end && !move_return_end){
	if(pull_rele_mecha != FINISH){
	  pull_rele_mecha = pull_rele_mecha_reset();
	}
	
	if(test_mode == ALL_MOVING_DESK_A){
	  sus_state = go_to_target(START_ZONE, MV_FRONT, distancetowall_desk_a, zone, 0);
	}else if(test_mode == ALL_MOVING_DESK_B){
	  sus_state = go_to_target(START_ZONE, MV_CENTER, distancetowall_desk_b, zone, 0);
	}else if(test_mode == ALL_MOVING_DESK_C){
	  sus_state = go_to_target(START_ZONE, MV_BACK, distancetowall_desk_c, zone, 0);
	}else if(test_mode == ALL_MOVING_DESK_D_TOP || test_mode == ALL_MOVING_DESK_D_BOTTOM){
	  sus_state = go_to_target(START_ZONE, FIX_TWO, distancetowall_desk_d, zone, 0);
	}
	
	if(sus_state == MOVE_END){
	  move_return_end = true;
	}
      }else if(move_go_end && adjust_line && shot_end && move_return_end){
	shot_state == SHOTMECHA_RESET;
	move_go_end = false;
	adjust_line = false;
	shot_end = false;
	move_return_end = false;
	test_flag = false;
	all_motor_stop();
      }
      
      break;
  
    case STOP_EVERYTHING:
      all_motor_stop();
      break;
    } 
  }else{
    degreemecha = FIRSTRESET;
    pullmecha = FIRSTRESET;
    releasemecha = FIRSTRESET;
    encoder_move_end = false;
    linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,NO_LINE);
    degree_mecha_set_target(0,0,1);
    release_mecha_set_target(0,0,1);
    auto_shot(0, 0, 1);
    go_to_target(MV_FRONT, START_ZONE, 0, RED, 1);
    DD_encoder1reset();
    DD_encoder2reset();
    all_motor_stop();
    move_go_end = false;
    adjust_line = false;
    shot_end = false;
    move_return_end = false;
    shot_state = SHOTMECHA_RESET;
  }

  if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
    MW_printf("testmode       :[%21s]\n",testmode_name[test_mode]);
    MW_printf("testmode_target[0]:[%6d]\ntestmode_target[1]:[%6d]\n",testmode_target[0],testmode_target[1]);
    x_encoder = DD_encoder1Get_int32();
    MW_printf("x_encoder:[%7d]\n",x_encoder);
    y_encoder = DD_encoder2Get_int32();
    MW_printf("y_encoder:[%7d]\n",y_encoder);
    releasemecha_encoder = I2C_Encoder(0,GET_ENCODER_VALUE);
    MW_printf("releasemecha_encoder:[%7d]\n",releasemecha_encoder);
    degreemecha_encoder = I2C_Encoder(1,GET_ENCODER_VALUE);
    MW_printf("degreemecha_encoder :[%7d]\n",degreemecha_encoder);
    MW_printf("linetrace : state[%12s] direction[%11s] gap_degree[%2d]\n",linetrace_state_name[linetrace_state],linetrace_direction_name[robot_direction.direction],robot_direction.gap_degree);
    if(_IS_PRESSED_FRONT_FOOTSW()){
      MW_printf("FRONT_FOOTSWITCH [ ON  ] \n");
    }else{
      MW_printf("FRONT_FOOTSWITCH [ OFF ] \n");
    }
    if(_IS_PRESSED_BACK_FOOTSW()){
      MW_printf("BACK_FOOTSWITCH [ ON  ] \n");
    }else{
      MW_printf("BACK_FOOTSWITCH [ OFF ] \n");
    }
  }

#endif
  
  /*それぞれの機構ごとに処理をする*/
  /*途中必ず定数回で終了すること。*/

  return EXIT_SUCCESS;
}

static
int change_buttonflag(uint8_t *panel1, uint8_t *panel2){
  static int panel_flag[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  static uint32_t panel_decimal1 = 0;
  static uint32_t panel_decimal2 = 0;
  unsigned int bit;
  int i;

  panel_decimal1 = (uint16_t)((*panel1) << 8) + (uint8_t)(*(panel1+1));
  panel_decimal2 = (uint16_t)((*panel2) << 8) + (uint8_t)(*(panel2+1));

  bit = (1 << (2*8-1));
  i=15;
  for(;bit != 0 ; bit>>=1){
    if(panel_decimal1 & bit){
      *(panel_flag + i) = 1;
    }else{
      *(panel_flag + i) = 0;
    }
    i--;
  }

  bit = (1 << (2*8-1));
  i=31;
  for(;bit != 0 ; bit>>=1){
    if(panel_decimal2 & bit){
      *(panel_flag + i) = 1;
    }else{
      *(panel_flag + i) = 0;
    }
    i--;
  }
  
  for(i=0;i<32;i++){
    if( i!=20 && i!=21 && i!=22 && i!=23 && i!=24 && i!=25){
      if(panel_flag[i] == 1){
	panel_sw[i] = true;
      }else{
	panel_sw[i] = false;
      }
    }
  }

  if(panel_sw[SW_ZONE]){
    panel_sw[SW_ZONE_RED] = true;
    panel_sw[SW_ZONE_BLUE] = false;
  }else{
    panel_sw[SW_ZONE_RED] = false;
    panel_sw[SW_ZONE_BLUE] = true;
  }

  if(panel_sw[SW_DIRECTION]){
    panel_sw[SW_MV_FORWARD] = true;
    panel_sw[SW_MV_BACKWARD] = false;
  }else{
    panel_sw[SW_MV_FORWARD] = false;
    panel_sw[SW_MV_BACKWARD] = true;
  }
  
  return EXIT_SUCCESS;
}


//注意:台形制御(trapezoid)ありきの4輪計算式であります
static
SuspensionMoveState_t cal_omni_value(TargetMoveDirection_t direction, SuspensionStopType_t type,int32_t target_value, int32_t now_value, int offset_x, int offset_y, int offset_z){

  tc_const_t suspension_tcon;
  const int num_of_motor = 4;/*モータの個数*/
  int matrix[4][3] = {{100,100,100},
                      {-100,100,100},
                      {-100,-100,100},
                      {100,-100,100}};
  int input[3],hold_value[4][3],result[4];
  unsigned int idx;/*インデックス*/
  int i = 0,j = 0;
  int max = 0;
  int power = 0;
  int func = 0;
  int enhance = 0;

  static SuspensionMoveState_t state = MAXSPEED;
  static bool stop_flag = false;
  static int count = 0;

  count++;

  offset_z = -offset_z;

  state = MAXSPEED;
  
  switch(direction){
  case FRONT:
    /* if(target_value-now_value < 0 && state != OVERRUN){ */
    /*   state = OVERRUN; */
    /*   input[0] = 0; */
    /*   input[1] = 0; */
    /*   input[2] = 0; */
    /*   stop_flag = true; */
    /*   count = 0; */
    /*   break; */
    /* } */
    if(stop_flag){
      input[0] = 0;
      input[1] = 0;
      input[2] = 0;
      if(count>=5){
	state = MOVE_END;
	stop_flag = false;
	break;
      }
    }else{
      input[0] = offset_x;
      input[2] = offset_z;
      if(abs(target_value-now_value) > MOVEFRONT_SPEEDDOWN_VALUE){
	state = MAXSPEED;
	input[1] = offset_y + MOVEFRONT_MAX_DUTY;
      }else if(abs(target_value-now_value) < MOVEFRONT_STOP_VALUE){
	if(type == MOVE_STOP){
	  state = STOP;
	  stop_flag = true;
	  count = 0;
	  input[1] = offset_y;
	}else if(type == MOVE_CONTINUE){
	  input[1] = offset_y + MOVEFRONT_MIN_DUTY;
	  state = MOVE_END;
	  stop_flag = false;
	}
      }else{
	state = MINSPEED;
	input[1] = offset_y + MOVEFRONT_MIN_DUTY;
      }
    }
    suspension_tcon.inc_con = MOVEFRONT_INC_CON;
    suspension_tcon.dec_con = MOVEFRONT_DEC_CON;
    break;
    
  case BACK:
    /* if(target_value-now_value > 0 && state != OVERRUN){ */
    /*   state = OVERRUN; */
    /*   input[0] = 0; */
    /*   input[1] = 0; */
    /*   input[2] = 0; */
    /*   stop_flag = true; */
    /*   count = 0; */
    /*   break; */
    /* } */
    if(stop_flag){
      input[0] = 0;
      input[1] = 0;
      input[2] = 0;
      if(count>=5){
	state = MOVE_END;
	stop_flag = false;
	break;
      }
    }else{
      input[0] = offset_x;
      input[2] = offset_z;
      if(abs(target_value-now_value) > MOVEBACK_SPEEDDOWN_VALUE){
	state = MAXSPEED;
	input[1] = offset_y + MOVEBACK_MAX_DUTY;
      }else if(abs(target_value-now_value) < MOVEBACK_STOP_VALUE){
	if(type == MOVE_STOP){
	  state = STOP;
	  stop_flag = true;
	  count = 0;
	  input[1] = offset_y;
	}else if(type == MOVE_CONTINUE){
	  input[1] = offset_y + MOVEBACK_MIN_DUTY;
	  state = MOVE_END;
	  stop_flag = false;
	}
      }else{
	state = MINSPEED;
	input[1] = offset_y + MOVEBACK_MIN_DUTY;
      }
    }
    
    suspension_tcon.inc_con = MOVEBACK_INC_CON;
    suspension_tcon.dec_con = MOVEBACK_DEC_CON;
    break;
    
  case RIGHT:
    /* if(target_value-now_value < 0 && state != OVERRUN){ */
    /*   state = OVERRUN; */
    /*   input[0] = 0; */
    /*   input[1] = 0; */
    /*   input[2] = 0; */
    /*   stop_flag = true; */
    /*   count = 0; */
    /*   break; */
    /* } */
    if(stop_flag){
      input[0] = 0;
      input[1] = 0;
      input[2] = 0;
      if(count>=5){
	state = MOVE_END;
	stop_flag = false;
	break;
      }
    }else{
      input[1] = offset_y;
      input[2] = offset_z;
      if(abs(target_value-now_value) > MOVERIGHT_SPEEDDOWN_VALUE){
	state = MAXSPEED;
	input[0] = offset_x + MOVERIGHT_MAX_DUTY;
      }else if(abs(target_value-now_value) < MOVERIGHT_STOP_VALUE){
	if(type == MOVE_STOP){
	  state = STOP;
	  stop_flag = true;
	  count = 0;
	  input[1] = offset_x;
	}else if(type == MOVE_CONTINUE){
	  input[1] = offset_x + MOVERIGHT_MIN_DUTY;
	  state = MOVE_END;
	  stop_flag = false;
	}
      }else{
	state = MINSPEED;
	input[0] = offset_x + MOVERIGHT_MIN_DUTY;
      }
    }
    
    suspension_tcon.inc_con = MOVERIGHT_INC_CON;
    suspension_tcon.dec_con = MOVERIGHT_DEC_CON;
    break;
    
  case LEFT:
    /* if(target_value-now_value > 0 && state != OVERRUN){ */
    /*   state = OVERRUN; */
    /*   input[0] = 0; */
    /*   input[1] = 0; */
    /*   input[2] = 0; */
    /*   stop_flag = true; */
    /*   count = 0; */
    /*   break; */
    /* } */
    if(stop_flag){
      input[0] = 0;
      input[1] = 0;
      input[2] = 0;
      if(count>=5){
	state = MOVE_END;
	stop_flag = false;
	break;
      }
    }else{
      input[1] = offset_y;
      input[2] = offset_z;
      if(abs(target_value-now_value) > MOVELEFT_SPEEDDOWN_VALUE){
	state = MAXSPEED;
	input[0] = offset_x + MOVELEFT_MAX_DUTY;
      }else if(abs(target_value-now_value) < MOVELEFT_STOP_VALUE){
	if(type == MOVE_STOP){
	  state = STOP;
	  stop_flag = true;
	  count = 0;
	  input[1] = offset_x;
	}else if(type == MOVE_CONTINUE){
	  input[1] = offset_x + MOVELEFT_MIN_DUTY;
	  state = MOVE_END;
	  stop_flag = false;
	}
      }else{
	state = MINSPEED;
	input[0] = offset_x + MOVELEFT_MIN_DUTY;
      }
    }

    suspension_tcon.inc_con = MOVELEFT_INC_CON;
    suspension_tcon.dec_con = MOVELEFT_DEC_CON;
    break;

  case NOTHING:
    input[0] = offset_x;
    input[1] = offset_y;
    input[2] = offset_z;
    suspension_tcon.inc_con = MOVEOFFSET_INC_CON;
    suspension_tcon.dec_con = MOVEOFFSET_DEC_CON;
    state = MOVE_OFFSET;
    break;
  }

  //デューティのエンハンス値を計算
  if( cal_abs(input[0],input[1]) >= abs(input[2])){
    enhance = cal_abs(input[0],input[1]);
  }else{
    enhance = abs(input[2]);
  }

  //デューティ最大値(power)を計算
  func = MAX_DUTY/CONT_ABS_MAX;
  power = MAX_DUTY - func*(abs(CONT_ABS_MAX - enhance));
  
  //行列を計算
  for(i=0;i<4;i++){
    for(j=0;j<3;j++){
      hold_value[i][j] = matrix[i][j] * input[j];
    }
    result[i] = hold_value[i][0] + hold_value[i][1] + hold_value[i][2];
  }

  //計算値から最大値を求め、デューティ最大値に近づける
  max = abs(result[0]);
  for(i=1;i<4;i++){
    if(max<=abs(result[i])){
      max = abs(result[i]);
    }
  }
  for(i=0;i<4;i++){
    result[i] = result[i] * power/max;
  }
  
  /*for each motor*/
  for(i=0;i<num_of_motor;i++){
    /*それぞれの差分*/
    switch(i){
    case 0:
      idx = KUDO_LF;
      break;
    case 1:
      idx = KUDO_LB;
      break;
    case 2:
      idx = KUDO_RB;
      break;
    case 3:
      idx = KUDO_RF;
      break;
      
    default:return EXIT_FAILURE;
    }

    /* if(idx == KUDO_LF){ */
    /*   if(abs(result[i]) > 0){ */
    /* 	result[i] = result[i] + result[i]/100; */
    /*   }  */
    /* } */

    /* if(idx == KUDO_RF){ */
    /*   result[i] = result[i] + result[i]/100; */
    /* } */
    
    if(result[i]==0){
      trapezoidCtrl(0, &g_md_h[idx], &suspension_tcon);
    }else if(result[i]>0){
      trapezoidCtrl(-abs(result[i]), &g_md_h[idx], &suspension_tcon);
    }else if(result[i]<0){
      trapezoidCtrl(abs(result[i]), &g_md_h[idx], &suspension_tcon);
    }
  }
  
  return state;
}

static
ShotMechaResetState_t pull_mecha_reset(void){
  const tc_const_t pull_mecha_tcon = {
    .inc_con = PULLMECHA_INC_CON,
    .dec_con = PULLMECHA_DEC_CON
  };
  static ShotMechaResetState_t state;
  static int init_value = 0;
  static int count = 0;
  
  count++;
  
  switch(init_value){
  case 0:
    state = FIRSTRESET;
    if(_IS_PRESSED_PULLMECHA_LIMITSW()){
      trapezoidCtrl(0, &g_md_h[PULLMECHA], &pull_mecha_tcon);
      init_value++;
    }else{
      trapezoidCtrl(-PULLMECHA_MAX_DUTY, &g_md_h[PULLMECHA], &pull_mecha_tcon);
    }
    break;
  case 1:
    state = SECONDRESET;
    if(_IS_PRESSED_PULLMECHA_LIMITSW()){
      trapezoidCtrl(PULLMECHA_MAX_DUTY, &g_md_h[PULLMECHA], &pull_mecha_tcon);
    }else{
      count = 0;
      init_value++;
    }
    break;
  case 2:
    state = SECONDRESET;
    if(count >= 5){
      trapezoidCtrl(0, &g_md_h[PULLMECHA], &pull_mecha_tcon);
      init_value++;
      count = 0;
    }
    break;
  case 3:
    if(_IS_PRESSED_PULLMECHA_LIMITSW()){
      trapezoidCtrl(0, &g_md_h[PULLMECHA], &pull_mecha_tcon);
      if(count >= 10){
	state = FINISH;
	init_value = 0;
	count = 0;
      }
    }else{
      trapezoidCtrl(-PULLMECHA_MAX_DUTY, &g_md_h[PULLMECHA], &pull_mecha_tcon);
    }
    break;
  }

  return state;
}

static
ShotMechaResetState_t release_mecha_reset(void){
  const tc_const_t release_mecha_tcon = {
    .inc_con = RELEASEMECHA_INC_CON,
    .dec_con = RELEASEMECHA_DEC_CON
  };
  static ShotMechaResetState_t state;
  static int init_value = 0;
  static int count = 0;

  count++;
  
  switch(init_value){
  case 0:
    state = FIRSTRESET;
    if(_IS_PRESSED_RELEASEMECHA_LIMITSW()){
     trapezoidCtrl(0, &g_md_h[RELEASEMECHA], &release_mecha_tcon);
     init_value++;
    }else{
      trapezoidCtrl(RELEASEMECHA_MAX_DUTY, &g_md_h[RELEASEMECHA], &release_mecha_tcon);
    }
    break;
  case 1:
    state = SECONDRESET;
    if(_IS_PRESSED_RELEASEMECHA_LIMITSW()){
      trapezoidCtrl(-RELEASEMECHA_MAX_DUTY, &g_md_h[RELEASEMECHA], &release_mecha_tcon);
    }else{
      count = 0;
      init_value++;
    }
    break;
  case 2:
    state = SECONDRESET;
    if(count >= 15){
      trapezoidCtrl(0, &g_md_h[RELEASEMECHA], &release_mecha_tcon);
      init_value++;
      count = 0;
    }
    break;
  case 3:
    if(_IS_PRESSED_RELEASEMECHA_LIMITSW()){
      trapezoidCtrl(0, &g_md_h[RELEASEMECHA], &release_mecha_tcon);
      if(count >= 10){
	state = FINISH;
	init_value = 0;
	count = 0;
      }
    }else{
      trapezoidCtrl(RELEASEMECHA_MAX_DUTY, &g_md_h[RELEASEMECHA], &release_mecha_tcon);
    }
    break;
  }

  return state;
}

static
ShotMechaResetState_t pull_rele_mecha_reset(void){
  static ShotMechaResetState_t pullmecha = FIRSTRESET;
  static ShotMechaResetState_t releasemecha = FIRSTRESET;
  static ShotMechaResetState_t state = FIRSTRESET;

  if(pullmecha != FINISH){
    pullmecha = pull_mecha_reset();
  }
  if(releasemecha != FINISH){
    releasemecha = release_mecha_reset();
  }

  if(pullmecha == FINISH && releasemecha == FINISH){
    pullmecha = FIRSTRESET;
    releasemecha = FIRSTRESET;
    state = FINISH;
  }else{
    state = FIRSTRESET;
  }

  return state;
}

static
ShotMechaResetState_t degree_mecha_reset(void){
  const tc_const_t degree_mecha_tcon = {
    .inc_con = DEGREEMECHA_INC_CON,
    .dec_con = DEGREEMECHA_DEC_CON
  };
  static ShotMechaResetState_t state;
  static int init_value = 0;
  static int count = 0;
  
  count++;
  
  switch(init_value){
  case 0:
    state = FIRSTRESET;
    if(_IS_PRESSED_DEGREEMECHA_LIMITSW()){
      trapezoidCtrl(0, &g_md_h[DEGREEMECHA], &degree_mecha_tcon);
      init_value++;
    }else{
      trapezoidCtrl(DEGREEMECHA_MAX_DUTY, &g_md_h[DEGREEMECHA], &degree_mecha_tcon);
    }
    break;
  case 1:
    state = SECONDRESET;
    if(_IS_PRESSED_DEGREEMECHA_LIMITSW()){
      trapezoidCtrl(-DEGREEMECHA_MAX_DUTY, &g_md_h[DEGREEMECHA], &degree_mecha_tcon);
    }else{
      count = 0;
      init_value++;
    }
    break;
  case 2:
    state = SECONDRESET;
    if(count >= 10){
      trapezoidCtrl(0, &g_md_h[DEGREEMECHA], &degree_mecha_tcon);
      init_value++;
      count = 0;
    }
    break;
  case 3:
    if(_IS_PRESSED_DEGREEMECHA_LIMITSW()){
      trapezoidCtrl(0, &g_md_h[DEGREEMECHA], &degree_mecha_tcon);
      if(count >= 15){
	state = FINISH;
	init_value = 0;
	count = 0;
      }
    }else{
      trapezoidCtrl(DEGREEMECHA_MAX_DUTY, &g_md_h[DEGREEMECHA], &degree_mecha_tcon);
    }
    break;
  }

  return state;
}


static
ShotMechaSetState_t release_mecha_set_target(int32_t target_value, int32_t now_value, int reset){
  const tc_const_t release_mecha_set_tcon = {
    .inc_con = RELEASEMECHA_INC_CON,
    .dec_con = RELEASEMECHA_DEC_CON
  };
  static ShotMechaSetState_t state = SET_READY;
  static int count = 0;
  static bool set_flag = false;

  count++;

  if(reset == 1){
    state = SET_READY;
    set_flag = false;
    count = 0;
    return state;
  }
  
  if(set_flag){
    trapezoidCtrl(0, &g_md_h[RELEASEMECHA], &release_mecha_set_tcon);
    if(count>=15){
      state = SET_OK;
      set_flag = false;
      count = 0;
    }
  }else{
    if(abs(target_value-now_value) > RELEASEMECHA_SPEEDDOWN_VALUE){
      state = SET_READY;
      trapezoidCtrl(-RELEASEMECHA_MAX_DUTY, &g_md_h[RELEASEMECHA], &release_mecha_set_tcon);
    }else if(abs(target_value-now_value) < RELEASEMECHA_STOP_VALUE){
      trapezoidCtrl(0, &g_md_h[RELEASEMECHA], &release_mecha_set_tcon);
      set_flag = true;
      count = 0;
    }else{
      state = SET_READY;
      trapezoidCtrl(-RELEASEMECHA_MIN_DUTY, &g_md_h[RELEASEMECHA], &release_mecha_set_tcon);
    }
  }

  return state;
}

static
ShotMechaSetState_t degree_mecha_set_target(int32_t target_value, int32_t now_value, int reset){
  const tc_const_t degree_mecha_set_tcon = {
    .inc_con = DEGREEMECHA_INC_CON,
    .dec_con = DEGREEMECHA_DEC_CON
  };
  static ShotMechaSetState_t state = SET_READY;
  static int count = 0;
  static bool set_flag = false;

  count++;

  if(reset == 1){
    state = SET_READY;
    set_flag = false;
    return state;
  }

  if(set_flag){
    trapezoidCtrl(0, &g_md_h[DEGREEMECHA], &degree_mecha_set_tcon);
    if(count>=15){
      state = SET_OK;
      set_flag = false;
      count = 0;
    }
  }else{
    if(abs(target_value-now_value) > DEGREEMECHA_SPEEDDOWN_VALUE){
      state = SET_READY;
      if(target_value-now_value < 0){
	trapezoidCtrl(-DEGREEMECHA_MAX_DUTY, &g_md_h[DEGREEMECHA], &degree_mecha_set_tcon);
      }else{
	trapezoidCtrl(DEGREEMECHA_MAX_DUTY, &g_md_h[DEGREEMECHA], &degree_mecha_set_tcon);
      }
    }else if(abs(target_value-now_value) < DEGREEMECHA_STOP_VALUE){
      trapezoidCtrl(0, &g_md_h[DEGREEMECHA], &degree_mecha_set_tcon);
      set_flag = true;
      count = 0;
    }else{
      state = SET_READY;
      if(target_value-now_value < 0){
	trapezoidCtrl(-DEGREEMECHA_MIN_DUTY, &g_md_h[DEGREEMECHA], &degree_mecha_set_tcon);
      }else{
	trapezoidCtrl(DEGREEMECHA_MIN_DUTY, &g_md_h[DEGREEMECHA], &degree_mecha_set_tcon);
      }
    }
  }

  return state;
  
}

static
ShotMechaSetState_t shotmecha_set_waitposition(void){
  static ShotMechaSetState_t state = SET_READY;
  static ShotMechaResetState_t pullmecha = FIRSTRESET;
  static ShotMechaResetState_t releasemecha = FIRSTRESET;
  static bool set_position = false;
  const tc_const_t pull_mecha_tcon = {
    .inc_con = PULLMECHA_INC_CON,
    .dec_con = PULLMECHA_DEC_CON
  };

  state = SET_READY;
  
  if(pullmecha != FINISH){
    pullmecha = pull_mecha_reset();
  }
  if(releasemecha != FINISH){
    releasemecha = release_mecha_reset();
  }

  if(_IS_PRESSED_WAITPOSITION_LIMITSW()){
    pullmecha = FINISH;
    set_position = true;
    g_md_h[PULLMECHA].duty = 0;
    g_md_h[PULLMECHA].mode = D_MMOD_BRAKE;
  }
  
  if(pullmecha == FINISH && !set_position){
    trapezoidCtrl(PULLMECHA_MAX_DUTY, &g_md_h[PULLMECHA], &pull_mecha_tcon);
  }else if(pullmecha == FINISH && set_position && releasemecha == FINISH){
    pullmecha = FIRSTRESET;
    releasemecha = FIRSTRESET;
    set_position = false;
    state = SET_OK;
  }

  return state;
  
}


static
AdjustLineState_t move_linecenter(int32_t straight_duty, int32_t time){
  static AdjustLineState_t state = ADJUST_READY;
  static int count = 0;
  static LineTrace_State_t linetrace_state = RESET_DATA;
  int linetrace_duty = 0;
  static Robot_Direction_t robot_direction = {
    .gap_degree = 0,
    .direction = NO_LINE,
  };
  
  linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,NO_LINE);
  linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;
  linetrace_duty = linetrace_duty_decide(linetrace_state,linetrace_duty);

  count++;
  
  if(count <= time){
    state = ADJUST_READY;
    
    switch(robot_direction.direction){
    case D_RIGHT:
      cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, linetrace_duty, straight_duty, 0);
      break;
    case D_LEFT:
      cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -linetrace_duty, straight_duty, 0);
      break;
    case D_FRONT_RIGHT:
      cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, straight_duty, -linetrace_duty);
      break;
    case D_FRONT_LEFT:
      cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, straight_duty, linetrace_duty);
      break;
    case D_MIDDLE:
      cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, straight_duty, 0);
      break;
    case NO_LINE:
      cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, straight_duty, 0);
      break;
    }
  }else{
    state = ADJUST_OK;
    robot_direction.direction = NO_LINE;
    robot_direction.gap_degree = 0;
    count = 0;
    linetrace_state = RESET_DATA;
    linetrace_duty = 0;
  }

  return state;
}


static
ShotState_t auto_shot(const int32_t power, const int32_t degree, int reset){
  static ShotMechaResetState_t pullmecha = FIRSTRESET;
  static ShotMechaResetState_t releasemecha = FIRSTRESET;
  static ShotMechaSetState_t releasemecha_set = SET_READY;
  static ShotMechaSetState_t degreemecha_set = SET_READY;
  static ShotState_t shot_state = PULLING;
  static ShotState_t return_shot_state = SHOTMECHA_RESET;
  static ShotActionState_t shot_action = RESET_MECHA;
  static int32_t releasemecha_encoder = 0;
  static int32_t degreemecha_encoder = 0;

  if(reset == 1){
    return_shot_state = SHOTMECHA_RESET;
    pullmecha = FIRSTRESET;
    releasemecha = FIRSTRESET;
    releasemecha_set = SET_READY;
    degreemecha_set = SET_READY;
    shot_state = PULLING;
    shot_action = RESET_MECHA;

    return return_shot_state;
  }
  
  releasemecha_encoder = I2C_Encoder(0,GET_ENCODER_VALUE);
  degreemecha_encoder = I2C_Encoder(1,GET_ENCODER_VALUE);
  
  switch(shot_action){
  case RESET_MECHA:
    return_shot_state = SHOTMECHA_RESET;
    
    if(pullmecha != FINISH){
      pullmecha = pull_mecha_reset();
    }
    if(releasemecha != FINISH){
      releasemecha = release_mecha_reset();
    }
    if(degreemecha_set != SET_OK){
      degreemecha_set = degree_mecha_set_target(degree,degreemecha_encoder,0);
    }
    if(pullmecha == FINISH && releasemecha == FINISH){
      I2C_Encoder(0,RESET_ENCODER_VALUE);
      shot_action = SET_POWER;
    }
    break;
      
  case SET_POWER:
    return_shot_state = SHOTMECHA_RESET;
    
    if(releasemecha_set != SET_OK){
      releasemecha_set = release_mecha_set_target(power,releasemecha_encoder,0);
    }
    if(degreemecha_set != SET_OK){
      degreemecha_set = degree_mecha_set_target(degree,degreemecha_encoder,0);
    }
    if(releasemecha_set == SET_OK && degreemecha_set == SET_OK){
      I2C_Encoder(0,RESET_ENCODER_VALUE);
      shot_action = SHOT_BOTTLE;
      return_shot_state = PULLING;
      releasemecha_set = SET_READY;
      degreemecha_set = SET_READY;
    }
    break;
      
  case SHOT_BOTTLE:
    if(shot_state != COMPLETED_SHOT){
      shot_state = shot();
    }else if(shot_state == COMPLETED_SHOT){
      return_shot_state = COMPLETED_SHOT;
      pullmecha = FIRSTRESET;
      releasemecha = FIRSTRESET;
      releasemecha_set = SET_READY;
      degreemecha_set = SET_READY;
      shot_state = PULLING;
      shot_action = RESET_MECHA;
    }
    break;
  }

  return return_shot_state;
}

static
ShotState_t shot(void){
  const tc_const_t shot_pull_mecha_tcon = {
    .inc_con = PULLMECHA_INC_CON,
    .dec_con = PULLMECHA_DEC_CON
  };
  static ShotState_t state = PULLING;
  static int count = 0;
  static bool shot_flag = false;
  static bool shot_wait_flag = false;
  
  count ++;

  if(shot_wait_flag){
    state = WAIT_PERFECT_RELEASE;
    if(count>=1){
      trapezoidCtrl(0, &g_md_h[PULLMECHA], &shot_pull_mecha_tcon);
      shot_wait_flag = false;
      shot_flag = true;
      count = 0;
    }
  }else if(shot_flag){
    trapezoidCtrl(0, &g_md_h[PULLMECHA], &shot_pull_mecha_tcon);
    if(count>=15){
      state = COMPLETED_SHOT;
      shot_flag = false;
      count = 0;
    }
  }else{
    if(!_IS_PRESSED_RELE_PULL_LIMITSW()){
      state = PULLING;
      trapezoidCtrl(PULLMECHA_MAX_DUTY, &g_md_h[PULLMECHA], &shot_pull_mecha_tcon);
    }else{
      state = WAIT_PERFECT_RELEASE;
      shot_wait_flag = true;
      count = 0;
    }
  }
  
  return state;
}

static
int Pull_mecha_stop(void){  
  g_md_h[PULLMECHA].duty = 0;
  g_md_h[PULLMECHA].mode = D_MMOD_FREE;
  return EXIT_SUCCESS;
}

static
int release_mecha_stop(void){  
  g_md_h[RELEASEMECHA].duty = 0;
  g_md_h[RELEASEMECHA].mode = D_MMOD_FREE;
  return EXIT_SUCCESS;
}

static
int suspension_stop(void){  
  g_md_h[KUDO_LF].duty = 0;
  g_md_h[KUDO_LF].mode = D_MMOD_BRAKE;
  g_md_h[KUDO_LB].duty = 0;
  g_md_h[KUDO_LB].mode = D_MMOD_BRAKE;
  g_md_h[KUDO_RB].duty = 0;
  g_md_h[KUDO_RB].mode = D_MMOD_BRAKE;
  g_md_h[KUDO_RF].duty = 0;
  g_md_h[KUDO_RF].mode = D_MMOD_BRAKE;
  return EXIT_SUCCESS;
}

static
int all_motor_stop(void){
  for(int i=0;i<DD_NUM_OF_MD-1;i++){
    g_md_h[i].duty = 0;
    g_md_h[i].mode = D_MMOD_FREE;
  }
  return EXIT_SUCCESS;
}

static
int Led_brink(LedMode_t mode, int brink_interval, int duty){
  static bool change_state_flag = false;
  static int count = 0;
  
  if(mode == OFF){
    MW_GPIOWrite(GPIOCID,GPIO_PIN_4,GPIO_PIN_RESET);
    count = 0;
    
    return EXIT_SUCCESS;
  }

  count++;
  if(count == (int)( ((double)(brink_interval*2)/100.0) * (double)duty )){
    MW_GPIOWrite(GPIOCID,GPIO_PIN_4,GPIO_PIN_RESET);
  }

  if(count == (brink_interval*2)){
    MW_GPIOWrite(GPIOCID,GPIO_PIN_4,GPIO_PIN_SET);
    count = 0;
  }
  
  return EXIT_SUCCESS;
}

static
LineTrace_State_t find_robotdirection(uint8_t *front, uint8_t *behind, Robot_Direction_t *robot_direction, int reset, Direction_t reset_direction){
  //LED側
  // 
  //   16    15    14    13    12    11    10    9     8     7     6     5     4     3     2     1
  //   o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o  
  //^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
  //33 32 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1 

  static Robot_Direction_t recent_direction = {
    .direction = NO_LINE,
    .gap_degree = 0,
  };
  LineTrace_State_t state;
  Direction_t front_direction;
  Direction_t behind_direction;
  uint16_t front_decimal = 0;
  uint16_t behind_decimal = 0;
  int front_binary[16];
  int behind_binary_reverse[16];
  int behind_binary[16];
  int front_right_on = 0,front_left_on = 0,front_middle = 0;
  int behind_right_on = 0,behind_left_on = 0,behind_middle = 0;
  static int recent_front_middle = 0,recent_behind_middle = 0;
  static bool first_flag = true;
  int i;

  if(reset==1){
    recent_direction.direction = NO_LINE;
    recent_direction.gap_degree = 0;
    state = RESET_DATA;
    first_flag = true;
    return state; 
  }

  if(first_flag){
    if(reset_direction == D_MIDDLE){
      recent_front_middle = 17;
      recent_behind_middle = 17;
    }else if(reset_direction == D_RIGHT){
      recent_front_middle = 33;
      recent_behind_middle = 33;
    }else if(reset_direction == D_LEFT){
      recent_front_middle = 1;
      recent_behind_middle = 1;
    }
    first_flag = false;
  }
  
  front_decimal = (uint16_t)((*front) << 8) + (uint8_t)(*(front+1));
  change_binary(front_decimal,front_binary);
  
  for(i=0;i<16;i++){
    if(front_binary[i] == 1){
      front_right_on = i+1;
      break;
    }
  }
  
  for(i=15;i>=0;i--){
    if(front_binary[i] == 1){
      front_left_on = i+1;
      break;
    }
  }

  front_middle = ((front_left_on*2)-(front_right_on*2))/2 + front_right_on*2;

  behind_decimal = (uint16_t)((*behind) << 8) + (uint8_t)(*(behind+1));
  change_binary(behind_decimal,behind_binary_reverse);

  for(i=0;i<16;i++){
    behind_binary[i] = behind_binary_reverse[15-i];
  }
  
  for(i=0;i<16;i++){
    if(behind_binary[i] == 1){
      behind_right_on = i+1;
      break;
    }
  }
  
  for(i=15;i>=0;i--){
    if(behind_binary[i] == 1){
      behind_left_on = i+1;
      break;
    }
  }

  behind_middle = ((behind_left_on*2)-(behind_right_on*2))/2 + behind_right_on*2;

  if(front_decimal == 0 && behind_decimal == 0){
    if(recent_direction.direction == NO_LINE){
      robot_direction->direction = NO_LINE;
      robot_direction->gap_degree = 0;
    }else{
      robot_direction->direction = recent_direction.direction;
      robot_direction->gap_degree = recent_direction.gap_degree;
    }
    state = RECENT_DATA;
    return state;
    
  }else if(front_decimal == 0){
    if(recent_front_middle > 31){
      front_middle = 39;
    }else if(recent_front_middle < 3){
      front_middle = -5;
    }else{
      front_middle = recent_front_middle;
    }
    state = CURRENT_DATA;
  }else if(behind_decimal == 0){
    if(recent_behind_middle > 31){
      behind_middle = 39;
    }else if(recent_behind_middle < 3){
      behind_middle = -5;
    }else{
      behind_middle = recent_behind_middle;
    }
    state = CURRENT_DATA;
  }
  
  if(front_middle < 17){
    front_direction = D_RIGHT;
  }else if(front_middle > 17){
    front_direction = D_LEFT;
  }else{
    front_direction = D_MIDDLE;
  }

  if(behind_middle < 17){
    behind_direction = D_RIGHT;
  }else if(behind_middle > 17){
    behind_direction = D_LEFT;
  }else{
    behind_direction = D_MIDDLE;
  }

  if(front_direction == D_MIDDLE && behind_direction == D_MIDDLE){
    robot_direction->direction = D_MIDDLE;
    robot_direction->gap_degree = 0;
  }else if(front_direction == D_RIGHT && behind_direction == D_RIGHT){
    robot_direction->direction = D_LEFT;
    if(abs(front_middle-17) > abs(behind_middle-17)){
      robot_direction->gap_degree = abs(front_middle-17);
    }else{
      robot_direction->gap_degree = abs(behind_middle-17);
    }
  }else if(front_direction == D_LEFT && behind_direction == D_LEFT){
    robot_direction->direction = D_RIGHT;
    if(abs(front_middle-17) > abs(behind_middle-17)){
      robot_direction->gap_degree = abs(front_middle-17);
    }else{
      robot_direction->gap_degree = abs(behind_middle-17);
    }
  }else if((front_direction == D_RIGHT && behind_direction == D_LEFT)|| (front_direction == D_MIDDLE && behind_direction == D_LEFT) || (front_direction == D_RIGHT && behind_direction == D_MIDDLE) ){
    robot_direction->direction = D_FRONT_LEFT;
    robot_direction->gap_degree = abs(front_middle - behind_middle);
  }else if((front_direction == D_LEFT && behind_direction == D_RIGHT) || (front_direction == D_MIDDLE && behind_direction == D_RIGHT) || (front_direction == D_LEFT && behind_direction == D_MIDDLE)){
    robot_direction->direction = D_FRONT_RIGHT;
    robot_direction->gap_degree = abs(front_middle - behind_middle);
  }

  recent_front_middle = front_middle;
  recent_behind_middle = behind_middle;
  recent_direction.direction = robot_direction->direction;
  recent_direction.gap_degree = robot_direction->gap_degree;

  state = CURRENT_DATA;
  return state;
  
  /* for(i=15;i>=0;i--){ */
  /*   printf("%2d",front_binary[i]); */
  /* } */
  /* printf("\n"); */
  /* for(i=0;i<33-front_middle;i++){ */
  /*   printf(" "); */
  /* } */
  /* printf("^\n"); */

  /* for(i=15;i>=0;i--){ */
  /*   printf("%2d",behind_binary[i]); */
  /* } */
  /* printf("\n"); */
  /* for(i=0;i<33-behind_middle;i++){ */
  /*   printf(" "); */
  /* } */
  /* printf("^\n"); */

  /* printf("\n%d",front_right_on); */
  /* printf("\n%d",front_left_on); */
  /* printf("\n%d",front_middle); */
}

static
int change_binary(uint16_t value, int *arr){
  unsigned int bit = (1 << (2*8-1));
  int i;
  
  if(value > 65535){
    return -1;
  }

  i=15;
  for(;bit != 0 ; bit>>=1){
    if(value & bit){
      *(arr + i) = 1;
    }else{
      *(arr + i) = 0;
    }
    i--;
  }

  return 0;  
}


static
SuspensionMoveState_t go_to_target(Desk_t target, Desk_t now_place, int32_t distance_to_wall, GameZone_t zone ,int reset){
  //              ===  
  //             | A |
  //              ===
  //              ===
  //             | B |
  //              ===
  //              ===
  //             | C | 
  //              ===
  //               
  //        =====       =
  //BOTTOM  |D_1|  D  |D_2|  TOP
  //        =====       =
  //
  //      ===     ===     ===
  //     | E |   | F |   | G |
  //      ===     ===     ===
  //
  //               S
  //           =========
  //          |STARTZONE|
  //           =========
  
  static SuspensionMoveState_t state = MAXSPEED;
  TargettoTarget_t t_t;
  static LineTrace_State_t linetrace_state = RESET_DATA;
  static int linetrace_duty = 0;
  static SuspensionMoveState_t sus_state = MAXSPEED;
  static int line_count = 0;
  static bool line_flag = false;
  static bool vartical_move_end = false;
  static bool horizontal_move_end = false;
  static int32_t x_encoder = 0;
  static int32_t y_encoder = 0;
  static bool encoder_move_end = false;
  static int over_line = 0;
  int target_encoder_value = 0;

  static int32_t distance_to_wall_d = 0;
  static bool moved_x_enc_end = false;
  static bool moved_y_enc_end = false;
  static bool go_line = false;
  static bool trace_line = false;
  
  static bool first_flag = true;
  
  static Robot_Direction_t robot_direction = {
    .gap_degree = 0,
    .direction = NO_LINE,
  };

  static int on_line_count = 0;
  static int out_line_count = 0;
  static int count = 0;

  const int32_t deskd_x_enc_tar = 8000; //11900
  const int32_t deskd_y_enc_tar = 3000; //6000
  
  if(reset == 1){
    on_line_count = 0;
    out_line_count = 0;
    line_count = 0;

    linetrace_state = RESET_DATA;
    linetrace_duty = 0;
    line_count = 0;
    line_flag = false;
    vartical_move_end = false;
    horizontal_move_end = false;
    x_encoder = 0;
    y_encoder = 0;
    encoder_move_end = false;
    over_line = 0;
    find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT);

    go_line = false;
    trace_line = false;

    state = MAXSPEED;
    
    return state;
  }
  
  count++;

  if(first_flag){
    on_line_count = 0;
    out_line_count = 0;
    line_count = 0;
    
    DD_encoder1reset();
    DD_encoder2reset();
    first_flag = false;
  }
  
  if(target == MV_FRONT && now_place == START_ZONE){
    t_t = S_A;
  }else if(target == MV_CENTER && now_place == START_ZONE){
    t_t = S_B;
  }else if(target == MV_BACK && now_place == START_ZONE){
    t_t = S_C;
  }else if(target == FIX_TWO && now_place == START_ZONE){
    t_t = S_D;
  }else if(target == START_ZONE && now_place == MV_FRONT){
    t_t = A_S;
  }else if(target == MV_CENTER && now_place == MV_FRONT){
    t_t = A_B;
  }else if(target == MV_BACK && now_place == MV_FRONT){
    t_t = A_C;
  }else if(target == FIX_TWO && now_place == MV_FRONT){
    t_t = A_D;
  }else if(target == START_ZONE && now_place == MV_CENTER){
    t_t = B_S;
  }else if(target == MV_FRONT && now_place == MV_CENTER){
    t_t = B_A;
  }else if(target == MV_BACK && now_place == MV_CENTER){
    t_t = B_C;
  }else if(target == FIX_TWO && now_place == MV_CENTER){
    t_t = B_D;
  }else if(target == START_ZONE && now_place == MV_BACK){
    t_t = C_S;
  }else if(target == MV_FRONT && now_place == MV_BACK){
    t_t = C_A;
  }else if(target == MV_CENTER && now_place == MV_BACK){
    t_t = C_B;
  }else if(target == FIX_TWO && now_place == MV_BACK){
    t_t = C_S;
  }else if(target == START_ZONE && now_place == FIX_TWO){
    t_t = D_S;
  }else if(target == MV_FRONT && now_place == FIX_TWO){
    t_t = D_A;
  }else if(target == MV_CENTER && now_place == FIX_TWO){
    t_t = D_B;
  }else if(target == MV_BACK && now_place == FIX_TWO){
    t_t = D_C;
  }else if((target == FIX_RIGHT || target == FIX_LEFT) && now_place == START_ZONE){
    t_t = S_E;
  }else if(target == START_ZONE && (now_place == FIX_RIGHT || now_place == FIX_LEFT)){
    t_t = E_S;
  }

  x_encoder = DD_encoder1Get_int32();
  y_encoder = DD_encoder2Get_int32();
  
  switch(t_t){
  case S_D:

    if(zone == RED){
      target_encoder_value = target_encoder_value * -1;
    }
    
    /////横移動//////
    if(!horizontal_move_end){
      state = MAXSPEED;

      if(zone == RED){
	linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,D_RIGHT);
      }else{
	linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,D_LEFT);
      }
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;

      linetrace_duty = linetrace_duty_decide(linetrace_state,linetrace_duty);
  
      if(line_count < 1){

	if(abs(x_encoder) >= STARTZONE_LINE_VALUE){
	  if(linetrace_state == CURRENT_DATA){
	    out_line_count = 0;
	    on_line_count++;
	    if(on_line_count >= 8 && !line_flag){
	      line_count++;
	      on_line_count = 0;
	      line_flag = true;
	    }
	  }
	  if(linetrace_state == RECENT_DATA){
	    on_line_count = 0;
	    out_line_count++;
	    if(out_line_count >= 8){
	      out_line_count = 0;
	      line_flag = false;
	    }
	  }
	}
	if(!encoder_move_end){
	  if(zone == RED){
	    sus_state = cal_omni_value(LEFT, MOVE_CONTINUE, -deskd_x_enc_tar, x_encoder, 0, MOVE_DESKD_FRONT_MAX_DUTY, MOVESIDE_SPIN_MAX_DUTY);
	  }else{
	    sus_state = cal_omni_value(RIGHT, MOVE_CONTINUE, deskd_x_enc_tar, x_encoder, 0, MOVE_DESKD_FRONT_MAX_DUTY, -MOVESIDE_SPIN_MAX_DUTY);
	  }
	  if(sus_state == MOVE_END){
	    encoder_move_end = true;
	  }
	}else{
	  if(zone == RED){
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVELEFT_MIN_DUTY, MOVE_DESKD_FRONT_MIN_DUTY, MOVESIDE_SPIN_MIN_DUTY);
	  }else{
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVERIGHT_MIN_DUTY, MOVE_DESKD_FRONT_MIN_DUTY, -MOVESIDE_SPIN_MIN_DUTY);
	  }
	}

      }else{
	on_line_count = 0;
	out_line_count = 0;
	line_count = 0;
	line_flag = false;
	encoder_move_end = false;
	horizontal_move_end = true;
	sus_state = MAXSPEED;
	find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT);

	distance_to_wall_d = distance_to_wall;
	distance_to_wall_d -= abs(y_encoder);
	
	DD_encoder1reset();
	DD_encoder2reset();
      }
    }

    ////縦移動////
    if(horizontal_move_end && !vartical_move_end){
      state = MAXSPEED;
      
      linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,NO_LINE);
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;
      
      linetrace_duty = linetrace_duty_decide(linetrace_state,linetrace_duty);

      if(_IS_PRESSED_FRONT_FOOTSW()){
	DD_encoder1reset();
	DD_encoder2reset();
	vartical_move_end = true;
      }else{

	/* if(!encoder_move_end){ */
	/*   switch(robot_direction.direction){ */
	/*   case D_RIGHT: */
	/*     sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall_d, y_encoder, linetrace_duty, 0, 0); */
	/*     break; */
	/*   case D_LEFT: */
	/*     sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall_d, y_encoder, -linetrace_duty, 0, 0); */
	/*     break; */
	/*   case D_FRONT_RIGHT: */
	/*     sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall_d, y_encoder, 0, 0, -linetrace_duty); */
	/*     break; */
	/*   case D_FRONT_LEFT: */
	/*     sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall_d, y_encoder, 0, 0, linetrace_duty); */
	/*     break; */
	/*   case D_MIDDLE: */
	/*     sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall_d, y_encoder, 0, 0, 0); */
	/*     break; */
	/*   case NO_LINE: */
	/*     sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall_d, y_encoder, 0, 0, 0); */
	/*     break; */
	/*   } */
	/*   if(sus_state == MOVE_END){ */
	/*     encoder_move_end = true; */
	/*   } */
	  
	/* }else{ */
	  
	switch(robot_direction.direction){
	case D_RIGHT:
	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, linetrace_duty, MOVE_DESKD_FRONT_MIN_DUTY, 0);
	  break;
	case D_LEFT:
	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -linetrace_duty, MOVE_DESKD_FRONT_MIN_DUTY, 0);
	  break;
	case D_FRONT_RIGHT:
	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVE_DESKD_FRONT_MIN_DUTY, -linetrace_duty);
	  break;
	case D_FRONT_LEFT:
	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVE_DESKD_FRONT_MIN_DUTY, linetrace_duty);
	  break;
	case D_MIDDLE:
	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVE_DESKD_FRONT_MIN_DUTY, 0);
	  break;
	case NO_LINE:
	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVE_DESKD_FRONT_MIN_DUTY, 0);
	  break;
	  /* } */
	  
	}
      }
    }

    if(horizontal_move_end && vartical_move_end){
      linetrace_state = RESET_DATA;
      linetrace_duty = 0;
      line_count = 0;
      line_flag = false;
      x_encoder = 0;
      y_encoder = 0;
      over_line = 0;
      suspension_stop();
      DD_encoder1reset();
      DD_encoder2reset();
      first_flag = true;
      state = MOVE_END;
      encoder_move_end = false;
      sus_state = MAXSPEED;
      find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT);
      horizontal_move_end = false;
      vartical_move_end = false;
    }
    break;

    
    /* if(zone == RED){ */
    /*   linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,D_RIGHT); */
    /* }else{ */
    /*   linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,D_LEFT); */
    /* } */
    /* linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY; */
    /* linetrace_duty = linetrace_duty_decide(linetrace_state,linetrace_duty); */

    /* if(!go_line && !trace_line){ */
    /*   state = MAXSPEED; */
      
    /*   if(line_count < 1){ */
    /* 	if(linetrace_state == CURRENT_DATA){ */
    /* 	  out_line_count = 0; */
    /* 	  on_line_count++; */
    /* 	  if(on_line_count >= 8 && !line_flag){ */
    /* 	    line_count++; */
    /* 	    on_line_count = 0; */
    /* 	    line_flag = true; */
    /* 	  } */
    /* 	} */
    /* 	if(linetrace_state == RECENT_DATA){ */
    /* 	  on_line_count = 0; */
    /* 	  out_line_count++; */
    /* 	  if(out_line_count >= 8){ */
    /* 	    out_line_count = 0; */
    /* 	    line_flag = false; */
    /* 	  } */
    /* 	} */

    /* 	if((int)(abs(x_encoder)) >= deskd_x_enc_tar){ */
    /* 	  moved_x_enc_end = true; */
    /* 	} */
    /* 	if((int)(abs(y_encoder)) >= deskd_y_enc_tar){ */
    /* 	  moved_y_enc_end = true; */
    /* 	} */
	
    /* 	if(!moved_x_enc_end && !moved_y_enc_end){ */
    /* 	  if(zone == RED){ */
    /* 	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVE_DESKD_SIDE_MAX_DUTY, MOVE_DESKD_FRONT_MAX_DUTY, 0); */
    /* 	  }else{ */
    /* 	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -MOVE_DESKD_SIDE_MAX_DUTY, MOVE_DESKD_FRONT_MAX_DUTY, 0); */
    /* 	  } */
    /* 	}else if(!moved_x_enc_end && moved_y_enc_end){ */
    /* 	  if(zone == RED){ */
    /* 	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVE_DESKD_SIDE_MAX_DUTY, MOVE_DESKD_FRONT_MIN_DUTY, 0); */
    /* 	  }else{ */
    /* 	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -MOVE_DESKD_SIDE_MAX_DUTY, MOVE_DESKD_FRONT_MIN_DUTY, 0); */
    /* 	  } */
    /* 	}else if(moved_x_enc_end && !moved_y_enc_end){ */
    /* 	  if(zone == RED){ */
    /* 	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVE_DESKD_SIDE_MIN_DUTY, MOVE_DESKD_FRONT_MAX_DUTY, 0); */
    /* 	  }else{ */
    /* 	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -MOVE_DESKD_SIDE_MIN_DUTY, MOVE_DESKD_FRONT_MAX_DUTY, 0); */
    /* 	  } */
    /* 	}else if(moved_x_enc_end && moved_y_enc_end){ */
    /* 	  if(zone == RED){ */
    /* 	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVE_DESKD_SIDE_MIN_DUTY, MOVE_DESKD_FRONT_MIN_DUTY, 0); */
    /* 	  }else{ */
    /* 	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -MOVE_DESKD_SIDE_MIN_DUTY, MOVE_DESKD_FRONT_MIN_DUTY, 0); */
    /* 	  } */
    /* 	  encoder_move_end = true; */
    /* 	} */
    /*   }else{ */
    /* 	go_line = true; */
    /* 	on_line_count = 0; */
    /* 	out_line_count = 0; */
    /* 	line_count = 0; */
    /* 	line_flag = false; */
    /* 	sus_state = MAXSPEED; */
    /* 	find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT); */
    /*   } */
    /* }else if(go_line && !trace_line){ */
    /*   state = MAXSPEED; */
      
    /*   if(_IS_PRESSED_FRONT_FOOTSW()){ */
    /* 	DD_encoder1reset(); */
    /* 	DD_encoder2reset(); */
    /* 	trace_line = true; */
    /*   }else{ */
    /* 	switch(robot_direction.direction){ */
    /* 	case D_RIGHT: */
    /* 	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, linetrace_duty, MOVEFRONT_MIN_DUTY, 0); */
    /* 	  break; */
    /* 	case D_LEFT: */
    /* 	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -linetrace_duty, MOVEFRONT_MIN_DUTY, 0); */
    /* 	  break; */
    /* 	case D_FRONT_RIGHT: */
    /* 	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, -linetrace_duty); */
    /* 	  break; */
    /* 	case D_FRONT_LEFT: */
    /* 	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, linetrace_duty); */
    /* 	  break; */
    /* 	case D_MIDDLE: */
    /* 	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, 0); */
    /* 	  break; */
    /* 	case NO_LINE: */
    /* 	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, 0); */
    /* 	  break; */
    /* 	} */
    /*   } */
    /* }else if(go_line && trace_line){ */
    /*   go_line = false; */
    /*   trace_line = false; */
    /*   on_line_count = 0; */
    /*   out_line_count = 0; */
    /*   line_count = 0; */
    /*   line_flag = false; */
    /*   sus_state = MAXSPEED; */
    /*   find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT); */

    /*   linetrace_state = RESET_DATA; */
    /*   linetrace_duty = 0; */
    /*   line_count = 0; */
    /*   line_flag = false; */
    /*   x_encoder = 0; */
    /*   y_encoder = 0; */
    /*   over_line = 0; */
    /*   suspension_stop(); */
    /*   DD_encoder1reset(); */
    /*   DD_encoder2reset(); */
    /*   first_flag = true; */
    /*   state = MOVE_END; */
    /*   encoder_move_end = false; */
    /*   sus_state = MAXSPEED; */
    /*   find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT); */
    /*   horizontal_move_end = false; */
    /*   vartical_move_end = false; */
    /* } */
    /* break; */
    
  case S_A:
  case S_B:
  case S_C:

    if(t_t == S_A){
      target_encoder_value = MOVEGO_LINE_A_VALUE;
      over_line = 4;
    }else if(t_t == S_B){
      target_encoder_value = MOVEGO_LINE_B_VALUE;
      over_line = 3;
    }else if(t_t == S_C){
      target_encoder_value = MOVEGO_LINE_C_VALUE;
      over_line = 2;
    }else if(t_t == S_D){
      target_encoder_value = MOVEGO_LINE_D_VALUE;
      over_line = 1;
    }

    if(zone == RED){
      target_encoder_value = target_encoder_value * -1;
    }
    
    /////横移動//////
    if(!horizontal_move_end){
      state = MAXSPEED;

      if(zone == RED){
	linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,D_RIGHT);
      }else{
	linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,D_LEFT);
      }
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;

      linetrace_duty = linetrace_duty_decide(linetrace_state,linetrace_duty);
  
      if(line_count < over_line){

	if(abs(x_encoder) >= STARTZONE_LINE_VALUE){
	  if(linetrace_state == CURRENT_DATA){
	    out_line_count = 0;
	    on_line_count++;
	    if(on_line_count >= 8 && !line_flag){
	      line_count++;
	      on_line_count = 0;
	      line_flag = true;
	    }
	  }
	  if(linetrace_state == RECENT_DATA){
	    on_line_count = 0;
	    out_line_count++;
	    if(out_line_count >= 8){
	      out_line_count = 0;
	      line_flag = false;
	    }
	  }
	}

	if(!encoder_move_end){
	  if(zone == RED){
	    sus_state = cal_omni_value(LEFT, MOVE_CONTINUE, target_encoder_value, x_encoder, 0, MOVELEFT_BACK_DUTY, MOVESIDE_SPIN_MAX_DUTY);
	  }else{
	    sus_state = cal_omni_value(RIGHT, MOVE_CONTINUE, target_encoder_value, x_encoder, 0, MOVERIGHT_BACK_DUTY, -MOVESIDE_SPIN_MAX_DUTY);
	  }
	  if(sus_state == MOVE_END){
	    encoder_move_end = true;
	  }
	}else{
	  if(zone == RED){
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVELEFT_MIN_DUTY, MOVELEFT_BACK_DUTY, MOVESIDE_SPIN_MIN_DUTY);
	  }else{
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVERIGHT_MIN_DUTY, MOVERIGHT_BACK_DUTY, -MOVESIDE_SPIN_MIN_DUTY);
	  }
	}

      }else{
	on_line_count = 0;
	out_line_count = 0;
	line_count = 0;
	line_flag = false;
	encoder_move_end = false;
	horizontal_move_end = true;
	sus_state = MAXSPEED;
	find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT);
	DD_encoder1reset();
	DD_encoder2reset();
      }
    }

    ////縦移動////
    if(horizontal_move_end && !vartical_move_end){
      state = MAXSPEED;
      
      linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,NO_LINE);
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;
      
      linetrace_duty = linetrace_duty_decide(linetrace_state,linetrace_duty);

      if(_IS_PRESSED_FRONT_FOOTSW()){
	DD_encoder1reset();
	DD_encoder2reset();
	vartical_move_end = true;
      }else{

	if(!encoder_move_end){
	  switch(robot_direction.direction){
	  case D_RIGHT:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall, y_encoder, linetrace_duty, 0, 0);
	    break;
	  case D_LEFT:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall, y_encoder, -linetrace_duty, 0, 0);
	    break;
	  case D_FRONT_RIGHT:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall, y_encoder, 0, 0, -linetrace_duty);
	    break;
	  case D_FRONT_LEFT:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall, y_encoder, 0, 0, linetrace_duty);
	    break;
	  case D_MIDDLE:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall, y_encoder, 0, 0, 0);
	    break;
	  case NO_LINE:
	    sus_state = cal_omni_value(FRONT, MOVE_CONTINUE, distance_to_wall, y_encoder, 0, 0, 0);
	    break;
	  }
	  if(sus_state == MOVE_END){
	    encoder_move_end = true;
	  }
	  
	}else{
	  
	  switch(robot_direction.direction){
	  case D_RIGHT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, linetrace_duty, MOVEFRONT_MIN_DUTY, 0);
	    break;
	  case D_LEFT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -linetrace_duty, MOVEFRONT_MIN_DUTY, 0);
	    break;
	  case D_FRONT_RIGHT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, -linetrace_duty);
	    break;
	  case D_FRONT_LEFT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, linetrace_duty);
	    break;
	  case D_MIDDLE:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, 0);
	    break;
	  case NO_LINE:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, 0);
	    break;
	  }
	  
	}
      }
    }

    if(horizontal_move_end && vartical_move_end){
      linetrace_state = RESET_DATA;
      linetrace_duty = 0;
      line_count = 0;
      line_flag = false;
      x_encoder = 0;
      y_encoder = 0;
      over_line = 0;
      suspension_stop();
      DD_encoder1reset();
      DD_encoder2reset();
      first_flag = true;
      state = MOVE_END;
      encoder_move_end = false;
      sus_state = MAXSPEED;
      find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT);
      horizontal_move_end = false;
      vartical_move_end = false;
    }
    break;


  case D_S:
    if(!vartical_move_end){
      state = MAXSPEED;
      if(_IS_PRESSED_BACK_FOOTSW()){
	vartical_move_end = true;
      }else{
	if(!encoder_move_end){
	  if(zone == RED){
	    sus_state = cal_omni_value(BACK, MOVE_CONTINUE, -deskd_y_enc_tar, y_encoder, MOVE_DESKD_RIGHT_MAX_DUTY, 0, -MOVESIDE_SPIN_MAX_DUTY);
	  }else{
	    sus_state = cal_omni_value(BACK, MOVE_CONTINUE, -deskd_y_enc_tar, y_encoder, MOVE_DESKD_LEFT_MAX_DUTY, 0, MOVESIDE_SPIN_MAX_DUTY);
	  }
	  if(sus_state == MOVE_END){
	    encoder_move_end = true;
	  }
	}else{
	  if(zone == RED){
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVE_DESKD_RIGHT_MIN_DUTY, MOVE_DESKD_BACK_MIN_DUTY, -MOVESIDE_SPIN_MIN_DUTY);
	  }else{
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVE_DESKD_LEFT_MIN_DUTY, MOVE_DESKD_BACK_MIN_DUTY, MOVESIDE_SPIN_MIN_DUTY);
	  }
	}
      }
    }else if(vartical_move_end && !horizontal_move_end){
      if(abs(x_encoder) <= MOVERETURN_LINE_D_VALUE){
	if(zone == RED){
	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVE_DESKD_RIGHT_MIN_DUTY, 0, -MOVESIDE_SPIN_MIN_DUTY);
	}else{
	  sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, MOVE_DESKD_LEFT_MIN_DUTY, 0, MOVESIDE_SPIN_MIN_DUTY);
	}
      }else{
	all_motor_stop();
	horizontal_move_end = true;
      }
    }else if(vartical_move_end && horizontal_move_end){
      linetrace_state = RESET_DATA;
      linetrace_duty = 0;
      line_count = 0;
      line_flag = false;
      x_encoder = 0;
      y_encoder = 0;
      over_line = 0;
      suspension_stop();
      DD_encoder1reset();
      DD_encoder2reset();
      first_flag = true;
      state = MOVE_END;
      encoder_move_end = false;
      sus_state = MAXSPEED;
      find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT);
      horizontal_move_end = false;
      vartical_move_end = false;
    }
    break;
  case A_S:
  case B_S:
  case C_S:

    if(t_t == A_S){
      target_encoder_value = MOVERETURN_LINE_A_VALUE;
      over_line = 4;
    }else if(t_t == B_S){
      target_encoder_value = MOVERETURN_LINE_B_VALUE;
      over_line = 3;
    }else if(t_t == C_S){
      target_encoder_value = MOVERETURN_LINE_C_VALUE;
      over_line = 2;
    }else if(t_t == D_S){
      target_encoder_value = MOVERETURN_LINE_D_VALUE;
      over_line = 1;
    }

    if(zone == RED){
      target_encoder_value = target_encoder_value * -1;
    }
    
    if(!vartical_move_end){
      state = MAXSPEED;
	
      linetrace_state = find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,0,NO_LINE);
      linetrace_duty = robot_direction.gap_degree+SUSPENSION_STOP_DUTY;

      linetrace_duty = linetrace_duty_decide(linetrace_state,linetrace_duty);

      if(_IS_PRESSED_BACK_FOOTSW()){
	DD_encoder1reset();
	DD_encoder2reset();
	vartical_move_end = true;
      }else{

	if(!encoder_move_end){
	  switch(robot_direction.direction){
	  case D_RIGHT:
	    sus_state = cal_omni_value(BACK, MOVE_CONTINUE, -distance_to_wall, y_encoder, linetrace_duty, 0, 0);
	    break;
	  case D_LEFT:
	    sus_state = cal_omni_value(BACK, MOVE_CONTINUE, -distance_to_wall, y_encoder, -linetrace_duty, 0, 0);
	    break;
	  case D_FRONT_RIGHT:
	    sus_state = cal_omni_value(BACK, MOVE_CONTINUE, -distance_to_wall, y_encoder, 0, 0, -linetrace_duty);
	    break;
	  case D_FRONT_LEFT:
	    sus_state = cal_omni_value(BACK, MOVE_CONTINUE, -distance_to_wall, y_encoder, 0, 0, linetrace_duty);
	    break;
	  case D_MIDDLE:
	    sus_state = cal_omni_value(BACK, MOVE_CONTINUE, -distance_to_wall, y_encoder, 0, 0, 0);
	    break;
	  case NO_LINE:
	    sus_state = cal_omni_value(BACK, MOVE_CONTINUE, -distance_to_wall, y_encoder, 0, 0, 0);
	    break;
	  }
	  if(sus_state == MOVE_END){
	    encoder_move_end = true;
	  }
	  
	}else{
	  
	  switch(robot_direction.direction){
	  case D_RIGHT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, linetrace_duty, MOVEBACK_MIN_DUTY, 0);
	    break;
	  case D_LEFT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, -linetrace_duty, MOVEBACK_MIN_DUTY, 0);
	    break;
	  case D_FRONT_RIGHT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEBACK_MIN_DUTY, -linetrace_duty);
	    break;
	  case D_FRONT_LEFT:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEBACK_MIN_DUTY, linetrace_duty);
	    break;
	  case D_MIDDLE:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEBACK_MIN_DUTY, 0);
	    break;
	  case NO_LINE:
	    sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEBACK_MIN_DUTY, 0);
	    break;
	  }	  
	}
      }      
    }

    if(vartical_move_end && !horizontal_move_end){
      state = MAXSPEED;

      if(zone == RED){
	sus_state = cal_omni_value(RIGHT, MOVE_CONTINUE, -target_encoder_value, x_encoder, 0, MOVERIGHT_BACK_DUTY, -MOVESIDE_SPIN_MAX_DUTY);
      }else{
	sus_state = cal_omni_value(LEFT, MOVE_CONTINUE, -target_encoder_value, x_encoder, 0, MOVELEFT_BACK_DUTY, MOVESIDE_SPIN_MAX_DUTY);
      }
      if(sus_state == MOVE_END){
	horizontal_move_end = true;
      }
    }

    if(vartical_move_end && horizontal_move_end){
      linetrace_state = RESET_DATA;
      linetrace_duty = 0;
      line_count = 0;
      line_flag = false;
      x_encoder = 0;
      y_encoder = 0;
      over_line = 0;
      suspension_stop();
      DD_encoder1reset();
      DD_encoder2reset();
      first_flag = true;
      state = MOVE_END;
      encoder_move_end = false;
      sus_state = MAXSPEED;
      find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT);
      horizontal_move_end = false;
      vartical_move_end = false;
    }
    
    break;
  case S_E:
    if(!vartical_move_end){
      state = MAXSPEED;
      
      if(_IS_PRESSED_FRONT_FOOTSW()){
	DD_encoder1reset();
	DD_encoder2reset();
	vartical_move_end = true;
      }else{
	sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEFRONT_MIN_DUTY, 0);
      }
    }else if(vartical_move_end){
      linetrace_state = RESET_DATA;
      linetrace_duty = 0;
      line_count = 0;
      line_flag = false;
      x_encoder = 0;
      y_encoder = 0;
      over_line = 0;
      suspension_stop();
      DD_encoder1reset();
      DD_encoder2reset();
      first_flag = true;
      state = MOVE_END;
      encoder_move_end = false;
      sus_state = MAXSPEED;
      find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT);
      horizontal_move_end = false;
      vartical_move_end = false;
    }
    break;
    
  case E_S:
    if(!vartical_move_end){
      state = MAXSPEED;
      
      if(_IS_PRESSED_BACK_FOOTSW()){
	DD_encoder1reset();
	DD_encoder2reset();
	vartical_move_end = true;
      }else{
	sus_state = cal_omni_value(NOTHING, MOVE_CONTINUE, 0, 0, 0, MOVEBACK_MIN_DUTY, 0);
      }
    }else if(vartical_move_end){
      linetrace_state = RESET_DATA;
      linetrace_duty = 0;
      line_count = 0;
      line_flag = false;
      x_encoder = 0;
      y_encoder = 0;
      over_line = 0;
      suspension_stop();
      DD_encoder1reset();
      DD_encoder2reset();
      first_flag = true;
      state = MOVE_END;
      encoder_move_end = false;
      sus_state = MAXSPEED;
      find_robotdirection(g_ss_h[PHOTOARRAY_FRONT].data,g_ss_h[PHOTOARRAY_BEHIND].data,&robot_direction,1,D_RIGHT);
      horizontal_move_end = false;
      vartical_move_end = false;
    }
    break;
  }
  
  return state;
}

static
int linetrace_duty_decide(LineTrace_State_t state, int cal_duty){
  int linetrace_duty = 0;

  if(state == CURRENT_DATA){
    if(cal_duty <= 16){
      linetrace_duty = 1;
    }else if(cal_duty >= 30){
      linetrace_duty = 16;
    }else if(cal_duty >= 17){
      linetrace_duty = 14;
    }
  }else if(state == RECENT_DATA){
    if(cal_duty <= 11){
      linetrace_duty = 5;
    }else if(cal_duty >= 12){
      linetrace_duty = 17;
    }
  }

  return linetrace_duty;
}

static
int manual_suspensionsystem(void){
  const tc_const_t suspension_tcon = {
    .inc_con = 150,
    .dec_con = 250
  };
  const int num_of_motor = 4;/*モータの個数*/
  int matrix[4][3] = {{100,100,100},
                      {-100,100,100},
                      {-100,-100,100},
                      {100,-100,100}};
  int input[3],hold_value[4][3],result[4];
  unsigned int idx;/*インデックス*/
  int i = 0,j = 0;
  int max = 0;
  int power = 0;
  int func = 0;
  int enhance = 0;

  int target = 0;
  
  //中心補正値よりコントローラからの値が小さければ大きさは0、その他は値を代入

  //アナログパッドLのx値
  if(abs( DD_RCGetLX(g_rc_data))<CENTRAL_THRESHOLD){
    input[0] = 0;
  }else{
    input[0] = -DD_RCGetLX(g_rc_data);   
  }
  
  //アナログパッドLのy値(入力値が反転しているためマイナスをかけてある)
  if(abs( DD_RCGetLY(g_rc_data))<CENTRAL_THRESHOLD){
    input[1] = 0;
  }else{
    input[1] = DD_RCGetLY(g_rc_data);
  }

  //アナログパッドRのx値
  if(abs( DD_RCGetRX(g_rc_data))<CENTRAL_THRESHOLD){
    input[2] = 0;
  }else{
    input[2] = -DD_RCGetRX(g_rc_data);   
  }


  //デューティのエンハンス値を計算
  if( cal_abs(input[0],input[1]) >= abs(input[2])){
    enhance = cal_abs(input[0],input[1]);
  }else{
    enhance = abs(input[2]);
  }

  //デューティ最大値(power)を計算
  func = MANUAL_SUS_MAX_DUTY/CONT_ABS_MAX;
  power = MANUAL_SUS_MAX_DUTY - func*(abs(CONT_ABS_MAX - enhance));
  
  //行列を計算
  for(i=0;i<4;i++){
    for(j=0;j<3;j++){
      hold_value[i][j] = matrix[i][j] * input[j]; 
    }
    result[i] = hold_value[i][0] + hold_value[i][1] + hold_value[i][2];
  }

  //計算値から最大値を求め、デューティ最大値に近づける
  max = abs(result[0]);
  for(i=1;i<4;i++){
    if(max<=abs(result[i])){
      max = abs(result[i]);
    }
  }
  for(i=0;i<4;i++){
    result[i] = result[i] * power/max;
  }
  
  /*for each motor*/
  for(i=0;i<num_of_motor;i++){
    /*それぞれの差分*/
    switch(i){
    case 0:
      idx = KUDO_LF;
      break;
    case 1:
      idx = KUDO_LB;
      break;
    case 2:
      idx = KUDO_RB;
      break;
    case 3:
      idx = KUDO_RF;
      break;
    default:
      return EXIT_FAILURE;
    }

    if(idx == KUDO_LF){
      if(abs(result[i]) > 0){
	result[i] = result[i] + result[i]/20;
	/* if(result[i] > 0){ */
	/*   result[i]+=300; */
	/* }else if(result[i] < 0){ */
	/*   result[i]-=300; */
	/* } */
      } 
    }

    if(idx == KUDO_RF){
      result[i] = result[i] + result[i]/15;
    }
    
    if(abs(input[0])<CENTRAL_THRESHOLD && abs(input[1])<CENTRAL_THRESHOLD
       && abs(input[2])<CENTRAL_THRESHOLD ){
      target = 0;
      trapezoidCtrl(target, &g_md_h[idx], &suspension_tcon);
    }else if(result[i]>0){
      trapezoidCtrl(-abs(result[i]), &g_md_h[idx], &suspension_tcon);
    }else if(result[i]<0){
      trapezoidCtrl(abs(result[i]), &g_md_h[idx], &suspension_tcon);
    }
  }

  return EXIT_SUCCESS;
}

int cal_abs(int x,int y){

  int absolute = 0;
  double d_x,d_y;

  d_x = (double)x;
  d_y = (double)y;

  absolute =(int)(cal_root(d_x*d_x + d_y*d_y));

  return abs(absolute);
}


/*与えられた値の平方根を計算する
  math.h の sqrt()相当*/
double cal_root(double s){

  double x = s/2.0;
  double last_x = 0.0;

  while(x!=last_x){
    last_x = x;
    x = (x + s/x)/2.0;
  }

  return x; 
}

static
int32_t I2C_Encoder(int32_t encoder_num, EncoderOperation_t operation){
  int32_t value=0,temp_value=0;
  static int32_t adjust[4] = {0,0,0,0};

  switch(encoder_num){
  case 0:
    temp_value = g_ss_h[I2C_ENCODER_1].data[0] + (g_ss_h[I2C_ENCODER_1].data[1] << 8) + (g_ss_h[I2C_ENCODER_1].data[2] << 16) + (g_ss_h[I2C_ENCODER_1].data[3] << 24);
    value = temp_value + adjust[0];
    if(operation == GET_ENCODER_VALUE){
      break;
    }else if(operation == RESET_ENCODER_VALUE){
      adjust[0] = -temp_value;
      value = temp_value + adjust[0];
    }
    break;
    
  case 1:
    temp_value = g_ss_h[I2C_ENCODER_1].data[4] + (g_ss_h[I2C_ENCODER_1].data[5] << 8) + (g_ss_h[I2C_ENCODER_1].data[6] << 16) + (g_ss_h[I2C_ENCODER_1].data[7] << 24);
    value = temp_value + adjust[1];
    if(operation == GET_ENCODER_VALUE){
      break;
    }else if(operation == RESET_ENCODER_VALUE){
      adjust[1] = -temp_value;
      value = temp_value + adjust[1];
    }
    break;
    
  case 2:
    temp_value = g_ss_h[I2C_ENCODER_2].data[0] + (g_ss_h[I2C_ENCODER_2].data[1] << 8) + (g_ss_h[I2C_ENCODER_2].data[2] << 16) + (g_ss_h[I2C_ENCODER_2].data[3] << 24);
    value = temp_value + adjust[2];
    if(operation == GET_ENCODER_VALUE){
      break;
    }else if(operation == RESET_ENCODER_VALUE){
      adjust[2] = -temp_value;
      value = temp_value + adjust[2];
    }
    break;
    
  case 3:
    temp_value = g_ss_h[I2C_ENCODER_2].data[4] + (g_ss_h[I2C_ENCODER_2].data[5] << 8) + (g_ss_h[I2C_ENCODER_2].data[6] << 16) + (g_ss_h[I2C_ENCODER_2].data[7] << 24);
    value = temp_value + adjust[3];
    if(operation == GET_ENCODER_VALUE){
      break;
    }else if(operation == RESET_ENCODER_VALUE){
      adjust[3] = -temp_value;
      value = temp_value + adjust[3];
    }
    break;
  }
  
  return value;
}
