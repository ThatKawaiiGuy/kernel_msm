/* Lge_touch_core.c
 *
 * Copyright (C) 2011 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/jiffies.h>
#include <linux/sysdev.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/cpufreq.h>
#include <linux/hotplug.h>
#include <linux/cpu.h>

#include <linux/input/lge_touch_core.h>

#ifdef CONFIG_TOUCHSCREEN_PREVENT_SLEEP
#ifdef CONFIG_TOUCHSCREEN_SWEEP2WAKE
#include <linux/input/sweep2wake.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
#include <linux/input/doubletap2wake.h>
#endif
#endif

#ifdef CUST_G_TOUCH
#include "./DS4/RefCode.h"
#include "./DS4/RefCode_PDTScan.h"
struct i2c_client *ds4_i2c_client;
static int f54_fullrawcap_mode = 0;

#if defined(CONFIG_MACH_APQ8064_GK_KR) || defined(CONFIG_MACH_APQ8064_GKATT) || defined(CONFIG_MACH_APQ8064_GVDCM) || defined(CONFIG_MACH_APQ8064_J1D) || defined(CONFIG_MACH_APQ8064_J1KD)
// do nothing
#else
#define G_ONLY
#endif


#endif

struct lge_touch_data
{
	void*			h_touch;
	atomic_t		next_work;
	atomic_t		device_init;
	u8				work_sync_err_cnt;
	u8				ic_init_err_cnt;
	volatile int	curr_pwr_state;
	int				int_pin_state;
	struct i2c_client 			*client;
	struct input_dev 			*input_dev;
	struct hrtimer 				timer;
	struct work_struct  		work;
	struct delayed_work			work_init;
	struct delayed_work			work_touch_lock;
	struct work_struct  		work_fw_upgrade;
	struct early_suspend		early_suspend;
	struct touch_platform_data 	*pdata;
	struct touch_data			ts_data;
	struct touch_fw_info		fw_info;
	struct section_info			st_info;
	struct kobject 				lge_touch_kobj;
	struct ghost_finger_ctrl	gf_ctrl;
	struct jitter_filter_info	jitter_filter;
	struct accuracy_filter_info	accuracy_filter;
};

struct touch_device_driver*	touch_device_func;
struct workqueue_struct*	touch_wq;

struct lge_touch_attribute {
	struct attribute attr;
	ssize_t (*show)(struct lge_touch_data *ts, char *buf);
	ssize_t (*store)(struct lge_touch_data *ts,
				const char *buf, size_t count);
};

static int is_pressure;
static int is_width_major;
static int is_width_minor;

/* extern vars */
struct lge_touch_data *_ts;

bool suspended = false;

bool doubletap_to_wake = false;
module_param(doubletap_to_wake, bool, 0664);

#define LGE_TOUCH_ATTR(_name, _mode, _show, _store)               \
	struct lge_touch_attribute lge_touch_attr_##_name =       \
	__ATTR(_name, _mode, _show, _store)

/* Debug mask value
 * usage: echo [debug_mask] > /sys/module/lge_touch_core/parameters/debug_mask
 */
u32 touch_debug_mask = DEBUG_BASE_INFO;
module_param_named(debug_mask, touch_debug_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);

#ifdef LGE_TOUCH_TIME_DEBUG
/* Debug mask value
 * usage: echo [debug_mask] > /sys/module/lge_touch_core/parameters/time_debug_mask
 */
u32 touch_time_debug_mask = DEBUG_TIME_PROFILE_NONE;
module_param_named(time_debug_mask, touch_time_debug_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);

#define get_time_interval(a,b) ((a)>=(b) ? (a)-(b) : 1000000+(a)-(b))
struct timeval t_debug[TIME_PROFILE_MAX];
#endif

#ifdef CUST_G_TOUCH
#define get_time_interval(a,b) a>=b ? a-b : 1000000+a-b
static u8 resume_flag = 0;
static unsigned int ta_debouncing_count = 0;
static unsigned int button_press_count =0;
static unsigned int ts_rebase_count =0;
struct timeval t_ex_debug[TIME_EX_PROFILE_MAX];
struct t_data	 ts_prev_finger_press_data;
int force_continuous_mode = 0;
int long_press_check_count = 0;
int long_press_check = 0;
int finger_subtraction_check_count = 0;
int ghost_detection = 0;
int ghost_detection_count = 0;

#endif

#define MAX_RETRY_COUNT         3

#ifdef LGE_TOUCH_POINT_DEBUG
#define MAX_TRACE	500
struct pointer_trace {
	int	x;
	int	y;
	s64	time;
};

static struct pointer_trace tr_data[MAX_TRACE];
static int tr_last_index;
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void touch_early_suspend(struct early_suspend *h);
static void touch_late_resume(struct early_suspend *h);
#endif

/* Auto Test interface for some model */
struct lge_touch_data *touch_test_dev = NULL;
EXPORT_SYMBOL(touch_test_dev);
#ifdef CUST_G_TOUCH
static void release_all_ts_event(struct lge_touch_data *ts);
/* Jitter Filter
 *
 */
#define jitter_abs(x)		(x > 0 ? x : -x)
#define jitter_sub(x, y)	(x > y ? x - y : y - x)

int trigger_baseline = 0;
int ts_charger_plug = 0;
int ts_charger_type = 0;
static void safety_reset(struct lge_touch_data *ts);
static int touch_ic_init(struct lge_touch_data *ts);
#ifdef G_ONLY
int cur_hopping_idx = 3;
extern int cns_en;
#endif
static struct hrtimer hr_touch_trigger_timer;
#define MS_TO_NS(x)	(x * 1E6L)

static enum hrtimer_restart touch_trigger_timer_handler(struct hrtimer *timer)
{
	if (touch_test_dev && touch_test_dev->pdata->role->ghost_detection_enable) {
		if(trigger_baseline==1 && atomic_read(&touch_test_dev->device_init) == 1)
		{
			trigger_baseline = 2;
			atomic_inc(&touch_test_dev->next_work);
			queue_work(touch_wq, &touch_test_dev->work);
		}
	}
	return HRTIMER_NORESTART;
}

void trigger_baseline_state_machine(int plug_in, int type)
{
	u8 buf=0;

#ifdef G_ONLY
	extern u8 hopping;
#endif

	if (touch_test_dev && touch_test_dev->pdata->role->ghost_detection_enable) {

	    if(plug_in == 0 || plug_in == 1)
	    {
			if(touch_test_dev->curr_pwr_state == POWER_ON) {
				if(plug_in ==0){
					touch_i2c_read(touch_test_dev->client, 0x50, 1, &buf);
					buf = buf & 0xDF;
					touch_i2c_write_byte(touch_test_dev->client, 0x50, buf);
#ifdef G_ONLY
					cns_en = 0;
					if(cur_hopping_idx != 3) cur_hopping_idx = 3;
					safety_reset(touch_test_dev);
					queue_delayed_work(touch_wq, &touch_test_dev->work_init,
								msecs_to_jiffies(touch_test_dev->pdata->role->booting_delay));
					TOUCH_INFO_MSG("cur_hopping_idx [ %s ] = %x\n", __func__, cur_hopping_idx);
#endif
				} else if(plug_in ==1){
					touch_i2c_read(touch_test_dev->client, 0x50, 1, &buf);
					buf = buf | 0x20;
					touch_i2c_write_byte(touch_test_dev->client, 0x50, buf);
#ifdef G_ONLY
					touch_i2c_write_byte(touch_test_dev->client, 0xFF, 0x01);
					touch_i2c_read(touch_test_dev->client, 0x0D, 1, &buf);

					if( buf >= 1 ) {
						switch(type) {
							case 0:
							case 1:
								if( cur_hopping_idx != 4 ) {
									//touch_i2c_write_byte(touch_test_dev->client, 0x04, 0x84);

									cur_hopping_idx = 4;
									hopping = 1;
									TOUCH_INFO_MSG("cur_hopping_idx [ %s ] = %x\n", __func__, cur_hopping_idx);
								} else {
									hopping = 0;
								}
								break;
							default:
								break;
						}
					}
					touch_i2c_write_byte(touch_test_dev->client, 0xFF, 0x00);
#endif
				}
			}
			
			ts_charger_type = type;
			TOUCH_INFO_MSG(" trigger_baseline_state_machine = %d type = %d \n", plug_in, type);
			ts_charger_plug = plug_in;

#ifdef G_ONLY
			if( trigger_baseline==0 && plug_in ==1){
#else
			if( trigger_baseline==0 ){
#endif
				trigger_baseline = 1;

				hrtimer_start(&hr_touch_trigger_timer, ktime_set(0, MS_TO_NS(1000)), HRTIMER_MODE_REL);
			}
		}
	}
}

#ifdef PRESSURE_DIFF
/* pressure_diff_detection  */
/* if two fingers pressed, compare their pressure and then if  the difference in pressure between the two fingers is over 10,
    run baseline rebase algorithm when finger released.  */
int pressure_diff_detection(struct lge_touch_data *ts)
{
	int id = 0;
	int z_less_30 = 0;
	int z_more_30 = 0;

	for(id=0; id < ts->pdata->caps->max_id; id++){
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED){
			if (unlikely(touch_debug_mask & DEBUG_PRESSURE)){
				TOUCH_INFO_MSG("[z_diff] z30_set_check[ %d] id[ %d] z_curr[ %d] pre_state[ %s] cur_state[ %s]\n",
					ts->pressure_diff.z30_set_check, id, ts->ts_data.curr_data[id].pressure, ts->ts_data.prev_data[id].status?"PRESSED":"RELEASED",ts->ts_data.curr_data[id].status?"PRESSED":"RELEASED");
			}

			/* pressure 30 but pressure 30 is already pressed one time  */
			if (ts->pressure_diff.z30_set_check && ts->ts_data.curr_data[id].pressure == 30 && ts->ts_data.prev_data[id].status == FINGER_RELEASED){
				memset(&ts->pressure_diff, 0, sizeof(ts->pressure_diff));
				ts->pressure_diff.z30_id = -1;
				ts->pressure_diff.z_more30_id= -1;
				TOUCH_INFO_MSG("[z_diff] pressure 30 is already pressed , set z_diff count 0 \n");
				return false;
			}

			/* pressure 30  ( just enter once )*/
			if (!ts->pressure_diff.z30_set_check && ts->ts_data.curr_data[id].pressure == 30 && ts->ts_data.prev_data[id].status == FINGER_PRESSED){
				ts->pressure_diff.z30_set_check = true;
				ts->pressure_diff.z30_id = id;
				TOUCH_INFO_MSG("[z_diff] pressure 30 is pressed 1st, set z30_id[ %d]\n", ts->pressure_diff.z30_id);
			}

			/* finger pressure */
			if (ts->pressure_diff.z30_set_check  && ts->ts_data.curr_data[id].pressure > 30 && ts->ts_data.prev_data[id].status == FINGER_RELEASED){
				z_more_30 = ts->ts_data.curr_data[id].pressure;
				ts->pressure_diff.z_more30_id = id;
				TOUCH_INFO_MSG("[z_diff] pressure[ %d] is pressed, set z_more30_id[ %d]\n", z_more_30, ts->pressure_diff.z_more30_id);
			}
		}

		if (unlikely(touch_debug_mask & DEBUG_PRESSURE))
			TOUCH_INFO_MSG("[z_diff] z30_set_check[ %d] z_more_30[ %d]\n", ts->pressure_diff.z30_set_check, z_more_30);

		if(ts->pressure_diff.z30_set_check && z_more_30){
			break;
		}
	}

	if(ts->pressure_diff.z30_id > -1){
		 /* if position between first_press position and curr position is under 5, set pen or ghost pressure (30) */
		if(ABS_DIFF_Z(ts->pressure_diff.z30_x_pos_1st - ts->ts_data.curr_data[ts->pressure_diff.z30_id].x_position) < 5 &&
			ABS_DIFF_Z(ts->pressure_diff.z30_y_pos_1st - ts->ts_data.curr_data[ts->pressure_diff.z30_id].y_position) < 5){
				z_less_30 = ts->ts_data.curr_data[ts->pressure_diff.z30_id].pressure;
				TOUCH_INFO_MSG("[z_diff] remain the same position [%d][%d] pressure [%d][%d]\n",
					ts->pressure_diff.z30_x_pos_1st, ts->pressure_diff.z30_y_pos_1st, z_less_30, z_more_30);
		}else{
			TOUCH_INFO_MSG("[z_diff] press position [%d][%d] current position[%d][%d]\n",
				ts->pressure_diff.z30_x_pos_1st, ts->pressure_diff.z30_y_pos_1st,
				ts->ts_data.curr_data[ts->pressure_diff.z30_id].x_position,ts->ts_data.curr_data[ts->pressure_diff.z30_id].y_position);
			ts->pressure_diff.z30_x_pos_1st = ts->ts_data.curr_data[ts->pressure_diff.z30_id].x_position;
			ts->pressure_diff.z30_y_pos_1st = ts->ts_data.curr_data[ts->pressure_diff.z30_id].y_position;
			ts->pressure_diff.z_more30_id = -1;
			ts->pressure_diff.z_diff_cnt = 0;
			return false;
		}
	}

	if((z_less_30 == 30) && (z_more_30 > 30)){
		 /* if pressure between the two fingers is over 10, add pressure different count. if pressure different count is more than 2, we need to rebase. */
		if((z_more_30 - z_less_30) > 10){
			ts->pressure_diff.z_diff_cnt++;
			TOUCH_INFO_MSG("[z_diff] z_diff_cnt[ %d]\n", ts->pressure_diff.z_diff_cnt);
			if(ts->pressure_diff.z_diff_cnt > 1){
				return true;
			}
		}
	}

	return false;
}
#endif
int ghost_detect_solution(struct lge_touch_data *ts)
{
	extern u8 pressure_zero;
#ifdef G_ONLY
	extern u8 hopping;
#endif
	int first_int_detection = 0;
	int cnt = 0, id =0;

	if(ts->gf_ctrl.incoming_call && (ts->ts_data.total_num > 1)) {
		TOUCH_INFO_MSG("call state rebase\n");
		goto out_need_to_rebase;
	}

	if(trigger_baseline==2) 
		goto out_need_to_rebase;
	
	if(resume_flag) {
		resume_flag = 0;
		do_gettimeofday(&t_ex_debug[TIME_EX_FIRST_INT_TIME]);

		if( t_ex_debug[TIME_EX_FIRST_INT_TIME].tv_sec - t_ex_debug[TIME_EX_INIT_TIME].tv_sec == 0 ) {
			if((get_time_interval(t_ex_debug[TIME_EX_FIRST_INT_TIME].tv_usec,
			                   t_ex_debug[TIME_EX_INIT_TIME].tv_usec)) <= 200000) first_int_detection= 1;
		} else if( t_ex_debug[TIME_EX_FIRST_INT_TIME].tv_sec - t_ex_debug[TIME_EX_INIT_TIME].tv_sec == 1 ) {
			if( t_ex_debug[TIME_EX_FIRST_INT_TIME].tv_usec + 1000000
				- t_ex_debug[TIME_EX_INIT_TIME].tv_usec <= 200000) {
				first_int_detection = 1;
			}
		}
	}

	if(first_int_detection) {
		for (cnt = 0; cnt < ts->pdata->caps->max_id; cnt++) {
			if (ts->ts_data.curr_data[cnt].status == FINGER_PRESSED) {
					TOUCH_INFO_MSG("first input time is 200ms\n");
					ghost_detection = true;
			}
		}
	}

	if(pressure_zero == 1) {
		TOUCH_INFO_MSG("pressure\n");
		ghost_detection = true;
	}

#ifdef G_ONLY
	/*if(hopping == 1) {
		TOUCH_INFO_MSG("hopping\n");
		ghost_detection = true;
	}*/
#endif
	if (ts_charger_plug) {
		if( (ts->pdata->role->ta_debouncing_finger_num  <= ts->ts_data.total_num) && ( ta_debouncing_count < ts->pdata->role->ta_debouncing_count)) {
			ta_debouncing_count ++;
			memset(&ts->ts_data.curr_data, 0x0, sizeof(ts->ts_data.curr_data));
			goto out_need_to_debounce;
		} else if (ts->ts_data.total_num < ts->pdata->role->ta_debouncing_finger_num) {
			ta_debouncing_count = 0;
		} else ;
	}

#ifdef PRESSURE_DIFF
	if((ts->gf_ctrl.stage != 0x0b) && (ts->ts_data.total_num == 2) && (ts->ts_data.prev_total_num < ts->ts_data.total_num) && (ts->pressure_diff.z_more30_id == -1)){
		if(pressure_diff_detection(ts)){
			if (unlikely(touch_debug_mask & DEBUG_PRESSURE))
				TOUCH_INFO_MSG("[z_diff] pressure is different %d times. run rebase when finger released.\n", ts->pressure_diff.z_diff_cnt);
			ts->pressure_diff.ghost_diff_detection = true;
		}
	}
#endif
	if ((ts->ts_data.state != TOUCH_ABS_LOCK) &&(ts->ts_data.total_num)){

		if (ts->ts_data.prev_total_num != ts->ts_data.total_num)
		{
			if (ts->ts_data.prev_total_num <= ts->ts_data.total_num) 
			{
			       if (ts->gf_ctrl.stage == GHOST_STAGE_CLEAR || (ts->gf_ctrl.stage | GHOST_STAGE_1) || ts->gf_ctrl.stage == GHOST_STAGE_4)
					ts->ts_data.state = TOUCH_BUTTON_LOCK;

				for(id=0; id < ts->pdata->caps->max_id; id++){
					if (ts->ts_data.curr_data[id].status == FINGER_PRESSED
							&& ts->ts_data.prev_data[id].status == FINGER_RELEASED) {
						break;
					}
				}


#ifdef PRESSURE_DIFF
				if(ts->ts_data.total_num == 1 && ts->ts_data.curr_data[id].pressure == 30){
					ts->pressure_diff.z30_x_pos_1st = ts->ts_data.curr_data[id].x_position;
					ts->pressure_diff.z30_y_pos_1st = ts->ts_data.curr_data[id].y_position;
				}
#endif
				if ( id < 10) 
				{
					memcpy(&t_ex_debug[TIME_EX_PREV_PRESS_TIME], &t_ex_debug[TIME_EX_CURR_PRESS_TIME], sizeof(struct timeval));
					do_gettimeofday(&t_ex_debug[TIME_EX_CURR_PRESS_TIME]);

					if ( 1<= ts->ts_data.prev_total_num && 1<= ts->ts_data.total_num && jitter_sub(ts_prev_finger_press_data.x_position,ts->ts_data.curr_data[id].x_position)<=10 && jitter_sub(ts_prev_finger_press_data.y_position,ts->ts_data.curr_data[id].y_position)<=10 )
					{
					       // if time_interval between prev fingger pressed and curr finger pressed is less than 50ms, we need to rebase touch ic.
						if(((t_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_sec - t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==1) &&
							(( get_time_interval(t_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_usec+1000000, t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) <= 50*1000))
						{
							ghost_detection = true;
							ghost_detection_count++;
						}
						else if(((t_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_sec - t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==0) &&
							(( get_time_interval(t_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_usec, t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) <= 50*1000))
						{
							ghost_detection = true;
							ghost_detection_count++;
						}
						else	; // do not anything
					}
					else if (ts->ts_data.prev_total_num==0 && ts->ts_data.total_num==1 && jitter_sub(ts_prev_finger_press_data.x_position,ts->ts_data.curr_data[id].x_position)<=10 && jitter_sub(ts_prev_finger_press_data.y_position,ts->ts_data.curr_data[id].y_position)<=10 )
					{
					       // if time_interval between prev fingger pressed and curr finger pressed is less than 50ms, we need to rebase touch ic.
						if(((t_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_sec - t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==1) &&	
							(( get_time_interval(t_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_usec+1000000, t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) <= 50*1000))
						{
							ghost_detection = true;
						}
						else if(((t_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_sec - t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==0) &&
							(( get_time_interval(t_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_usec, t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) <= 50*1000))
						{
							ghost_detection = true;
						}
						else	; // do not anything
					}
					else if ( 5 < jitter_sub(ts->ts_data.prev_total_num,ts->ts_data.total_num) )
					{
						 ghost_detection = true;
					}
					else; //do not anything

					memcpy(&ts_prev_finger_press_data, &ts->ts_data.curr_data[id], sizeof(ts_prev_finger_press_data));
				}
			}else{
					memcpy(&t_ex_debug[TIME_EX_PREV_PRESS_TIME], &t_ex_debug[TIME_EX_CURR_PRESS_TIME], sizeof(struct timeval));
					do_gettimeofday(&t_ex_debug[TIME_EX_CURR_INT_TIME]);

				       // if finger subtraction time is less than 10ms, we need to check ghost state.
					if(((t_ex_debug[TIME_EX_CURR_INT_TIME].tv_sec - t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==1) &&
						(( get_time_interval(t_ex_debug[TIME_EX_CURR_INT_TIME].tv_usec+1000000, t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) < 11*1000))
						finger_subtraction_check_count++;
					else if(((t_ex_debug[TIME_EX_CURR_INT_TIME].tv_sec - t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==0) &&
						(( get_time_interval(t_ex_debug[TIME_EX_CURR_INT_TIME].tv_usec, t_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) < 11*1000))
						finger_subtraction_check_count++;
					else
						finger_subtraction_check_count = 0;

					if(4<finger_subtraction_check_count){
						finger_subtraction_check_count = 0;
						TOUCH_INFO_MSG("need_to_rebase finger_subtraction!!! \n");
						goto out_need_to_rebase;
					}
			}
		}

		if (force_continuous_mode){
			do_gettimeofday(&t_ex_debug[TIME_EX_CURR_INT_TIME]);
			// if 20 sec have passed since resume, then return to the original report mode.
			if( t_ex_debug[TIME_EX_CURR_INT_TIME].tv_sec - t_ex_debug[TIME_EX_INIT_TIME].tv_sec >= 10) {
				if(touch_device_func->ic_ctrl){
					if(touch_device_func->ic_ctrl(ts->client, IC_CTRL_REPORT_MODE, ts->pdata->role->report_mode) < 0){
						TOUCH_ERR_MSG("IC_CTRL_BASELINE handling fail\n");
						goto out_need_to_init;
					}
				}
				force_continuous_mode = 0;
			}

			long_press_check = 0;

			for (cnt = 0; cnt < MAX_FINGER; cnt++) {
				if (ts->ts_data.curr_data[cnt].status == FINGER_PRESSED) {
					if (ts->ts_data.prev_data[cnt].status == FINGER_PRESSED&&jitter_sub(ts->ts_data.prev_data[cnt].x_position,ts->ts_data.curr_data[cnt].x_position)<10&&jitter_sub(ts->ts_data.prev_data[cnt].y_position,ts->ts_data.curr_data[cnt].y_position)<10) {
						long_press_check = true;
					}
				}
			}

			if (long_press_check)
				long_press_check_count ++;
			else
				long_press_check_count = 0;

			//TOUCH_INFO_MSG("long_press_check_count %d !!! \n", long_press_check_count);
			if (500< long_press_check_count) {
				long_press_check_count = 0;
				TOUCH_INFO_MSG("need_to_rebase long press!!! \n");
				goto out_need_to_rebase;
			}
		}
	}else if (!ts->ts_data.total_num){
			long_press_check_count = 0;
			finger_subtraction_check_count = 0;

#ifdef PRESSURE_DIFF
			memset(&ts->pressure_diff, 0, sizeof(ts->pressure_diff));
			ts->pressure_diff.z30_id = -1;
			ts->pressure_diff.z_more30_id= -1;
#endif

	}


	if (ts->ts_data.state != TOUCH_BUTTON_LOCK) {
#endif
		if (ts->work_sync_err_cnt > 0
				&& ts->ts_data.prev_button.state == BUTTON_RELEASED) {
			/* Do nothing */
		} else {

				if (button_press_count ==0)
					do_gettimeofday(&t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME]);
				else
					do_gettimeofday(&t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME]);

				button_press_count++;

				if (6 <= button_press_count)
				{
				     if(((t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME].tv_sec - t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME].tv_sec)==1) &&
						(( get_time_interval(t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME].tv_usec+1000000, t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME].tv_usec)) <= 100*1000)) {
							TOUCH_INFO_MSG("need_to_rebase button zero\n");
							goto out_need_to_rebase;
					} else if(((t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME].tv_sec - t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME].tv_sec)==0) &&
						(( get_time_interval(t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME].tv_usec, t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME].tv_usec)) <= 100*1000)) {
							TOUCH_INFO_MSG("need_to_rebase button zero\n");
							goto out_need_to_rebase;
					} else; //do not anything

					button_press_count = 0;
				} else {
					if((t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME].tv_sec - 
						t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME].tv_sec) > 1) 
							button_press_count = 0;
					else if(((t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME].tv_sec - t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME].tv_sec)==1) &&
						(( get_time_interval(t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME].tv_usec+1000000, t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME].tv_usec)) >= 100*1000)) {
							button_press_count = 0;
					} else if(((t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME].tv_sec - t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME].tv_sec)==0) &&
						(( get_time_interval(t_ex_debug[TIME_EX_BUTTON_PRESS_END_TIME].tv_usec, t_ex_debug[TIME_EX_BUTTON_PRESS_START_TIME].tv_usec)) >= 100*1000)) {
							button_press_count = 0;
					} else; //do not anything
				}
		}
	}

	if(ghost_detection == true && ts->ts_data.total_num == 0 && ts->ts_data.palm == 0) {
		TOUCH_INFO_MSG("need_to_rebase zero\n");

		goto out_need_to_rebase;
	}
	else if (ghost_detection == true && 3 <= ghost_detection_count && ts->ts_data.palm == 0) {
		TOUCH_INFO_MSG("need_to_rebase zero\n");
		goto out_need_to_rebase;
	}

#ifdef PRESSURE_DIFF
	else if(ts->pressure_diff.ghost_diff_detection == true && ts->ts_data.total_num == 1 && ts->ts_data.curr_data[ts->pressure_diff.z_more30_id].status == FINGER_RELEASED) {
		TOUCH_INFO_MSG("need_to_rebase pressure_diff is detected\n");
		goto out_need_to_rebase;
	}
#endif
	return 0;

out_need_to_debounce:
	return NEED_TO_OUT;

out_need_to_rebase:
	{
			ghost_detection = false;
			ghost_detection_count = 0;
			memset(&ts_prev_finger_press_data, 0x0, sizeof(ts_prev_finger_press_data));
			button_press_count = 0;
			ts_rebase_count++;
#ifdef PRESSURE_DIFF
			memset(&ts->pressure_diff, 0, sizeof(ts->pressure_diff));
			ts->pressure_diff.z30_id = -1;
			ts->pressure_diff.z_more30_id= -1;
#endif
			if(ts_rebase_count==1) {
					do_gettimeofday(&t_ex_debug[TIME_EX_FIRST_GHOST_DETECT_TIME]);

					if((t_ex_debug[TIME_EX_FIRST_GHOST_DETECT_TIME].tv_sec - t_ex_debug[TIME_EX_INIT_TIME].tv_sec) <= 3) {
						ts_rebase_count = 0;
						TOUCH_INFO_MSG("need_to_init in 3 sec\n");
						goto out_need_to_init;
					}
			} else {
					do_gettimeofday(&t_ex_debug[TIME_EX_SECOND_GHOST_DETECT_TIME]);

					if(((t_ex_debug[TIME_EX_SECOND_GHOST_DETECT_TIME].tv_sec - t_ex_debug[TIME_EX_FIRST_GHOST_DETECT_TIME].tv_sec) <= 5))
					{
							ts_rebase_count = 0;
							TOUCH_INFO_MSG("need_to_init\n");
							goto out_need_to_init;
					} else {
						ts_rebase_count = 1;
						memcpy(&t_ex_debug[TIME_EX_FIRST_GHOST_DETECT_TIME], &t_ex_debug[TIME_EX_SECOND_GHOST_DETECT_TIME], sizeof(struct timeval));
					}
			}
			release_all_ts_event(ts);
			memset(&ts->ts_data, 0, sizeof(ts->ts_data));
			memset(&ts->accuracy_filter.his_data, 0, sizeof(ts->accuracy_filter.his_data));
			ts->accuracy_filter.finish_filter = 0;
			if(touch_device_func->ic_ctrl){
				if(touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_REBASE) < 0){
					TOUCH_ERR_MSG("IC_CTRL_REBASE handling fail\n");
				}
			}
			TOUCH_INFO_MSG("need_to_rebase\n");
	}
	return NEED_TO_OUT;

out_need_to_init:	
	return NEED_TO_INIT;
}
#endif

void Send_Touch( unsigned int x, unsigned int y)
{
	if (touch_test_dev)
	{
		/* press */
#if !defined(MT_PROTOCOL_A)
		input_mt_slot(touch_test_dev->input_dev, 0);
		input_mt_report_slot_state(touch_test_dev->input_dev, MT_TOOL_FINGER, true);
#endif	/* !defined(MT_PROTOCOL_A) */
		input_report_abs(touch_test_dev->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(touch_test_dev->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(touch_test_dev->input_dev, ABS_MT_PRESSURE, 1);
		input_report_abs(touch_test_dev->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(touch_test_dev->input_dev, ABS_MT_WIDTH_MINOR, 1);
#if defined(MT_PROTOCOL_A)
		input_mt_sync(touch_test_dev->input_dev);
#endif	/* defined(MT_PROTOCOL_A) */
		input_sync(touch_test_dev->input_dev);

		/* release */
#if !defined(MT_PROTOCOL_A)
		input_mt_slot(touch_test_dev->input_dev, 0);
		input_mt_report_slot_state(touch_test_dev->input_dev, MT_TOOL_FINGER, false);
#else
		input_mt_sync(touch_test_dev->input_dev);
#endif	/* !defined(MT_PROTOCOL_A) */
		input_sync(touch_test_dev->input_dev);
	}
	else
	{
		TOUCH_ERR_MSG("Touch device not found\n");
	}
}
EXPORT_SYMBOL(Send_Touch);

int get_touch_ts_fw_version(char *fw_ver)
{
	if (touch_test_dev)
	{
		sprintf(fw_ver, "%s", touch_test_dev->fw_info.ic_fw_version);
		return 1;
	}
	else
	{
		return 0;
	}
}
EXPORT_SYMBOL(get_touch_ts_fw_version);

/* set_touch_handle / get_touch_handle
 *
 * Developer can save their object using 'set_touch_handle'.
 * Also, they can restore that using 'get_touch_handle'.
 */
void set_touch_handle(struct i2c_client *client, void *h_touch)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	ts->h_touch = h_touch;
}

void* get_touch_handle(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	return ts->h_touch;
}

/* touch_i2c_read / touch_i2c_write
 *
 * Developer can use these fuctions to communicate with touch_device through I2C.
 */
int touch_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	} else {
		return 0;
	}
}

int touch_i2c_write(struct i2c_client *client, u8 reg, int len, u8 * buf)
{
	unsigned char send_buf[len + 1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = len+1,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char)reg;
	memcpy(&send_buf[1], buf, len);

	if (i2c_transfer(client->adapter, msgs, 1) < 0) {
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	} else
		return 0;
}

int touch_i2c_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	unsigned char send_buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = 2,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char)reg;
	send_buf[1] = (unsigned char)data;

	if (i2c_transfer(client->adapter, msgs, 1) < 0) {
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	} else {
		return 0;
	}
}

#ifdef LGE_TOUCH_TIME_DEBUG
static void time_profile_result(struct lge_touch_data *ts)
{
	if (touch_time_debug_mask & DEBUG_TIME_PROFILE_ALL) {
		if (t_debug[TIME_INT_INTERVAL].tv_sec == 0
				&& t_debug[TIME_INT_INTERVAL].tv_usec == 0) {
			t_debug[TIME_INT_INTERVAL].tv_sec =
						t_debug[TIME_ISR_START].tv_sec;
			t_debug[TIME_INT_INTERVAL].tv_usec =
						t_debug[TIME_ISR_START].tv_usec;
		} else {
			TOUCH_INFO_MSG("Interval [%6luus], Total [%6luus], "
				"IRQ -> Thread IRQ [%6luus] -> work [%6luus]"
				" -> report [%6luus]\n",
				get_time_interval(t_debug[TIME_ISR_START].tv_usec, t_debug[TIME_INT_INTERVAL].tv_usec),
				get_time_interval(t_debug[TIME_WORKQUEUE_END].tv_usec, t_debug[TIME_ISR_START].tv_usec),
				get_time_interval(t_debug[TIME_THREAD_ISR_START].tv_usec, t_debug[TIME_ISR_START].tv_usec),
				get_time_interval(t_debug[TIME_WORKQUEUE_START].tv_usec, t_debug[TIME_THREAD_ISR_START].tv_usec),
				get_time_interval(t_debug[TIME_WORKQUEUE_END].tv_usec, t_debug[TIME_WORKQUEUE_START].tv_usec));

			t_debug[TIME_INT_INTERVAL].tv_sec =
						t_debug[TIME_ISR_START].tv_sec;
			t_debug[TIME_INT_INTERVAL].tv_usec =
						t_debug[TIME_ISR_START].tv_usec;
		}
	} else {
		if (touch_time_debug_mask & DEBUG_TIME_INT_INTERVAL) {
			if (t_debug[TIME_INT_INTERVAL].tv_sec == 0
				&& t_debug[TIME_INT_INTERVAL].tv_usec == 0) {
				t_debug[TIME_INT_INTERVAL].tv_sec =
						t_debug[TIME_ISR_START].tv_sec;
				t_debug[TIME_INT_INTERVAL].tv_usec =
						t_debug[TIME_ISR_START].tv_usec;
			} else {
				TOUCH_INFO_MSG("Interrupt interval: %6luus\n",
					get_time_interval(t_debug[TIME_ISR_START].tv_usec, t_debug[TIME_INT_INTERVAL].tv_usec));

				t_debug[TIME_INT_INTERVAL].tv_sec =
						t_debug[TIME_ISR_START].tv_sec;
				t_debug[TIME_INT_INTERVAL].tv_usec =
						t_debug[TIME_ISR_START].tv_usec;
			}
		}

		if (touch_time_debug_mask & DEBUG_TIME_INT_IRQ_DELAY) {
			TOUCH_INFO_MSG("IRQ -> Thread IRQ : %6luus\n",
				get_time_interval(t_debug[TIME_THREAD_ISR_START].tv_usec, t_debug[TIME_ISR_START].tv_usec));
		}

		if (touch_time_debug_mask & DEBUG_TIME_INT_THREAD_IRQ_DELAY) {
			TOUCH_INFO_MSG("Thread IRQ -> work: %6luus\n",
				get_time_interval(t_debug[TIME_WORKQUEUE_START].tv_usec, t_debug[TIME_THREAD_ISR_START].tv_usec));
		}

		if (touch_time_debug_mask & DEBUG_TIME_DATA_HANDLE) {
			TOUCH_INFO_MSG("work -> report: %6luus\n",
				get_time_interval(t_debug[TIME_WORKQUEUE_END].tv_usec, t_debug[TIME_WORKQUEUE_START].tv_usec));
		}
	}

	if (!ts->ts_data.total_num)
		memset(t_debug, 0x0, sizeof(t_debug));
}
#endif

/* release_all_ts_event
 *
 * When system enters suspend-state,
 * if user press touch-panel, release them automatically.
 */
static void release_all_ts_event(struct lge_touch_data *ts)
{
	if (ts->pdata->role->key_type == TOUCH_HARD_KEY){
		if (ts->ts_data.prev_total_num) {
			touch_asb_input_report(ts, FINGER_RELEASED);

			if (likely(touch_debug_mask & (DEBUG_ABS | DEBUG_BASE_INFO)))
				TOUCH_INFO_MSG("touch finger position released\n");
		}

		if(ts->ts_data.prev_button.state == BUTTON_PRESSED) {
			input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_RELEASED);
			if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
				TOUCH_INFO_MSG("Touch KEY[%d] is released\n", ts->ts_data.prev_button.key_code);
		}
	}
	else if (ts->pdata->role->key_type == VIRTUAL_KEY){
		if (ts->ts_data.prev_total_num) {
			touch_asb_input_report(ts, FINGER_RELEASED);

			if (likely(touch_debug_mask & (DEBUG_ABS | DEBUG_BASE_INFO)))
				TOUCH_INFO_MSG("touch finger position released\n");
		}
	}
	else if (ts->pdata->role->key_type == TOUCH_SOFT_KEY){
		if (ts->ts_data.state == ABS_PRESS) {
			touch_asb_input_report(ts, FINGER_RELEASED);

	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (!ts->ts_data.curr_data[id].state)
			continue;

		input_mt_slot(ts->input_dev, id);
		input_mt_report_slot_state(ts->input_dev,
				ts->ts_data.curr_data[id].tool_type, 0);
		ts->ts_data.curr_data[id].state = 0;
	}

	input_sync(ts->input_dev);
}

/* touch_power_cntl
 *
 * 1. POWER_ON
 * 2. POWER_OFF
 * 3. POWER_SLEEP
 * 4. POWER_WAKE
 */
static int touch_power_cntl(struct lge_touch_data *ts, int onoff)
{
	int ret = 0;

	if (touch_device_func->power == NULL) {
		TOUCH_INFO_MSG("There is no specific power control function\n");
		return -1;
	}

	switch (onoff) {
	case POWER_ON:
		ret = touch_device_func->power(ts->client, POWER_ON);
		if (ret < 0)
			TOUCH_ERR_MSG("power on failed\n");
		else
			ts->curr_pwr_state = POWER_ON;
		break;
	case POWER_OFF:
		ret = touch_device_func->power(ts->client, POWER_OFF);
		if (ret < 0)
			TOUCH_ERR_MSG("power off failed\n");
		else
			ts->curr_pwr_state = POWER_OFF;

		msleep(ts->pdata->role->reset_delay);

		atomic_set(&ts->device_init, 0);
		break;
	case POWER_SLEEP:
		ret = touch_device_func->power(ts->client, POWER_SLEEP);
		if (ret < 0)
			TOUCH_ERR_MSG("power sleep failed\n");
		else
			ts->curr_pwr_state = POWER_SLEEP;
		break;
	case POWER_WAKE:
		ret = touch_device_func->power(ts->client, POWER_WAKE);
		if (ret < 0)
			TOUCH_ERR_MSG("power wake failed\n");
		else
			ts->curr_pwr_state = POWER_WAKE;
		break;
	default:
		break;
	}

	if (unlikely(touch_debug_mask & DEBUG_POWER))
		if (ret >= 0)
			TOUCH_INFO_MSG("%s: power_state[%d]",
					__FUNCTION__, ts->curr_pwr_state);

	return ret;
}

/* safety_reset
 *
 * 1. disable irq/timer.
 * 2. turn off the power.
 * 3. turn on the power.
 * 4. sleep (booting_delay)ms, usually 400ms(synaptics).
 * 5. enable irq/timer.
 *
 * After 'safety_reset', we should call 'touch_init'.
 */
static void safety_reset(struct lge_touch_data *ts)
{
	if (ts->pdata->role->operation_mode)
		disable_irq(ts->client->irq);
	else
		hrtimer_cancel(&ts->timer);

	release_all_ts_event(ts);

	touch_power_cntl(ts, POWER_OFF);
	touch_power_cntl(ts, POWER_ON);
	msleep(ts->pdata->role->booting_delay);

	if (ts->pdata->role->operation_mode == INTERRUPT_MODE)
		enable_irq(ts->client->irq);
	else
		hrtimer_start(&ts->timer,
				ktime_set(0, ts->pdata->role->report_period),
				HRTIMER_MODE_REL);
}

/* touch_ic_init
 *
 * initialize the device_IC and variables.
 */
static int touch_ic_init(struct lge_touch_data *ts)
{
	int int_pin = 0;
	int next_work = 0;

	if (unlikely(ts->ic_init_err_cnt >= MAX_RETRY_COUNT)) {
		TOUCH_ERR_MSG("Init Failed: Irq-pin has some unknown problems\n");
		goto err_out_critical;
	}

	if (touch_device_func->init == NULL) {
		TOUCH_INFO_MSG("There is no specific IC init function\n");
		goto err_out_critical;
	}

	/* Do the soft reset to make sure the controller is reset OK */
	if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_RESET_CMD, 0) < 0) {
		TOUCH_ERR_MSG("RESET FAILED\n");
		goto err_out_retry;
	}

	atomic_set(&ts->next_work, 0);
	atomic_set(&ts->device_init, 1);

	if (touch_device_func->init(ts->client, &ts->fw_info) < 0) {
		TOUCH_ERR_MSG("specific device initialization fail\n");
		goto err_out_retry;
	}

	/* Interrupt pin check after IC init - avoid Touch lockup */
	if (ts->pdata->role->operation_mode == INTERRUPT_MODE) {
		int_pin = gpio_get_value(ts->pdata->int_pin);
		next_work = atomic_read(&ts->next_work);

		if (unlikely(int_pin != 1 && next_work <= 0)) {
			TOUCH_INFO_MSG("WARN: Interrupt pin is low"
					" - next_work: %d, try_count: %d]\n",
					next_work, ts->ic_init_err_cnt);
			goto err_out_retry;
		}
	}

	ts->gf_ctrl.count = 0;
	ts->gf_ctrl.ghost_check_count = 0;
	memset(&ts->ts_data, 0, sizeof(ts->ts_data));
	memset(&ts->fw_upgrade, 0, sizeof(ts->fw_upgrade));
	ts->ic_init_err_cnt = 0;

	ts->jitter_filter.id_mask = 0;
	memset(ts->jitter_filter.his_data,
				0, sizeof(ts->jitter_filter.his_data));
	memset(&ts->accuracy_filter.his_data,
				0, sizeof(ts->accuracy_filter.his_data));

	return 0;

err_out_retry:
	ts->ic_init_err_cnt++;
	safety_reset(ts);
	queue_delayed_work(touch_wq, &ts->work_init, msecs_to_jiffies(10));

	return 0;

err_out_critical:
	ts->ic_init_err_cnt = 0;

	return -1;
}

/* ghost_finger_solution
 *
 * GHOST_STAGE_1
 * - melt_mode.
 * - If user press and release their finger in 1 sec, STAGE_1 will be cleared. --> STAGE_2
 * - If there is no key-guard, ghost_finger_solution is finished.
 *
 * GHOST_STAGE_2
 * - no_melt_mode
 * - if user use multi-finger, stage will be changed to STAGE_1
 *   (We assume that ghost-finger occured)
 * - if key-guard is unlocked, STAGE_2 is cleared. --> STAGE_3
 *
 * GHOST_STAGE_3
 * - when user release their finger, device re-scan the baseline.
 * - Then, GHOST_STAGE3 is cleared and ghost_finger_solution is finished.
 */
#define ghost_sub(x, y)	(x > y ? x - y : y - x)

int ghost_finger_solution(struct lge_touch_data *ts)
{
	u8	id = 0;
for (id = 0; id < ts->pdata->caps->max_id; id++){
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			break;
		}
	}

	if(ts->gf_ctrl.stage & GHOST_STAGE_1){
		if(ts->ts_data.total_num == 0 && ts->ts_data.curr_button.state == 0 && ts->ts_data.palm == 0){
			if(ts->gf_ctrl.count < ts->gf_ctrl.min_count || ts->gf_ctrl.count >= ts->gf_ctrl.max_count){
				if(ts->gf_ctrl.stage & GHOST_STAGE_2)
					ts->gf_ctrl.ghost_check_count = MAX_GHOST_CHECK_COUNT - 1;
				else
					ts->gf_ctrl.ghost_check_count = 0;
			}
			else{
				if(ghost_sub(ts->gf_ctrl.saved_x, ts->gf_ctrl.saved_last_x) > ts->gf_ctrl.max_moved ||
				   ghost_sub(ts->gf_ctrl.saved_y, ts->gf_ctrl.saved_last_y) > ts->gf_ctrl.max_moved)
					ts->gf_ctrl.ghost_check_count = MAX_GHOST_CHECK_COUNT;
				else
					ts->gf_ctrl.ghost_check_count++;

				if (unlikely(touch_debug_mask & DEBUG_GHOST))
					TOUCH_INFO_MSG("ghost_stage_1: delta[%d/%d/%d]\n",
						ghost_sub(ts->gf_ctrl.saved_x, ts->gf_ctrl.saved_last_x),
						ghost_sub(ts->gf_ctrl.saved_y, ts->gf_ctrl.saved_last_y),
						ts->gf_ctrl.max_moved);
			}

			if (unlikely(touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO))
				TOUCH_INFO_MSG("ghost_stage_1: ghost_check_count+[0x%x]\n", ts->gf_ctrl.ghost_check_count);

			if(ts->gf_ctrl.ghost_check_count >= MAX_GHOST_CHECK_COUNT){
				ts->gf_ctrl.ghost_check_count = 0;
				if(touch_device_func->ic_ctrl){
					if(touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_FIX) < 0)
						return -1;
				}
				ts->gf_ctrl.stage &= ~GHOST_STAGE_1;
				if (unlikely(touch_debug_mask & DEBUG_GHOST|| touch_debug_mask & DEBUG_BASE_INFO))
					TOUCH_INFO_MSG("ghost_stage_1: cleared[0x%x]\n", ts->gf_ctrl.stage);
				if(!ts->gf_ctrl.stage){
					if (unlikely(touch_debug_mask & DEBUG_GHOST))
						TOUCH_INFO_MSG("ghost_stage_finished. (NON-KEYGUARD)\n");
				}
			}
			ts->gf_ctrl.count = 0;
			ts->gf_ctrl.saved_x = -1;
			ts->gf_ctrl.saved_y = -1;
		}
		else if(ts->ts_data.total_num == 1 && ts->ts_data.curr_button.state == 0
				&& id == 0 && ts->ts_data.palm == 0){
			if(ts->gf_ctrl.saved_x == -1 && ts->gf_ctrl.saved_x == -1){
				ts->gf_ctrl.saved_x = ts->ts_data.curr_data[id].x_position;
				ts->gf_ctrl.saved_y = ts->ts_data.curr_data[id].y_position;
			}
			ts->gf_ctrl.count++;
			if (unlikely(touch_debug_mask & DEBUG_GHOST))
				TOUCH_INFO_MSG("ghost_stage_1: int_count[%d/%d]\n", ts->gf_ctrl.count, ts->gf_ctrl.max_count);
		}
		else {
			if (unlikely(touch_debug_mask & DEBUG_GHOST) && ts->gf_ctrl.count != ts->gf_ctrl.max_count)
				TOUCH_INFO_MSG("ghost_stage_1: Not good condition. total[%d] button[%d] id[%d] palm[%d]\n",
						ts->ts_data.total_num, ts->ts_data.curr_button.state,
						id, ts->ts_data.palm);
			ts->gf_ctrl.count = ts->gf_ctrl.max_count;
		}
	}
	else if(ts->gf_ctrl.stage & GHOST_STAGE_2){
		if(ts->ts_data.total_num > 1 || (ts->ts_data.total_num == 1 && ts->ts_data.curr_button.state)){
			ts->gf_ctrl.stage |= GHOST_STAGE_1;
			ts->gf_ctrl.ghost_check_count = MAX_GHOST_CHECK_COUNT - 1;
			ts->gf_ctrl.count = 0;
			ts->gf_ctrl.saved_x = -1;
			ts->gf_ctrl.saved_y = -1;
			if(touch_device_func->ic_ctrl){
				if(touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_OPEN) < 0)
					return -1;
#ifdef CUST_G_TOUCH
//do nothing
#else
				if(touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_REBASE) < 0)
					return -1;
#endif
			}
			if (unlikely(touch_debug_mask & DEBUG_GHOST))
				TOUCH_INFO_MSG("ghost_stage_2: multi_finger. return to ghost_stage_1[0x%x]\n", ts->gf_ctrl.stage);
		}
	}
	else if(ts->gf_ctrl.stage & GHOST_STAGE_3){
		if(ts->ts_data.total_num == 0 && ts->ts_data.curr_button.state == 0 && ts->ts_data.palm == 0){
			ts->gf_ctrl.ghost_check_count++;

			if (unlikely(touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO))
				TOUCH_INFO_MSG("ghost_stage_3: ghost_check_count+[0x%x]\n", ts->gf_ctrl.ghost_check_count);

			if(ts->gf_ctrl.ghost_check_count >= MAX_GHOST_CHECK_COUNT){
				ts->gf_ctrl.stage &= ~GHOST_STAGE_3;
				if (unlikely(touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO))
					TOUCH_INFO_MSG("ghost_stage_3: cleared[0x%x]\n", ts->gf_ctrl.stage);
				if(!ts->gf_ctrl.stage){
					if (unlikely(touch_debug_mask & DEBUG_GHOST))
						TOUCH_INFO_MSG("ghost_stage_finished. (NON-KEYGUARD)\n");
				}
			}
		}
		else if(ts->ts_data.total_num == 1 && ts->ts_data.curr_button.state == 0
				&& id == 0 && ts->ts_data.palm == 0);
		else{
			ts->gf_ctrl.stage &= ~GHOST_STAGE_3;
			ts->gf_ctrl.stage |= GHOST_STAGE_1;
			ts->gf_ctrl.ghost_check_count = 0;
			ts->gf_ctrl.count = 0;
			ts->gf_ctrl.saved_x = -1;
			ts->gf_ctrl.saved_y = -1;
			if (touch_device_func->ic_ctrl) {
				if(touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_OPEN) < 0)
					return -1;
			}

			if (unlikely(touch_debug_mask & DEBUG_GHOST)){
				TOUCH_INFO_MSG("ghost_stage_3: Not good condition. total[%d] button[%d] id[%d] palm[%d]\n",
						ts->ts_data.total_num, ts->ts_data.curr_button.state,
						id, ts->ts_data.palm);
				TOUCH_INFO_MSG("ghost_stage_3: return to ghost_stage_1[0x%x]\n", ts->gf_ctrl.stage);
			}
		}
	}
	else if(ts->gf_ctrl.stage & GHOST_STAGE_4){
		if(ts->ts_data.total_num == 0 && ts->ts_data.curr_button.state == 0 && ts->ts_data.palm == 0){
#ifdef CUST_G_TOUCH
//do nothing
#else
			if(touch_device_func->ic_ctrl){
				if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_REBASE) < 0)
					return -1;
			}
#endif
			ts->gf_ctrl.stage = GHOST_STAGE_CLEAR;
			if (unlikely(touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO))
				TOUCH_INFO_MSG("ghost_stage_4: cleared[0x%x]\n", ts->gf_ctrl.stage);
			if (unlikely(touch_debug_mask & DEBUG_GHOST))
				TOUCH_INFO_MSG("ghost_stage_finished. (KEYGUARD)\n");
		}

		if (unlikely(touch_debug_mask & DEBUG_GHOST) && ts->ts_data.palm != 0)
			TOUCH_INFO_MSG("ghost_stage_4: palm[%d]\n", ts->ts_data.palm);
	}

	ts->gf_ctrl.saved_last_x = ts->ts_data.curr_data[id].x_position;
	ts->gf_ctrl.saved_last_y = ts->ts_data.curr_data[id].y_position;

	return 0;
}

/* Jitter Filter
 *
 */
#define jitter_abs(x)           ((x) > 0 ? (x) : -(x))
#define jitter_sub(x, y)        ((x) > (y) ? (x) - (y) : (y) - (x))

static u16 check_boundary(int x, int max)
{
	if (x < 0)
		return 0;
	else if (x > max)
		return (u16)max;
	else
		return (u16)x;
}

static int check_direction(int x)
{
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

static int accuracy_filter_func(struct lge_touch_data *ts)
{
	int delta_x = 0;
	int delta_y = 0;
	u8	id = 0;
	for (id = 0; id < ts->pdata->caps->max_id; id++){
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			break;
		}
	}

	/* finish the accuracy_filter */
	if (ts->accuracy_filter.finish_filter == 1 &&
			(ts->accuracy_filter.his_data.count >
				ts->accuracy_filter.touch_max_count ||
			ts->ts_data.total_num != 1)) {
		ts->accuracy_filter.finish_filter = 0;
		ts->accuracy_filter.his_data.count = 0;
	}

	if (ts->accuracy_filter.finish_filter) {
		delta_x = (int)ts->accuracy_filter.his_data.x -
				(int)ts->ts_data.curr_data[0].x_position;
		delta_y = (int)ts->accuracy_filter.his_data.y -
				(int)ts->ts_data.curr_data[0].y_position;
		if (delta_x || delta_y) {
			ts->accuracy_filter.his_data.axis_x +=
						check_direction(delta_x);
			ts->accuracy_filter.his_data.axis_y +=
						check_direction(delta_y);
			ts->accuracy_filter.his_data.count++;
		}

		if (ts->accuracy_filter.his_data.count == 1 ||
			((jitter_sub(ts->ts_data.curr_data[0].pressure,
			ts->accuracy_filter.his_data.pressure) >
				ts->accuracy_filter.ignore_pressure_gap ||
			ts->ts_data.curr_data[0].pressure >
				ts->accuracy_filter.max_pressure) &&
			!((ts->accuracy_filter.his_data.count >
				ts->accuracy_filter.time_to_max_pressure &&
			(jitter_abs(ts->accuracy_filter.his_data.axis_x) ==
					ts->accuracy_filter.his_data.count ||
			jitter_abs(ts->accuracy_filter.his_data.axis_y) ==
					ts->accuracy_filter.his_data.count)) ||
			(jitter_abs(ts->accuracy_filter.his_data.axis_x) >
					ts->accuracy_filter.direction_count ||
			jitter_abs(ts->accuracy_filter.his_data.axis_y) >
				ts->accuracy_filter.direction_count)))) {
			ts->accuracy_filter.his_data.mod_x += delta_x;
			ts->accuracy_filter.his_data.mod_y += delta_y;
		}
	}

	/* if 'delta' > delta_max or id != 0, remove the modify-value. */
	if ((ts->accuracy_filter.his_data.count != 1 &&
		(jitter_abs(delta_x) > ts->accuracy_filter.delta_max ||
		jitter_abs(delta_y) > ts->accuracy_filter.delta_max))) {
		ts->accuracy_filter.his_data.mod_x = 0;
		ts->accuracy_filter.his_data.mod_y = 0;
	}

	/* start the accuracy_filter */
	if (ts->accuracy_filter.finish_filter == 0
			&& ts->accuracy_filter.his_data.count == 0
			&& ts->ts_data.total_num == 1
			&& ts->accuracy_filter.his_data.prev_total_num == 0) {
		ts->accuracy_filter.finish_filter = 1;
		memset(&ts->accuracy_filter.his_data, 0,
					sizeof(ts->accuracy_filter.his_data));
	}

	if (unlikely(touch_debug_mask & DEBUG_ACCURACY)) {
		TOUCH_INFO_MSG("AccuracyFilter: <0> pos[%4d,%4d] "
			"new[%4d,%4d] his[%4d,%4d] delta[%3d,%3d] "
			"mod[%3d,%3d] p[%d,%3d,%3d] axis[%2d,%2d] "
			"count[%2d/%2d] total_num[%d,%d] finish[%d]\n",
			ts->ts_data.curr_data[0].x_position,
			ts->ts_data.curr_data[0].y_position,
			check_boundary((int)ts->ts_data.curr_data[0].x_position + ts->accuracy_filter.his_data.mod_x, ts->pdata->caps->x_max),
			check_boundary((int)ts->ts_data.curr_data[0].y_position + ts->accuracy_filter.his_data.mod_y, ts->pdata->caps->y_max),
			ts->accuracy_filter.his_data.x,
			ts->accuracy_filter.his_data.y,
			delta_x, delta_y,
			ts->accuracy_filter.his_data.mod_x,
			ts->accuracy_filter.his_data.mod_y,
			jitter_sub(ts->ts_data.curr_data[0].pressure,
			ts->accuracy_filter.his_data.pressure) >
					ts->accuracy_filter.ignore_pressure_gap,
			ts->ts_data.curr_data[0].pressure,
			ts->accuracy_filter.his_data.pressure,
			ts->accuracy_filter.his_data.axis_x,
			ts->accuracy_filter.his_data.axis_y,
			ts->accuracy_filter.his_data.count,
			ts->accuracy_filter.touch_max_count,
			ts->accuracy_filter.his_data.prev_total_num,
			ts->ts_data.total_num, ts->accuracy_filter.finish_filter);
	}

	ts->accuracy_filter.his_data.x = ts->ts_data.curr_data[0].x_position;
	ts->accuracy_filter.his_data.y = ts->ts_data.curr_data[0].y_position;
	ts->accuracy_filter.his_data.pressure = ts->ts_data.curr_data[0].pressure;
	ts->accuracy_filter.his_data.prev_total_num = ts->ts_data.total_num;

	if (ts->ts_data.total_num) {
		ts->ts_data.curr_data[0].x_position =
			check_boundary((int)ts->ts_data.curr_data[0].x_position + ts->accuracy_filter.his_data.mod_x, ts->pdata->caps->x_max);
		ts->ts_data.curr_data[0].y_position =
			check_boundary((int)ts->ts_data.curr_data[0].y_position + ts->accuracy_filter.his_data.mod_y, ts->pdata->caps->y_max);
	}

	return 0;
}

static int jitter_filter_func(struct lge_touch_data *ts)
{
	int id;
	int jitter_count = 0;
	u16 new_id_mask = 0;
	u16 bit_mask = 0;
	u16 bit_id = 1;
	int curr_ratio = ts->pdata->role->jitter_curr_ratio;

	for (id = 0; id < ts->pdata->caps->max_id; id++){
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			u16 width = ts->ts_data.curr_data[id].width_major;
			new_id_mask |= (1 << id);

			if(ts->jitter_filter.id_mask & (1 << id)){
				int delta_x, delta_y;
				int f_jitter = curr_ratio*width;
				int adjust_x = 0;
				int adjust_y = 0;
				int adj_ratio = 0;
				char adj_mode = 0;

				if (ts->jitter_filter.adjust_margin > 0) {
					adjust_x = (int)ts->ts_data.curr_data[id].x_position
								- (int)ts->jitter_filter.his_data[id].x;
					adjust_y = (int)ts->ts_data.curr_data[id].y_position
								- (int)ts->jitter_filter.his_data[id].y;
				}

				ts->ts_data.curr_data[id].x_position =
						(ts->ts_data.curr_data[id].x_position + ts->jitter_filter.his_data[id].x) >> 1;
				ts->ts_data.curr_data[id].y_position =
						(ts->ts_data.curr_data[id].y_position + ts->jitter_filter.his_data[id].y) >> 1;

				if (ts->jitter_filter.adjust_margin > 0) {
					adj_ratio = (((width + 1) << 6) / (curr_ratio + 1))
							+ (ts->jitter_filter.adjust_margin >> 3);
					if (jitter_abs(adjust_x) > ts->jitter_filter.adjust_margin
							|| jitter_abs(adjust_x) > adj_ratio
							|| jitter_abs(adjust_y) > ts->jitter_filter.adjust_margin
							|| jitter_abs(adjust_y) > adj_ratio) {
						adjust_x = (int)ts->ts_data.curr_data[id].x_position + (adjust_x >> 2);
						ts->ts_data.curr_data[id].x_position = check_boundary(adjust_x, ts->pdata->caps->x_max);

						adjust_y = (int)ts->ts_data.curr_data[id].y_position + (adjust_y >> 2);
						ts->ts_data.curr_data[id].y_position = check_boundary(adjust_y, ts->pdata->caps->y_max);

						adj_mode = 1;
					}
				}
				delta_x = (int)ts->ts_data.curr_data[id].x_position - (int)ts->jitter_filter.his_data[id].x;
				delta_y = (int)ts->ts_data.curr_data[id].y_position - (int)ts->jitter_filter.his_data[id].y;

				ts->jitter_filter.his_data[id].delta_x = delta_x * curr_ratio
						+ ((ts->jitter_filter.his_data[id].delta_x * (128 - curr_ratio)) >> 7);
				ts->jitter_filter.his_data[id].delta_y = delta_y * curr_ratio
						+ ((ts->jitter_filter.his_data[id].delta_y * (128 - curr_ratio)) >> 7);

				if(unlikely(touch_debug_mask & DEBUG_JITTER)){
					TOUCH_INFO_MSG("JitterFilter[%s]: <%d> p[%d,%d] h_p[%d,%d] w[%d] a_r[%d] d[%d,%d] h_d[%d,%d] f_j[%d]\n",
							adj_mode?"fast":"norm", id, ts->ts_data.curr_data[id].x_position, ts->ts_data.curr_data[id].y_position,
							ts->jitter_filter.his_data[id].x, ts->jitter_filter.his_data[id].y,
							width, adj_ratio, delta_x, delta_y,
							ts->jitter_filter.his_data[id].delta_x, ts->jitter_filter.his_data[id].delta_y, f_jitter);
				}

				if(jitter_abs(ts->jitter_filter.his_data[id].delta_x) <= f_jitter &&
				   jitter_abs(ts->jitter_filter.his_data[id].delta_y) <= f_jitter)
					jitter_count++;
			}
		}
	}

	bit_mask = ts->jitter_filter.id_mask ^ new_id_mask;

	for (id = 0, bit_id = 1; id < ts->pdata->caps->max_id; id++){
		if ((ts->jitter_filter.id_mask & bit_id) && !(new_id_mask & bit_id)){
			if(unlikely(touch_debug_mask & DEBUG_JITTER))
				TOUCH_INFO_MSG("JitterFilter: released - id[%d] mask[0x%x]\n", bit_id, ts->jitter_filter.id_mask);
			memset(&ts->jitter_filter.his_data[id], 0, sizeof(ts->jitter_filter.his_data[id]));
		}
		bit_id = bit_id << 1;
	}

	for (id = 0; id < ts->pdata->caps->max_id; id++){
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			ts->jitter_filter.his_data[id].pressure = ts->ts_data.curr_data[id].pressure;
		}
	}

	if (!bit_mask && ts->ts_data.total_num && ts->ts_data.total_num == jitter_count){
		if(unlikely(touch_debug_mask & DEBUG_JITTER))
			TOUCH_INFO_MSG("JitterFilter: ignored - jitter_count[%d] total_num[%d] bitmask[0x%x]\n",
					jitter_count, ts->ts_data.total_num, bit_mask);
		return -1;
	}

	for (id = 0; id < ts->pdata->caps->max_id; id++){
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			ts->jitter_filter.his_data[id].x = ts->ts_data.curr_data[id].x_position;
			ts->jitter_filter.his_data[id].y = ts->ts_data.curr_data[id].y_position;
		}
	}

	ts->jitter_filter.id_mask = new_id_mask;

	return 0;
}
EXPORT_SYMBOL(jitter_filter_func);
/* touch_init_func
 *
 * In order to reduce the booting-time,
 * we used delayed_work_queue instead of msleep or mdelay.
 */
static void touch_init_func(struct work_struct *work_init)
{
	struct lge_touch_data *ts =
			container_of(to_delayed_work(work_init), struct lge_touch_data, work_init);

		if (!ts->ts_data.curr_data[id].state)
			continue;

		width = ts->ts_data.curr_data[id].width_major;
		new_id_mask |= (1 << id);

		if (ts->jitter_filter.id_mask & (1 << id)) {
			int delta_x, delta_y;
			int f_jitter = curr_ratio*width;
			int adjust_x, adjust_y;

			if (ts->jitter_filter.adjust_margin > 0) {
				adjust_x = (int)ts->ts_data.curr_data[id].x_position - (int)ts->jitter_filter.his_data[id].x;
				adjust_y = (int)ts->ts_data.curr_data[id].y_position - (int)ts->jitter_filter.his_data[id].y;

				if (jitter_abs(adjust_x) > ts->jitter_filter.adjust_margin) {
					adjust_x = (int)ts->ts_data.curr_data[id].x_position + (adjust_x >> 2);
					ts->ts_data.curr_data[id].x_position =
						check_boundary(adjust_x,
							ts->pdata->caps->x_max);
				}

				if (jitter_abs(adjust_y) > ts->jitter_filter.adjust_margin) {
					adjust_y = (int)ts->ts_data.curr_data[id].y_position + (adjust_y >> 2);
					ts->ts_data.curr_data[id].y_position =
						check_boundary(adjust_y,
							ts->pdata->caps->y_max);
				}
			}

			ts->ts_data.curr_data[id].x_position =
				(ts->ts_data.curr_data[id].x_position +
					ts->jitter_filter.his_data[id].x) >> 1;
			ts->ts_data.curr_data[id].y_position =
				(ts->ts_data.curr_data[id].y_position +
					ts->jitter_filter.his_data[id].y) >> 1;

			delta_x = (int)ts->ts_data.curr_data[id].x_position -
					(int)ts->jitter_filter.his_data[id].x;
			delta_y = (int)ts->ts_data.curr_data[id].y_position -
					(int)ts->jitter_filter.his_data[id].y;

			ts->jitter_filter.his_data[id].delta_x = delta_x * curr_ratio + ((ts->jitter_filter.his_data[id].delta_x * (128 - curr_ratio)) >> 7);
			ts->jitter_filter.his_data[id].delta_y = delta_y * curr_ratio + ((ts->jitter_filter.his_data[id].delta_y * (128 - curr_ratio)) >> 7);

			if (unlikely(touch_debug_mask & DEBUG_JITTER))
				TOUCH_INFO_MSG("JitterFilter: <%d> pos[%d,%d] "
					"h_pos[%d,%d] delta[%d,%d] "
					"h_delta[%d,%d] j_fil[%d]\n",
					id, ts->ts_data.curr_data[id].x_position,
					ts->ts_data.curr_data[id].y_position,
					ts->jitter_filter.his_data[id].x,
					ts->jitter_filter.his_data[id].y,
					delta_x, delta_y,
					ts->jitter_filter.his_data[id].delta_x,
					ts->jitter_filter.his_data[id].delta_y,
					f_jitter);

			if (jitter_abs(ts->jitter_filter.his_data[id].delta_x) <= f_jitter && jitter_abs(ts->jitter_filter.his_data[id].delta_y) <= f_jitter)
				jitter_count++;
		}
	}

	bit_mask = ts->jitter_filter.id_mask ^ new_id_mask;

	for (id = 0, bit_id = 1; id < ts->pdata->caps->max_id; id++) {
		if ((ts->jitter_filter.id_mask & bit_id) &&
						!(new_id_mask & bit_id)) {
			if (unlikely(touch_debug_mask & DEBUG_JITTER))
				TOUCH_INFO_MSG("JitterFilter: released - "
					"id[%d] mask[0x%x]\n",
					bit_id, ts->jitter_filter.id_mask);
			memset(&ts->jitter_filter.his_data[id], 0,
					sizeof(ts->jitter_filter.his_data[id]));
		}
		bit_id = bit_id << 1;
	}
	
	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (!ts->ts_data.curr_data[id].state)
			continue;

		ts->jitter_filter.his_data[id].pressure =
					ts->ts_data.curr_data[id].pressure;
	TOUCH_INFO_MSG("touch_release[%s] : <%d> x[%4d] y[%4d]\n",
			ts->ts_data.palm?"Palm":"", id,
			ts->ts_data.prev_data[id].x_position,
			ts->ts_data.prev_data[id].y_position);
}

/* touch_work_pre_proc
 *
 * Pre-process work at touch_work
 */
static int touch_work_pre_proc(struct lge_touch_data *ts)
{
#ifdef CUST_G_TOUCH
	int ret = 0;
#endif
	atomic_dec(&ts->next_work);
	ts->ts_data.total_num = 0;
	ts->int_pin_state = 0;
if(unlikely(ts->work_sync_err_cnt >= MAX_RETRY_COUNT)){
		TOUCH_ERR_MSG("Work Sync Failed: Irq-pin has some unknown problems\n");
		return -EIO;
	}

	if (!bit_mask && ts->ts_data.total_num &&
				ts->ts_data.total_num == jitter_count) {
		if (unlikely(touch_debug_mask & DEBUG_JITTER))
			TOUCH_INFO_MSG("JitterFilter: ignored - "
					"jitter_count[%d] total_num[%d] "						"bitmask[0x%x]\n",
					jitter_count, ts->ts_data.total_num,
					bit_mask);
		return -1;
	}

	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (!ts->ts_data.curr_data[id].state)
			continue;

		ts->jitter_filter.his_data[id].x =
					ts->ts_data.curr_data[id].x_position;
		ts->jitter_filter.his_data[id].y =
					ts->ts_data.curr_data[id].y_position;
	}

	ts->jitter_filter.id_mask = new_id_mask;

	return 0;
}

/* touch_work_post_proc
 *
 * Post-process work at touch_work
 */
static void touch_work_post_proc(struct lge_touch_data *ts, int post_proc)
{
	int next_work = 0;

	if (post_proc >= WORK_POST_MAX)
		return;

	switch (post_proc) {
	case WORK_POST_OUT:
		if(likely(ts->pdata->role->operation_mode == INTERRUPT_MODE)){
			next_work = atomic_read(&ts->next_work);

			if(unlikely(ts->int_pin_state != 1 && next_work <= 0)){
				TOUCH_INFO_MSG("WARN: Interrupt pin is low - next_work: %d, try_count: %d]\n",
						next_work, ts->work_sync_err_cnt);
				post_proc = WORK_POST_ERR_RETRY;
				break;
			}
		}

#ifdef LGE_TOUCH_TIME_DEBUG
		do_gettimeofday(&t_debug[TIME_WORKQUEUE_END]);
		if (next_work)
			memset(t_debug, 0x0, sizeof(t_debug));
		time_profile_result(ts);
#endif

		ts->work_sync_err_cnt = 0;
		post_proc = WORK_POST_COMPLATE;
		break;

	case WORK_POST_ERR_RETRY:
		ts->work_sync_err_cnt++;
		atomic_inc(&ts->next_work);
		queue_work(touch_wq, &ts->work);
		post_proc = WORK_POST_COMPLATE;
		break;

	case WORK_POST_ERR_CIRTICAL:
		ts->work_sync_err_cnt = 0;
		safety_reset(ts);
		touch_ic_init(ts);
		post_proc = WORK_POST_COMPLATE;
		break;

	default:
		post_proc = WORK_POST_COMPLATE;
		break;
	}

	if (post_proc != WORK_POST_COMPLATE)
		touch_work_post_proc(ts, post_proc);
}

/* touch_work_func_a
 *
 * HARD_TOUCH_KEY
 */
static void touch_work_func_a(struct work_struct *work)
{
	struct lge_touch_data *ts =
			container_of(work, struct lge_touch_data, work);
	u8 report_enable = 0;
	int ret = 0;

#ifdef CUST_G_TOUCH
	if (ts->pdata->role->ghost_detection_enable) {
		if(trigger_baseline==2){
			ret = ghost_detect_solution(ts);
			trigger_baseline = 0;
			touch_device_func->data(ts->client, &ts->ts_data);
			goto out;
		}
	}
#endif

	ret = touch_work_pre_proc(ts);

	if (ret == -EIO)
		goto err_out_critical;
	else if (ret == -EAGAIN)
		goto out;

#ifdef CUST_G_TOUCH
	else if (ret == -IGNORE_INTERRUPT)
		return;
#endif

#ifdef CUST_G_TOUCH
	/* Ghost detection solution */
	if (ts->pdata->role->ghost_detection_enable) {
		ret = ghost_detect_solution(ts);
		if(ret == NEED_TO_OUT)
			goto out;
		else if(ret == NEED_TO_INIT)
			goto err_out_init;
	}
#endif	

	/* Finger handle */
	if (ts->ts_data.state != TOUCH_ABS_LOCK) {
		if (!ts->ts_data.total_num) {
			touch_asb_input_report(ts, FINGER_RELEASED);
			report_enable = 1;

			queue_delayed_work(touch_wq, &ts->work_touch_lock, msecs_to_jiffies(200));

			if (likely(touch_debug_mask & (DEBUG_BASE_INFO | DEBUG_ABS))) {
				if (ts->ts_data.prev_total_num)
					check_log_finger_released(ts);
			}

			ts->ts_data.prev_total_num = 0;

			/* Reset previous finger position data */
			memset(&ts->ts_data.prev_data, 0x0, sizeof(ts->ts_data.prev_data));
		} else if (ts->ts_data.total_num <= ts->pdata->caps->max_id) {
			cancel_delayed_work_sync(&ts->work_touch_lock);

			if (ts->gf_ctrl.stage == GHOST_STAGE_CLEAR || (ts->gf_ctrl.stage | GHOST_STAGE_1) || ts->gf_ctrl.stage == GHOST_STAGE_4)
				ts->ts_data.state = TOUCH_BUTTON_LOCK;

			/* key button cancel */
			if(ts->ts_data.prev_button.state == BUTTON_PRESSED && ts->ts_data.state == TOUCH_BUTTON_LOCK) {
				input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_CANCLED);

				if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
					TOUCH_INFO_MSG("Touch KEY[%d] is canceled\n",
							ts->ts_data.prev_button.key_code);

				memset(&ts->ts_data.prev_button, 0x0, sizeof(ts->ts_data.prev_button));
			}

			if (likely(touch_debug_mask & (DEBUG_BASE_INFO | DEBUG_ABS)))
				check_log_finger_changed(ts, ts->ts_data.total_num);

			ts->ts_data.prev_total_num = ts->ts_data.total_num;

			touch_asb_input_report(ts, FINGER_PRESSED);
			report_enable = 1;

			memcpy(ts->ts_data.prev_data, ts->ts_data.curr_data, sizeof(ts->ts_data.curr_data));
		}

		/* Reset finger position data */
		memset(&ts->ts_data.curr_data, 0x0, sizeof(ts->ts_data.curr_data));

		if (report_enable)
			input_sync(ts->input_dev);
	}

	/* Button handle */
	if (ts->ts_data.state != TOUCH_BUTTON_LOCK) {
		/* do not check when there is no pressed button at error case
		 * 	- if you check it, sometimes touch is locked becuase button pressed via IC error.
		 */
		if (ts->work_sync_err_cnt > 0
				&& ts->ts_data.prev_button.state == BUTTON_RELEASED) {
			/* Do nothing */
		} else {
			report_enable = 0;

			if (unlikely(touch_debug_mask & DEBUG_BUTTON))
				TOUCH_INFO_MSG("Cur. button -code: %d state: %d, Prev. button -code: %d state: %d\n",
						ts->ts_data.curr_button.key_code,
						ts->ts_data.curr_button.state,
						ts->ts_data.prev_button.key_code,
						ts->ts_data.prev_button.state);

			if (ts->ts_data.curr_button.state == BUTTON_PRESSED
					&& ts->ts_data.prev_button.state == BUTTON_RELEASED) {
				/* button pressed */
				cancel_delayed_work_sync(&ts->work_touch_lock);

				input_report_key(ts->input_dev, ts->ts_data.curr_button.key_code, BUTTON_PRESSED);

				if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
					TOUCH_INFO_MSG("Touch KEY[%d] is pressed\n",
							ts->ts_data.curr_button.key_code);

				memcpy(&ts->ts_data.prev_button, &ts->ts_data.curr_button,
						sizeof(ts->ts_data.curr_button));

				report_enable = 1;
			}else if (ts->ts_data.curr_button.state == BUTTON_PRESSED
					&& ts->ts_data.prev_button.state == BUTTON_PRESSED
					&& ts->ts_data.prev_button.key_code != ts->ts_data.curr_button.key_code) {
				/* exception case - multi press button handle */
				queue_delayed_work(touch_wq, &ts->work_touch_lock, msecs_to_jiffies(200));

				/* release previous pressed button */
				input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_RELEASED);

				ts->ts_data.prev_button.state = BUTTON_RELEASED;

				if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
					TOUCH_INFO_MSG("Touch KEY[%d] is released\n",
							ts->ts_data.prev_button.key_code);

				report_enable = 1;
			} else if (ts->ts_data.curr_button.state == BUTTON_RELEASED /* button released */
					&& ts->ts_data.prev_button.state == BUTTON_PRESSED
					&& ts->ts_data.prev_button.key_code == ts->ts_data.curr_button.key_code) {
				/* button release */
				input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_RELEASED);

				if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
					TOUCH_INFO_MSG("Touch KEY[%d] is released\n",
							ts->ts_data.prev_button.key_code);

				memset(&ts->ts_data.prev_button, 0x0, sizeof(ts->ts_data.prev_button));
				memset(&ts->ts_data.curr_button, 0x0, sizeof(ts->ts_data.curr_button));

				report_enable = 1;
			}

			if (report_enable)
				input_sync(ts->input_dev);
		}
	}

out:
	touch_work_post_proc(ts, WORK_POST_OUT);
	return;

err_out_critical:
	touch_work_post_proc(ts, WORK_POST_ERR_CIRTICAL);
	return;

#ifdef CUST_G_TOUCH
err_out_init:
	touch_work_post_proc(ts, WORK_POST_ERR_CIRTICAL);
	return;
#endif	
}

static bool is_in_section(struct rect rt, u16 x, u16 y)
{
	return x >= rt.left && x <= rt.right && y >= rt.top && y <= rt.bottom;
}

static u16 find_button(const struct t_data data, const struct section_info sc)
{
	int i;

	if (is_in_section(sc.panel, data.x_position, data.y_position))
		return KEY_PANEL;

	for(i=0; i<sc.b_num; i++){
		if (is_in_section(sc.button[i], data.x_position, data.y_position))
			return sc.b_name[i];
	}

	return KEY_BOUNDARY;
}

static bool check_cancel(u16 button, u16 x, u16 y, const struct section_info sc)
{
	int i;

	for(i=0; i<sc.b_num; i++){
		if (sc.b_name[i] == button)
			break;
	}

	if (i < sc.b_num){
		if (is_in_section(sc.button_cancel[i], x, y))
			return false;
	}

	return true;
}

/* touch_work_func_b
 *
 * In order to reduce the booting-time,
 * we used delayed_work_queue instead of msleep or mdelay.
 */
static void touch_init_func(struct work_struct *work_init)
{
	struct lge_touch_data *ts =
			container_of(to_delayed_work(work_init),
					struct lge_touch_data, work_init);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	/* Specific device initialization */
	touch_ic_init(ts);
}

#ifdef LGE_TOUCH_POINT_DEBUG
static void dump_pointer_trace(void)
{
	int	i;

	printk("Single Touch Trace: Total Points %d in %dms (%dHz)\n",
		tr_last_index,
		tr_last_index > 1
			? (int)(tr_data[tr_last_index-1].time - tr_data[0].time)
			: 0,
		tr_last_index > 1
			? (tr_last_index * 1000) / (int)(tr_data[tr_last_index-1].time - tr_data[0].time)
			: 0);

	for (i = 0; i < tr_last_index; i++) {
		printk("(%x,%x,%d)",
			tr_data[i].x, tr_data[i].y,
			(i == 0) ? 0 : (int)(tr_data[i].time - tr_data[i-1].time));

		if ((i % 4) == 3)
			printk("\n");
	}

	printk("\n");

	tr_last_index = 0;
}
#endif

static void touch_input_report(struct lge_touch_data *ts)
{
	int	id;

	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (!ts->ts_data.curr_data[id].state)
			continue;

		input_mt_slot(ts->input_dev, id);
		input_mt_report_slot_state(ts->input_dev,
				ts->ts_data.curr_data[id].tool_type,
				ts->ts_data.curr_data[id].state != ABS_RELEASE);

		if (ts->ts_data.curr_data[id].state != ABS_RELEASE) {
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					ts->ts_data.curr_data[id].x_position);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					ts->ts_data.curr_data[id].y_position);
			if (is_pressure)
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					ts->ts_data.curr_data[id].pressure);

			/* Only support circle region */
			if (is_width_major)
				input_report_abs(ts->input_dev,
					ABS_MT_TOUCH_MAJOR,
					ts->ts_data.curr_data[id].width_major);

			if (is_width_minor)
				input_report_abs(ts->input_dev,
					ABS_MT_TOUCH_MINOR,
					ts->ts_data.curr_data[id].width_minor);
#ifdef LGE_TOUCH_POINT_DEBUG
			if (id == 0 && tr_last_index < MAX_TRACE) {
				tr_data[tr_last_index].x = ts->ts_data.curr_data[id].x_position;
				tr_data[tr_last_index].y = ts->ts_data.curr_data[id].y_position;
				tr_data[tr_last_index++].time = ktime_to_ms(ktime_get());
			}
#endif
		}
		else {
			ts->ts_data.curr_data[id].state = 0;
#ifdef LGE_TOUCH_POINT_DEBUG
			dump_pointer_trace();
#endif
		}
	}

	input_sync(ts->input_dev);
}

static struct double_tap_to_wake {
	unsigned long touch_time;
	unsigned long window_time;
	unsigned long sample_time_ms;
	unsigned int touches;
	struct input_dev *input_device;
} wake = {
	.touch_time = 0,
	.window_time = 0,
	.sample_time_ms = 100,
	.touches = 0,
};

void wake_up_display(struct input_dev *input_dev)
{
	wake.input_device = input_dev;
	return;
}

/*
 * Touch work function
 */
static void touch_work_func(struct work_struct *work)
{
	struct lge_touch_data *ts =
			container_of(work, struct lge_touch_data, work);
	int int_pin = 0;
	int next_work = 0;
	int ret;

	if (suspended && doubletap_to_wake)
	{
		if (!(wake.touch_time + 2000 >= ktime_to_ms(ktime_get())))
		{
			wake.touch_time = ktime_to_ms(ktime_get());
			wake.touches = 0;
		}
			
		if (!time_is_after_jiffies(
			wake.window_time + msecs_to_jiffies(wake.sample_time_ms)))
		{
			/*
			 * Don't count as touch when we release the touch input
			 */
			if (ts->ts_data.curr_data[0].state != ABS_RELEASE)
				++wake.touches;

			if (wake.touches == 2)
			{
				input_event(wake.input_device, EV_KEY, KEY_POWER, 1);
				input_event(wake.input_device, EV_SYN, 0, 0);
				msleep(100);
				input_event(wake.input_device, EV_KEY, KEY_POWER, 0);
				input_event(wake.input_device, EV_SYN, 0, 0);

				input_sync(wake.input_device);	
			}
		}

		wake.window_time = jiffies;
	} 

	atomic_dec(&ts->next_work);
	ts->ts_data.total_num = 0;

	if (unlikely(ts->work_sync_err_cnt >= MAX_RETRY_COUNT)) {
		TOUCH_ERR_MSG("Work Sync Failed: Irq-pin has some unknown problems\n");
		goto err_out_critical;
	}

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_WORKQUEUE_START]);
#endif
	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	ret = touch_device_func->data(ts->client, ts->ts_data.curr_data,
		&ts->ts_data.curr_button, &ts->ts_data.total_num);
	if (ret < 0) {
		if (ret == -EINVAL) /* Ignore the error */
			return;
		goto err_out_critical;
	}

	if (likely(ts->pdata->role->operation_mode == INTERRUPT_MODE))
		int_pin = gpio_get_value(ts->pdata->int_pin);

	/* Accuracy Solution */
	if (likely(ts->pdata->role->accuracy_filter_enable)) {
		if (accuracy_filter_func(ts) < 0)
			goto out;
	}

	/* Jitter Solution */
	if (likely(ts->pdata->role->jitter_filter_enable)) {
		if (jitter_filter_func(ts) < 0)
			goto out;
	}

	touch_input_report(ts);

out:
	if (likely(ts->pdata->role->operation_mode == INTERRUPT_MODE)) {
		next_work = atomic_read(&ts->next_work);

		if (unlikely(int_pin != 1 && next_work <= 0)) {
			TOUCH_INFO_MSG("WARN: Interrupt pin is low - "
					"next_work: %d, try_count: %d]\n",
					next_work, ts->work_sync_err_cnt);
			goto err_out_retry;
		}
	}

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_WORKQUEUE_END]);
	if (next_work)
		memset(t_debug, 0x0, sizeof(t_debug));
	time_profile_result(ts);
#endif
	ts->work_sync_err_cnt = 0;

	return;

err_out_retry:
	ts->work_sync_err_cnt++;
	atomic_inc(&ts->next_work);
	queue_work(touch_wq, &ts->work);

	return;

err_out_critical:
	ts->work_sync_err_cnt = 0;
	safety_reset(ts);
	touch_ic_init(ts);

	return;
}

/* touch_fw_upgrade_func
 *
 * it used to upgrade the firmware of touch IC.
 */
static void touch_fw_upgrade_func(struct work_struct *work_fw_upgrade)
{
	struct lge_touch_data *ts =
		container_of(work_fw_upgrade,
				struct lge_touch_data, work_fw_upgrade);
	u8 saved_state = ts->curr_pwr_state;

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (touch_device_func->fw_upgrade == NULL) {
		TOUCH_INFO_MSG("There is no specific firmware upgrade function\n");
		goto out;
	}

	if (likely(touch_debug_mask & (DEBUG_FW_UPGRADE | DEBUG_BASE_INFO)))
		TOUCH_INFO_MSG("fw_rev[%d:%d] product_id[%s:%s] force_upgrade[%d]\n",
			ts->fw_info.fw_rev, ts->fw_info.fw_image_rev,
			ts->fw_info.product_id, ts->fw_info.fw_image_product_id,
			ts->fw_upgrade.fw_force_upgrade);

	ts->fw_upgrade.is_downloading = UNDER_DOWNLOADING;

#ifdef CUST_G_TOUCH
	if(!strncmp(ts->fw_info.ic_fw_identifier, "DS4 R3.0", 8) && !ts->fw_info.fw_upgrade.fw_force_upgrade) {	//G's Obsolete PANEL
		TOUCH_INFO_MSG("DO NOT UPDATE 7020 gff, 7020 g2, 3203 g2 FW-upgrade is not executed\n");
		goto out;
	} else {
		if(ts->fw_info.fw_upgrade.fw_force_rework) {
			TOUCH_INFO_MSG("FW-upgrade Force Rework.\n");
		} else {
			TOUCH_INFO_MSG("ic %s img %s\n",ts->fw_info.ic_fw_version,ts->fw_info.syna_img_fw_version);
			if( ((int)simple_strtoul(&ts->fw_info.ic_fw_version[1], NULL, 10) >= 
				 (int)simple_strtoul(&ts->fw_info.syna_img_fw_version[1], NULL, 10))
				 && !ts->fw_info.fw_upgrade.fw_force_upgrade) {		   		
				TOUCH_INFO_MSG("FW-upgrade is not executed\n");
				goto out;
			} else {				
				TOUCH_INFO_MSG("FW-upgrade is executed\n");
			}
		}
	}
#else
	if ((!strcmp(ts->pdata->fw_version, ts->fw_info.ic_fw_version)
				|| ts->pdata->fw_version == NULL)
			&& !ts->fw_info.fw_upgrade.fw_force_upgrade){
		TOUCH_INFO_MSG("FW-upgrade is not executed\n");
		goto out;

	if (ts->curr_pwr_state == POWER_ON ||
				ts->curr_pwr_state == POWER_WAKE) {
		if (ts->pdata->role->operation_mode == INTERRUPT_MODE)
			disable_irq(ts->client->irq);
		else
			hrtimer_cancel(&ts->timer);
	}

	if (ts->curr_pwr_state == POWER_OFF) {
		touch_power_cntl(ts, POWER_ON);
		msleep(ts->pdata->role->booting_delay);
	}

	if (likely(touch_debug_mask & (DEBUG_FW_UPGRADE | DEBUG_BASE_INFO)))
		TOUCH_INFO_MSG("F/W upgrade - Start\n");

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_FW_UPGRADE_START]);
#endif

	if (touch_device_func->fw_upgrade(ts->client, ts->fw_upgrade.fw_path) < 0) {
		TOUCH_ERR_MSG("Firmware upgrade was failed\n");

		if (ts->curr_resume_state)
			if (ts->pdata->role->operation_mode == INTERRUPT_MODE)
				enable_irq(ts->client->irq);

		goto err_out;
	}

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_FW_UPGRADE_END]);
#endif

	if (!ts->curr_resume_state) {
		touch_power_cntl(ts, POWER_OFF);
	}
	else {
		if (ts->pdata->role->operation_mode == INTERRUPT_MODE)
			enable_irq(ts->client->irq);
		else
			hrtimer_start(&ts->timer,
				ktime_set(0, ts->pdata->role->report_period),
				HRTIMER_MODE_REL);

		touch_ic_init(ts);

		if (saved_state == POWER_WAKE || saved_state == POWER_SLEEP)
			touch_power_cntl(ts, saved_state);
	}

	if (likely(touch_debug_mask & (DEBUG_FW_UPGRADE |DEBUG_BASE_INFO)))
		TOUCH_INFO_MSG("F/W upgrade - Finish\n");

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_FW_UPGRADE_END]);

	if (touch_time_debug_mask & DEBUG_TIME_FW_UPGRADE ||
		touch_time_debug_mask & DEBUG_TIME_PROFILE_ALL) {
		TOUCH_INFO_MSG("FW upgrade time is under %3lu.%06lusec\n",
			get_time_interval(t_debug[TIME_FW_UPGRADE_END].tv_sec,
			t_debug[TIME_FW_UPGRADE_START].tv_sec),
			get_time_interval(t_debug[TIME_FW_UPGRADE_END].tv_usec,
			t_debug[TIME_FW_UPGRADE_START].tv_usec));
	}
#endif
	goto out;

err_out:
	safety_reset(ts);
	touch_ic_init(ts);

out:
	memset(&ts->fw_upgrade, 0, sizeof(ts->fw_upgrade));

#ifdef CUST_G_TOUCH
	/* Specific device resolution */
	if (touch_device_func->resolution) {
		if (touch_device_func->resolution(ts->client) < 0) {
			TOUCH_ERR_MSG("specific device resolution fail\n");
		}
	}
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->pdata->caps->x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->pdata->caps->y_max, 0, 0);
#endif
	memset(&ts->fw_info.fw_upgrade, 0, sizeof(ts->fw_info.fw_upgrade));
	return;
}

/* touch_irq_handler
 *
 * When Interrupt occurs, it will be called before touch_thread_irq_handler.
 *
 * return
 * IRQ_HANDLED: touch_thread_irq_handler will not be called.
 * IRQ_WAKE_THREAD: touch_thread_irq_handler will be called.
 */
static irqreturn_t touch_irq_handler(int irq, void *dev_id)
{
	struct lge_touch_data *ts = (struct lge_touch_data *)dev_id;

	if (unlikely(atomic_read(&ts->device_init) != 1))
		return IRQ_HANDLED;

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_ISR_START]);
#endif
	atomic_inc(&ts->next_work);

	queue_work(touch_wq, &ts->work);

	return IRQ_HANDLED;
}

/* touch_timer_handler
 *
 * it will be called when timer interrupt occurs.
 */
static enum hrtimer_restart touch_timer_handler(struct hrtimer *timer)
{
	struct lge_touch_data *ts =
			container_of(timer, struct lge_touch_data, timer);

	atomic_inc(&ts->next_work);
	queue_work(touch_wq, &ts->work);
	hrtimer_start(&ts->timer,
		ktime_set(0, ts->pdata->role->report_period), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

/* check_platform_data
 *
 * check-list
 * 1. Null Pointer
 * 2. lcd, touch screen size
 * 3. button support
 * 4. operation mode
 * 5. power module
 * 6. report period
 */
static int check_platform_data(struct touch_platform_data *pdata)
{
	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (!pdata)
		return -1;

	if (!pdata->caps || !pdata->role || !pdata->pwr)
		return -1;

	if (!pdata->caps->lcd_x || !pdata->caps->lcd_y ||
			!pdata->caps->x_max || !pdata->caps->y_max) {
		TOUCH_ERR_MSG("lcd_x, lcd_y, x_max, y_max are should be defined\n");
		return -1;
	}

	if (pdata->caps->button_support) {
		if (!pdata->role->key_type) {
			TOUCH_ERR_MSG("button_support = 1, but key_type is not defined\n");
			return -1;
		}

		if (!pdata->caps->y_button_boundary) {
			if (pdata->role->key_type == TOUCH_HARD_KEY)
				pdata->caps->y_button_boundary = pdata->caps->y_max;
			else
				pdata->caps->y_button_boundary =
					(pdata->caps->lcd_y * pdata->caps->x_max) /
						pdata->caps->lcd_x;
		}

		if (pdata->caps->button_margin < 0 ||
					pdata->caps->button_margin > 49) {
			pdata->caps->button_margin = 10;
			TOUCH_ERR_MSG("0 < button_margin < 49, "
					"button_margin is set 10 by force\n");
		}
	}

	if (pdata->role->operation_mode == POLLING_MODE) {
		if (!pdata->role->report_period) {
			TOUCH_ERR_MSG("polling mode needs report_period\n");
			return -1;
		}
	}

	if (pdata->role->suspend_pwr == POWER_OFF ||
			pdata->role->suspend_pwr == POWER_SLEEP) {
		if (pdata->role->suspend_pwr == POWER_OFF)
			pdata->role->resume_pwr = POWER_ON;
		else
			pdata->role->resume_pwr = POWER_WAKE;
	} else {
		TOUCH_ERR_MSG("suspend_pwr = POWER_OFF or POWER_SLEEP\n");
	}

	if (pdata->pwr->use_regulator) {
		if (!pdata->pwr->vdd[0] || !pdata->pwr->vio[0]) {
			TOUCH_ERR_MSG("you should assign the name of vdd and vio\n");
			return -1;
		}
	} else {
		if (!pdata->pwr->power) {
			TOUCH_ERR_MSG("you should assign the power-control-function\n");
			return -1;
		}
	}

	if (pdata->role->report_period == 0)
		pdata->role->report_period = 12500000;

	return 0;
}

/* get_section
 *
 * it calculates the area of touch-key, automatically.
 */
void get_section(struct section_info* sc, struct touch_platform_data *pdata)
{
	int i;

	sc->panel.left = 0;
	sc->panel.right = pdata->caps->x_max;
	sc->panel.top = 0;
	sc->panel.bottom = pdata->caps->y_button_boundary;

	if (pdata->caps->button_support) {
		sc->b_width  = pdata->caps->x_max /
					pdata->caps->number_of_button;
		sc->b_margin = sc->b_width * pdata->caps->button_margin / 100;
		sc->b_inner_width = sc->b_width - (2 * sc->b_margin);
		sc->b_height = pdata->caps->y_max -
					pdata->caps->y_button_boundary;
		sc->b_num = pdata->caps->number_of_button;

		for (i = 0; i < sc->b_num; i++) {
			sc->button[i].left = i * (pdata->caps->x_max /
				pdata->caps->number_of_button) + sc->b_margin;
			sc->button[i].right = sc->button[i].left +
							sc->b_inner_width;
			sc->button[i].top = pdata->caps->y_button_boundary + 1;
			sc->button[i].bottom = pdata->caps->y_max;

			sc->button_cancel[i].left = sc->button[i].left - (2 * sc->b_margin) >= 0 ?  sc->button[i].left - (2 * sc->b_margin) : 0;
			sc->button_cancel[i].right = sc->button[i].right +
				(2 * sc->b_margin) <= pdata->caps->x_max ?
				sc->button[i].right + (2 * sc->b_margin)
				: pdata->caps->x_max;
			sc->button_cancel[i].top = sc->button[i].top;
			sc->button_cancel[i].bottom = sc->button[i].bottom;

			sc->b_name[i] = pdata->caps->button_name[i];
		}
	}
}

/* Sysfs
 *
 * For debugging easily, we added some sysfs.
 */
static ssize_t show_platform_data(struct lge_touch_data *ts, char *buf)
{
	struct touch_platform_data *pdata = ts->pdata;
	int ret = 0;

	ret = sprintf(buf, "====== Platform data ======\n");
	ret += sprintf(buf+ret, "int_pin[%d] reset_pin[%d]\n",
			pdata->int_pin, pdata->reset_pin);
	ret += sprintf(buf+ret, "caps:\n");
	if (pdata->caps->button_support) {
		ret += sprintf(buf+ret, "\tbutton_support        = %d\n",
			pdata->caps->button_support);
		ret += sprintf(buf+ret, "\ty_button_boundary     = %d\n",
			pdata->caps->y_button_boundary);
		ret += sprintf(buf+ret, "\tbutton_margin         = %d\n",
			pdata->caps->button_margin);
		ret += sprintf(buf+ret, "\tnumber_of_button      = %d\n",
			pdata->caps->number_of_button);
		ret += sprintf(buf+ret, "\tbutton_name           = %d, %d, %d, %d\n",
			pdata->caps->button_name[0],
			pdata->caps->button_name[1],
			pdata->caps->button_name[2],
			pdata->caps->button_name[3]);
	}
	ret += sprintf(buf+ret, "\tis_width_major_supported    = %d\n",
			pdata->caps->is_width_major_supported);
	ret += sprintf(buf+ret, "\tis_width_minor_supported    = %d\n",
			pdata->caps->is_width_minor_supported);
	ret += sprintf(buf+ret, "\tis_pressure_supported = %d\n",
			pdata->caps->is_pressure_supported);
	ret += sprintf(buf+ret, "\tis_id_supported       = %d\n",
			pdata->caps->is_id_supported);
	ret += sprintf(buf+ret, "\tmax_width_major       = %d\n",
			pdata->caps->max_width_major);
	ret += sprintf(buf+ret, "\tmax_width_minor       = %d\n",
			pdata->caps->max_width_minor);
	ret += sprintf(buf+ret, "\tmax_pressure          = %d\n",
			pdata->caps->max_pressure);
	ret += sprintf(buf+ret, "\tmax_id                = %d\n",
			pdata->caps->max_id);
	ret += sprintf(buf+ret, "\tx_max                 = %d\n",
			pdata->caps->x_max);
	ret += sprintf(buf+ret, "\ty_max                 = %d\n",
			pdata->caps->y_max);
	ret += sprintf(buf+ret, "\tlcd_x                 = %d\n",
			pdata->caps->lcd_x);
	ret += sprintf(buf+ret, "\tlcd_y                 = %d\n",
			pdata->caps->lcd_y);
	ret += sprintf(buf+ret, "role:\n");
	ret += sprintf(buf+ret, "\toperation_mode        = %d\n",
			pdata->role->operation_mode);
	ret += sprintf(buf+ret, "\tkey_type              = %d\n",
			pdata->role->key_type);
	ret += sprintf(buf+ret, "\treport_mode           = %d\n",
			pdata->role->report_mode);
	ret += sprintf(buf+ret, "\tdelta_pos_threshold   = %d\n",
			pdata->role->delta_pos_threshold);
	ret += sprintf(buf+ret, "\torientation           = %d\n",
			pdata->role->orientation);
	ret += sprintf(buf+ret, "\treport_period         = %d\n",
			pdata->role->report_period);
	ret += sprintf(buf+ret, "\tbooting_delay         = %d\n",
			pdata->role->booting_delay);
	ret += sprintf(buf+ret, "\treset_delay           = %d\n",
			pdata->role->reset_delay);
	ret += sprintf(buf+ret, "\tsuspend_pwr           = %d\n",
			pdata->role->suspend_pwr);
	ret += sprintf(buf+ret, "\tresume_pwr            = %d\n",
			pdata->role->resume_pwr);
	ret += sprintf(buf+ret, "\tirqflags              = 0x%lx\n",
			pdata->role->irqflags);
	ret += sprintf(buf+ret, "\tshow_touches          = %d\n",
			pdata->role->show_touches);
	ret += sprintf(buf+ret, "\tpointer_location      = %d\n",
			pdata->role->pointer_location);
	ret += sprintf(buf+ret, "pwr:\n");
	ret += sprintf(buf+ret, "\tuse_regulator         = %d\n",
			pdata->pwr->use_regulator);
	ret += sprintf(buf+ret, "\tvdd                   = %s\n",
			pdata->pwr->vdd);
	ret += sprintf(buf+ret, "\tvdd_voltage           = %d\n",
			pdata->pwr->vdd_voltage);
	ret += sprintf(buf+ret, "\tvio                   = %s\n",
			pdata->pwr->vio);
	ret += sprintf(buf+ret, "\tvio_voltage           = %d\n",
			pdata->pwr->vio_voltage);
	ret += sprintf(buf+ret, "\tpower                 = %s\n",
			pdata->pwr->power ? "YES" : "NO");
	return ret;
}

/* show_fw_info
 *
 * show only the firmware information
 */
static ssize_t show_fw_info(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "====== Firmware Info ======\n");
	ret += sprintf(buf+ret, "manufacturer_id  = %d\n",
			ts->fw_info.manufacturer_id);
	ret += sprintf(buf+ret, "product_id       = %s\n",
			ts->fw_info.product_id);
	ret += sprintf(buf+ret, "fw_version       = %s\n",
			ts->fw_info.fw_version);
	ret += sprintf(buf+ret, "fw_image_product_id = %s\n",
			ts->fw_info.fw_image_product_id);
	ret += sprintf(buf+ret, "fw_image_version = %s\n",
			ts->fw_info.fw_image_version);
	return ret;
}

/* store_fw_upgrade
 *
 * User can upgrade firmware, anytime, using this module.
 * Also, user can use both binary-img(SDcard) and header-file(Kernel image).
 */
static ssize_t store_fw_upgrade(struct lge_touch_data *ts,
					const char *buf, size_t count)
{
	int value = 0;
	int repeat = 0;
	char path[256] = {0};

	sscanf(buf, "%d %s", &value, path);

	printk(KERN_INFO "\n");
	TOUCH_INFO_MSG("Firmware image path: %s\n",
				path[0] != 0 ? path : "Internal");

	if (value) {
		for (repeat = 0; repeat < value; repeat++) {
			/* sync for n-th repeat test */
			while (ts->fw_upgrade.is_downloading);

			msleep(ts->pdata->role->booting_delay * 2);
			printk(KERN_INFO "\n");
			TOUCH_INFO_MSG("Firmware image upgrade: No.%d",
								repeat+1);

			/* for n-th repeat test
			 * because ts->fw_upgrade is setted 0 after
			 * FW upgrade */
			memcpy(ts->fw_upgrade.fw_path, path,
					sizeof(ts->fw_upgrade.fw_path)-1);

			/* set downloading flag for sync for n-th test */
			ts->fw_upgrade.is_downloading = UNDER_DOWNLOADING;
			ts->fw_upgrade.fw_force_upgrade = 1;

			queue_work(touch_wq, &ts->work_fw_upgrade);
		}

		/* sync for fw_upgrade test */
		while (ts->fw_upgrade.is_downloading);
	}

	return count;
}

/* show_fw_ver
 *
 * show only firmware version.
 * It will be used for AT-COMMAND
 */
static ssize_t show_fw_ver(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%s\n", ts->fw_info.fw_version);
	return ret;
}

/* store_ts_reset
 *
 * Reset the touch IC.
 */
static ssize_t store_ts_reset(struct lge_touch_data *ts,
					const char *buf, size_t count)
{
	unsigned char string[5];
	u8 saved_state = ts->curr_pwr_state;
	int ret = 0;

	sscanf(buf, "%s", string);

	if (ts->pdata->role->operation_mode == INTERRUPT_MODE)
		disable_irq_nosync(ts->client->irq);
	else
		hrtimer_cancel(&ts->timer);

	cancel_work_sync(&ts->work);
	cancel_delayed_work_sync(&ts->work_init);
	if (ts->pdata->role->key_type == TOUCH_HARD_KEY)
		cancel_delayed_work_sync(&ts->work_touch_lock);

	release_all_ts_event(ts);

	if (saved_state == POWER_ON || saved_state == POWER_WAKE) {
		if (!strncmp(string, "soft", 4)) {
			if (touch_device_func->ic_ctrl)
				touch_device_func->ic_ctrl(ts->client,
							IC_CTRL_RESET_CMD, 0);
			else
				TOUCH_INFO_MSG("There is no specific IC control function\n");
		} else if (!strncmp(string, "pin", 3)) {
			if (ts->pdata->reset_pin > 0) {
				gpio_set_value(ts->pdata->reset_pin, 0);
				msleep(ts->pdata->role->reset_delay);
				gpio_set_value(ts->pdata->reset_pin, 1);
			} else {
				TOUCH_INFO_MSG("There is no reset pin\n");
			}
		} else if (!strncmp(string, "vdd", 3)) {
			touch_power_cntl(ts, POWER_OFF);
			touch_power_cntl(ts, POWER_ON);
		} else {
			TOUCH_INFO_MSG("Usage: echo [soft | pin | vdd] > ts_reset\n");
			TOUCH_INFO_MSG(" - soft : reset using IC register setting\n");
			TOUCH_INFO_MSG(" - soft : reset using reset pin\n");
			TOUCH_INFO_MSG(" - hard : reset using VDD\n");
		}

		if (ret < 0)
			TOUCH_ERR_MSG("reset fail\n");
		else
			atomic_set(&ts->device_init, 0);

		msleep(ts->pdata->role->booting_delay);

	} else {
		TOUCH_INFO_MSG("Touch is suspend state. Don't need reset\n");
	}

	if (ts->pdata->role->operation_mode == INTERRUPT_MODE)
		enable_irq(ts->client->irq);
	else
		hrtimer_start(&ts->timer,
			ktime_set(0, ts->pdata->role->report_period),
					HRTIMER_MODE_REL);

	if (saved_state == POWER_ON || saved_state == POWER_WAKE)
		touch_ic_init(ts);

	return count;
}

/* ic_register_ctrl
 *
 * User can see any register of touch_IC
 */
static ssize_t ic_register_ctrl(struct lge_touch_data *ts,
					const char *buf, size_t count)
{
	unsigned char string[6];
	int reg = 0;
	int value = 0;
	int ret = 0;
	u32 write_data;

	sscanf(buf, "%s %d %d", string, &reg, &value);

	if (touch_device_func->ic_ctrl) {
		if (ts->curr_pwr_state == POWER_ON ||
					ts->curr_pwr_state == POWER_WAKE) {
			if (!strncmp(string, "read", 4)) {
				do {
					ret = touch_device_func->ic_ctrl(ts->client, IC_CTRL_READ, reg);
					if (ret >= 0) {
						TOUCH_INFO_MSG("register[0x%x] = 0x%x\n", reg, ret);
					} else {
						TOUCH_INFO_MSG("cannot read register[0x%x]\n", reg);
					}
					reg++;
				} while (--value > 0);
			} else if (!strncmp(string, "write", 4)) {
				write_data = ((0xFF & reg) << 8) | (0xFF & value);
				ret = touch_device_func->ic_ctrl(ts->client,
						IC_CTRL_WRITE, write_data);
				if (ret >= 0) {
					TOUCH_INFO_MSG("register[0x%x] is set to 0x%x\n", reg, value);
				} else {
					TOUCH_INFO_MSG("cannot write register[0x%x]\n", reg);
				}
			} else {
				TOUCH_INFO_MSG("Usage: echo [read | write] "
						"reg_num value > ic_rw\n");
				TOUCH_INFO_MSG(" - reg_num : register address\n");
				TOUCH_INFO_MSG(" - value [read] : number of register "
						"starting form reg_num\n");
				TOUCH_INFO_MSG(" - value [write] : set value into reg_num\n");
			}
		} else {
			TOUCH_INFO_MSG("state=[suspend]. we cannot use I2C, now\n");
		}
	} else {
		TOUCH_INFO_MSG("There is no specific IC control function\n");
	}

	return count;
}

static ssize_t store_jitter_solution(struct lge_touch_data *ts,
						const char *buf, size_t count)
{
	int ret = 0;

	memset(&ts->jitter_filter, 0, sizeof(ts->jitter_filter));

	ret = sscanf(buf, "%d %d", &ts->pdata->role->jitter_filter_enable,
				&ts->jitter_filter.adjust_margin);

	return count;
}

static ssize_t store_accuracy_solution(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int ret = 0;

	memset(&ts->accuracy_filter, 0, sizeof(ts->accuracy_filter));

	ret = sscanf(buf, "%d %d %d %d %d %d %d",
				&ts->pdata->role->accuracy_filter_enable,
				&ts->accuracy_filter.ignore_pressure_gap,
				&ts->accuracy_filter.delta_max,
				&ts->accuracy_filter.touch_max_count,
				&ts->accuracy_filter.max_pressure,
				&ts->accuracy_filter.direction_count,
				&ts->accuracy_filter.time_to_max_pressure);

	return count;
}

/* show_show_touches
 *
 * User can check the information of show_touches, using this module.
 */
static ssize_t show_show_touches(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", ts->pdata->role->show_touches);

	return ret;
}

/* store_show_touches
 *
 * This function is related with show_touches in framework.
 */
static ssize_t store_show_touches(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);

	ts->pdata->role->show_touches = value;

	return count;
}

/* show_pointer_location
 *
 * User can check the information of pointer_location, using this module.
 */
static ssize_t show_pointer_location(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", ts->pdata->role->pointer_location);

	return ret;
}

/* store_pointer_location
 *
 * This function is related with pointer_location in framework.
 */
static ssize_t store_pointer_location(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);

	ts->pdata->role->pointer_location = value;

	return count;
}

/* show_charger
 *
 * Show the current charger status
 */
static ssize_t show_charger(struct lge_touch_data *ts, char *buf)
{
	return sprintf(buf, "%d\n", ts->charger_type);
}

static LGE_TOUCH_ATTR(platform_data, S_IRUGO | S_IWUSR, show_platform_data, NULL);
static LGE_TOUCH_ATTR(firmware, S_IRUGO | S_IWUSR, show_fw_info, store_fw_upgrade);
static LGE_TOUCH_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_fw_ver, NULL);
static LGE_TOUCH_ATTR(reset, S_IRUGO | S_IWUSR, NULL, store_ts_reset);
static LGE_TOUCH_ATTR(ic_rw, S_IRUGO | S_IWUSR, NULL, ic_register_ctrl);
static LGE_TOUCH_ATTR(jitter, S_IRUGO | S_IWUSR, NULL, store_jitter_solution);
static LGE_TOUCH_ATTR(accuracy, S_IRUGO | S_IWUSR, NULL, store_accuracy_solution);
static LGE_TOUCH_ATTR(show_touches, S_IRUGO | S_IWUSR, show_show_touches, store_show_touches);
static LGE_TOUCH_ATTR(pointer_location, S_IRUGO | S_IWUSR, show_pointer_location,
					store_pointer_location);
static LGE_TOUCH_ATTR(charger, S_IRUGO | S_IWUSR, show_charger, NULL);

static struct attribute *lge_touch_attribute_list[] = {
	&lge_touch_attr_platform_data.attr,
	&lge_touch_attr_firmware.attr,
	&lge_touch_attr_fw_ver.attr,
	&lge_touch_attr_reset.attr,
	&lge_touch_attr_ic_rw.attr,
	&lge_touch_attr_jitter.attr,
	&lge_touch_attr_accuracy.attr,
	&lge_touch_attr_show_touches.attr,
	&lge_touch_attr_pointer_location.attr,
	&lge_touch_attr_charger.attr,
	NULL,
};

/* lge_touch_attr_show / lge_touch_attr_store
 *
 * sysfs bindings for lge_touch
 */
static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj, struct attribute *attr,
			     char *buf)
{
	struct lge_touch_data *ts = container_of(lge_touch_kobj,
					struct lge_touch_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(ts, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	struct lge_touch_data *ts = container_of(lge_touch_kobj,
				struct lge_touch_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(ts, buf, count);

	return ret;
}

static const struct sysfs_ops lge_touch_sysfs_ops = {
	.show	= lge_touch_attr_show,
	.store	= lge_touch_attr_store,
};

static struct kobj_type lge_touch_kobj_type = {
	.sysfs_ops	= &lge_touch_sysfs_ops,
	.default_attrs 	= lge_touch_attribute_list,
};

static struct sysdev_class lge_touch_sys_class = {
	.name = LGE_TOUCH_NAME,
};

static struct sys_device lge_touch_sys_device = {
	.id	= 0,
	.cls	= &lge_touch_sys_class,
};

#ifdef CONFIG_TOUCHSCREEN_CHARGER_NOTIFY
static void touch_external_power_changed(struct power_supply *psy)
{
	int status;
	struct lge_touch_data *ts = container_of(psy, struct lge_touch_data, touch_psy);

	status = power_supply_am_i_supplied(psy);

	pr_debug("psy name = %s ... status = %d\n", psy->name, status);

	if (ts->charger_type != status) {
		ts->charger_type = status;
		queue_work(touch_wq, &ts->work_charger);
	}
}

static enum power_supply_property touch_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE
};

static int touch_power_get_property_mains(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		return 0;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

/*
 * Touch work for charger
 */
static void touch_work_charger(struct work_struct *work)
{
	struct lge_touch_data *ts =
			container_of(work, struct lge_touch_data, work_charger);

	TOUCH_INFO_MSG("CHARGER = %d\n", ts->charger_type);

	if (ts->curr_resume_state)
		touch_device_func->ic_ctrl(ts->client,
					IC_CTRL_CHARGER, ts->charger_type);
}

static void touch_psy_init(struct lge_touch_data *ts)
{
	int rc;

	INIT_WORK(&ts->work_charger, touch_work_charger);

	ts->touch_psy.name = "touch";
	ts->touch_psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
	ts->touch_psy.properties = touch_power_props_mains,
	ts->touch_psy.num_properties = ARRAY_SIZE(touch_power_props_mains);
	ts->touch_psy.get_property = touch_power_get_property_mains;
	ts->touch_psy.external_power_changed = touch_external_power_changed;

	rc = power_supply_register(NULL, &ts->touch_psy);
	if (rc < 0) {
		pr_err("power_supply_register FAILED*** rc = %d\n", rc);
	}
	else {
		pr_debug("power_supply_register SUCCESS\n");
	}
}
#endif

static int touch_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct lge_touch_data *ts;
	int ret = 0;
	int one_sec = 0;

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TOUCH_ERR_MSG("i2c functionality check error\n");
		ret = -EPERM;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct lge_touch_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_ERR_MSG("Can not allocate memory\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->pdata = client->dev.platform_data;
	ret = check_platform_data(ts->pdata);
	if (ret < 0) {
		TOUCH_ERR_MSG("Can not read platform data\n");
		ret = -EINVAL;
		goto err_assign_platform_data;
	}

	one_sec = 1000000 / (ts->pdata->role->report_period / 1000);
	ts->ic_init_err_cnt = 0;
	ts->work_sync_err_cnt = 0;
	ts->gf_ctrl.min_count = 75000 / (ts->pdata->role->report_period / 1000);
	ts->gf_ctrl.max_count = one_sec;
	get_section(&ts->st_info, ts->pdata);

	ts->client = client;

#ifdef CUST_G_TOUCH
	ds4_i2c_client = client;

	ts->fw_info.fw_upgrade.fw_force_rework = false;
#endif
	i2c_set_clientdata(client, ts);

	ts->fw_info.fw_force_rework = false;

	/* Specific device probe */
	if (touch_device_func->probe) {
		ret = touch_device_func->probe(client);
		if (ret < 0) {
			TOUCH_ERR_MSG("specific device probe fail\n");
			goto err_assign_platform_data;
		}
	}

	/* reset pin setting */
	if (ts->pdata->reset_pin > 0) {
		ret = gpio_request(ts->pdata->reset_pin, "touch_reset");
		if (ret < 0) {
			TOUCH_ERR_MSG("FAIL: touch_reset gpio_request\n");
			goto err_assign_platform_data;
		}
		gpio_direction_output(ts->pdata->reset_pin, 0);
	}

	atomic_set(&ts->device_init, 0);
	ts->curr_resume_state = 1;

	/* Power on */
	if (touch_power_cntl(ts, POWER_ON) < 0)
		goto err_power_failed;

	msleep(ts->pdata->role->booting_delay);

	/* init work_queue */
	INIT_WORK(&ts->work, touch_work_func);

	INIT_DELAYED_WORK(&ts->work_init, touch_init_func);
	INIT_WORK(&ts->work_fw_upgrade, touch_fw_upgrade_func);

	/* input dev setting */
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		TOUCH_ERR_MSG("Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "touch_dev";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#endif

	if (ts->pdata->caps->button_support) {
		set_bit(EV_KEY, ts->input_dev->evbit);
		for (ret = 0; ret < ts->pdata->caps->number_of_button; ret++) {
			set_bit(ts->pdata->caps->button_name[ret], ts->input_dev->keybit);
		}
	}

	if (ts->pdata->caps->max_id > MAX_FINGER) {
		ts->pdata->caps->max_id = MAX_FINGER;
	}

	input_mt_init_slots(ts->input_dev, ts->pdata->caps->max_id);
	input_set_abs_params(ts->input_dev,
			ABS_MT_POSITION_X, 0, ts->pdata->caps->x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
			ts->pdata->caps->y_button_boundary
			? ts->pdata->caps->y_button_boundary
			: ts->pdata->caps->y_max,
			0, 0);

	/* Copy for efficient handling */
	is_pressure = ts->pdata->caps->is_pressure_supported;
	is_width_major = ts->pdata->caps->is_width_major_supported;
	is_width_minor = ts->pdata->caps->is_width_minor_supported;

	if (is_pressure)
		input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,
				0, ts->pdata->caps->max_pressure, 0, 0);

	if (is_width_major)
		input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				0, ts->pdata->caps->max_width_major, 0, 0);

	if (is_width_minor)
		input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR,
				0, ts->pdata->caps->max_width_minor, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret < 0) {
		TOUCH_ERR_MSG("Unable to register %s input device\n",
				ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	if (ts->pdata->role->operation_mode == INTERRUPT_MODE) {
		ret = gpio_request(ts->pdata->int_pin, "touch_int");
		if (ret < 0) {
			TOUCH_ERR_MSG("FAIL: touch_int gpio_request\n");
			goto err_interrupt_failed;
		}
		gpio_direction_input(ts->pdata->int_pin);

		ret = request_threaded_irq(client->irq, touch_irq_handler,
				NULL,
				ts->pdata->role->irqflags | IRQF_ONESHOT,
				client->name, ts);

		if (ret < 0) {
			TOUCH_ERR_MSG("request_irq failed. use polling mode\n");
			gpio_free(ts->pdata->int_pin);
			goto err_interrupt_failed;
		}
	} else {	/* polling mode */
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = touch_timer_handler;
		hrtimer_start(&ts->timer,
			ktime_set(0, ts->pdata->role->report_period),
					HRTIMER_MODE_REL);
	}

	/* Specific device initialization */
	touch_ic_init(ts);

	/* Firmware Upgrade Check - use thread for booting time reduction */
	if (touch_device_func->fw_upgrade) {
		queue_work(touch_wq, &ts->work_fw_upgrade);
	}

	/* jitter solution */
	if (ts->pdata->role->jitter_filter_enable) {
		ts->jitter_filter.adjust_margin = 100;
	}

	/* accuracy solution */
	if (ts->pdata->role->accuracy_filter_enable) {
		ts->accuracy_filter.ignore_pressure_gap = 5;
		ts->accuracy_filter.delta_max = 100;
		ts->accuracy_filter.max_pressure = 255;
		ts->accuracy_filter.time_to_max_pressure = one_sec / 20;
		ts->accuracy_filter.direction_count = one_sec / 6;
		ts->accuracy_filter.touch_max_count = one_sec / 2;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = touch_early_suspend;
	ts->early_suspend.resume = touch_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	/* Register sysfs for making fixed communication path to framework layer */
	ret = sysdev_class_register(&lge_touch_sys_class);
	if (ret < 0) {
		TOUCH_ERR_MSG("sysdev_class_register is failed\n");
		goto err_lge_touch_sys_class_register;
	}

	ret = sysdev_register(&lge_touch_sys_device);
	if (ret < 0) {
		TOUCH_ERR_MSG("sysdev_register is failed\n");
		goto err_lge_touch_sys_dev_register;
	}

	ret = kobject_init_and_add(&ts->lge_touch_kobj, &lge_touch_kobj_type,
			ts->input_dev->dev.kobj.parent,
			"%s", LGE_TOUCH_NAME);
	if (ret < 0) {
		TOUCH_ERR_MSG("kobject_init_and_add is failed\n");
		goto err_lge_touch_sysfs_init_and_add;
	}

	if (likely(touch_debug_mask & DEBUG_BASE_INFO))
		TOUCH_INFO_MSG("Touch driver is initialized\n");

#ifdef CONFIG_TOUCHSCREEN_CHARGER_NOTIFY
	touch_psy_init(ts);
#endif

	_ts = ts;

	return 0;

err_lge_touch_sysfs_init_and_add:
	sysdev_unregister(&lge_touch_sys_device);
err_lge_touch_sys_dev_register:
	sysdev_class_unregister(&lge_touch_sys_class);
err_lge_touch_sys_class_register:
	unregister_early_suspend(&ts->early_suspend);
	if (ts->pdata->role->operation_mode == INTERRUPT_MODE) {
		gpio_free(ts->pdata->int_pin);
		free_irq(ts->client->irq, ts);
	}
err_interrupt_failed:
	input_unregister_device(ts->input_dev);
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	touch_power_cntl(ts, POWER_OFF);
err_power_failed:
err_assign_platform_data:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int touch_remove(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	/* Specific device remove */
	if (touch_device_func->remove)
		touch_device_func->remove(ts->client);

	/* Power off */
	touch_power_cntl(ts, POWER_OFF);

	kobject_del(&ts->lge_touch_kobj);
	sysdev_unregister(&lge_touch_sys_device);
	sysdev_class_unregister(&lge_touch_sys_class);

	unregister_early_suspend(&ts->early_suspend);

	if (ts->pdata->role->operation_mode == INTERRUPT_MODE) {
		gpio_free(ts->pdata->int_pin);
		free_irq(client->irq, ts);
	}
	else {
		hrtimer_cancel(&ts->timer);
	}

	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void touch_early_suspend(struct early_suspend *h)
{
	struct lge_touch_data *ts =
			container_of(h, struct lge_touch_data, early_suspend);

	suspended = true;

	ts->curr_resume_state = 0;

	if (ts->fw_upgrade.is_downloading == UNDER_DOWNLOADING) {
		TOUCH_INFO_MSG("early_suspend is not executed\n");
		return;
	}

	if (doubletap_to_wake)
	{
		enable_irq_wake(ts->client->irq);
	}
	else 
	{
		if (ts->pdata->role->operation_mode == INTERRUPT_MODE)
                disable_irq(ts->client->irq);
        else
                hrtimer_cancel(&ts->timer);

        cancel_work_sync(&ts->work);
        cancel_delayed_work_sync(&ts->work_init);
        if (ts->pdata->role->key_type == TOUCH_HARD_KEY)
                cancel_delayed_work_sync(&ts->work_touch_lock);

        release_all_ts_event(ts);

        touch_power_cntl(ts, ts->pdata->role->suspend_pwr);

	}
}

static void touch_late_resume(struct early_suspend *h)
{
	struct lge_touch_data *ts =
			container_of(h, struct lge_touch_data, early_suspend);

	suspended = false;

	ts->curr_resume_state = 1;

	if (ts->fw_upgrade.is_downloading == UNDER_DOWNLOADING) {
		TOUCH_INFO_MSG("late_resume is not executed\n");
		return;
	}

	if (doubletap_to_wake)
	{
		disable_irq_wake(ts->client->irq);
	}
	else
	{
		touch_power_cntl(ts, ts->pdata->role->resume_pwr);

        if (ts->pdata->role->operation_mode == INTERRUPT_MODE)
                enable_irq(ts->client->irq);
        else
                hrtimer_start(&ts->timer,
                        ktime_set(0, ts->pdata->role->report_period),
                                        HRTIMER_MODE_REL);

        if (ts->pdata->role->resume_pwr == POWER_ON)
                queue_delayed_work(touch_wq, &ts->work_init,
                        msecs_to_jiffies(ts->pdata->role->booting_delay));
        else
                queue_delayed_work(touch_wq, &ts->work_init, 0);
	}
}
#endif

#if defined(CONFIG_PM)
static int touch_suspend(struct device *device)
{
	return 0;
}

static int touch_resume(struct device *device)
{
	return 0;
}
#endif

static struct i2c_device_id lge_ts_id[] = {
	{LGE_TOUCH_NAME, 0 },
};

#if defined(CONFIG_PM)
static struct dev_pm_ops touch_pm_ops = {
	.suspend = touch_suspend,
	.resume = touch_resume,
};
#endif

static struct i2c_driver lge_touch_driver = {
	.probe = touch_probe,
	.remove = touch_remove,
	.id_table = lge_ts_id,
	.driver = {
		.name = LGE_TOUCH_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_PM)
		.pm = &touch_pm_ops,
#endif
	},
};

int touch_driver_register(struct touch_device_driver* driver)
{
	int ret = 0;

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (touch_device_func != NULL) {
		TOUCH_ERR_MSG("CANNOT add new touch-driver\n");
		ret = -EMLINK;
		goto err_touch_driver_register;
	}

	touch_device_func = driver;

	touch_wq = alloc_workqueue("touch_wq", 0, 1);
	if (!touch_wq) {
		TOUCH_ERR_MSG("CANNOT create new workqueue\n");
		ret = -ENOMEM;
		goto err_work_queue;
	}

	ret = i2c_add_driver(&lge_touch_driver);
	if (ret < 0) {
		TOUCH_ERR_MSG("FAIL: i2c_add_driver\n");
		goto err_i2c_add_driver;
	}

	return 0;

err_i2c_add_driver:
	destroy_workqueue(touch_wq);
err_work_queue:
err_touch_driver_register:
	return ret;
}

void touch_driver_unregister(void)
{
	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	i2c_del_driver(&lge_touch_driver);
	touch_device_func = NULL;

	if (touch_wq)
		destroy_workqueue(touch_wq);
}

