#!/bin/bash

# python3 -B main_opt_NLOPT.py --model_name 3D_model_w_whip_T4 --vid_off --run_time 2.1 --start_time 0.0
# python3 -B main_opt_NLOPT.py --model_name 3D_model_w_whip_T5 --vid_off --run_time 2.1 --start_time 0.0
# python3 -B main_opt_NLOPT.py --model_name 3D_model_w_whip_T6 --vid_off --run_time 2.1 --start_time 0.0

# python3 -B main_opt_NLOPT.py --start_time 0 --run_time 5.0 --model_name 2D_model_w_whip_N20 --vid_off --opt_type 1
# python3 -B main_opt_NLOPT.py --start_time 0 --run_time 5.0 --model_name 2D_model_w_whip_N20 --vid_off --opt_type 2


python3 -B main_opt_NLOPT2.py --start_time 0 --model_name 3D_model_w_whip_T6 --vid_off --opt_type 1
# python3 -B main_opt_NLOPT2.py --start_time 0 --model_name 3D_model_w_whip_T6 --vid_off --opt_type 2
