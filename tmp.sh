#!/bin/bash

# python3 -B main_opt_NLOPT.py --model_name 3D_model_w_whip_T1 --vid_off --start_time 0.0 --run_time 2.1 
# python3 -B main_opt_NLOPT.py --model_name 3D_model_w_whip_T2 --vid_off --start_time 0.0 --run_time 2.1 
# python3 -B main_opt_NLOPT.py --model_name 3D_model_w_whip_T3 --vid_off --start_time 0.0 --run_time 2.1 
# python3 -B main_opt_NLOPT.py --model_name 3D_model_w_whip_T4 --vid_off --start_time 0.0 --run_time 2.1 
# python3 -B main_opt_NLOPT.py --model_name 3D_model_w_whip_T5 --vid_off --start_time 0.0 --run_time 2.1 
# python3 -B main_opt_NLOPT.py --model_name 3D_model_w_whip_T6 --vid_off --start_time 0.0 --run_time 2.1 

# python3 main_whip.py --start_time 0.1 --run_time 2.5 --target_type 1 --save_data 
# python3 main_whip.py --start_time 0.1 --run_time 2.5 --target_type 2 --save_data
# python3 main_whip.py --start_time 0.1 --run_time 2.5 --target_type 3 --save_data
# python3 main_whip.py --start_time 0.1 --run_time 2.5 --target_type 4 --save_data
# python3 main_whip.py --start_time 0.1 --run_time 2.5 --target_type 5 --save_data
# python3 main_whip.py --start_time 0.1 --run_time 2.5 --target_type 6 --save_data

python3 -B main_whip_sensitivity.py --vid_off --target_idx 1 --start_time 0.0 --run_time 1.8
python3 -B main_whip_sensitivity.py --vid_off --target_idx 2 --start_time 0.0 --run_time 1.8
python3 -B main_whip_sensitivity.py --vid_off --target_idx 3 --start_time 0.0 --run_time 1.5
python3 -B main_whip_sensitivity.py --vid_off --target_idx 4 --start_time 0.0 --run_time 1.8
python3 -B main_whip_sensitivity.py --vid_off --target_idx 5 --start_time 0.0 --run_time 1.8

# python3 -B main_opt_NLOPT.py --start_time 0 --run_time 5.0 --model_name 2D_model_w_whip_N20 --vid_off --opt_type 1
# python3 -B main_opt_NLOPT.py --start_time 0 --run_time 5.0 --model_name 2D_model_w_whip_N20 --vid_off --opt_type 2


# python3 -B main_opt_NLOPT2.py --start_time 0 --model_name 3D_model_w_whip_T6 --vid_off --opt_type 1
# python3 -B main_opt_NLOPT2.py --start_time 0 --model_name 3D_model_w_whip_T6 --vid_off --opt_type 2
