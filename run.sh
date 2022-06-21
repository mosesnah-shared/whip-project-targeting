#!/bin/zsh

python3 -B run.py --modelName="1_3D_model_w_N25_T1.xml" --startTime=0.0 --runTime=2.4 --runOptimization
python3 -B run.py --modelName="1_3D_model_w_N25_T2.xml" --startTime=0.0 --runTime=2.4 --runOptimization
python3 -B run.py --modelName="1_3D_model_w_N25_T3.xml" --startTime=0.0 --runTime=2.4 --runOptimization
