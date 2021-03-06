**Whip Project - Targeting with a Whip**
---
Human dexterity far exceeds that of modern robots, despite a much slower neuromuscular system. Understanding how this is accomplished may lead to improved robot control. The slow neuromuscular system of humans implies that prediction based on
some form of internal model plays a prominent role. However, the nature of the model itself remains unclear. To address this problem, we focused on one of the most complex and exotic tools humans can manipulate—a whip. We tested (in simulation) whether a distant target could be reached with a whip using a (small) number of dynamic primitives, whose parameters could be learned through optimization. This approach was able to manage the complexity of an (extremely) high degree-of-freedom system and discovered the optimal parameters of the upper-limb movement that achieved the task. A detailed model of the whip dynamics was not needed for this approach, which thereby significantly relieved the computational burden of task representation and performance optimization. These results support our hypothesis that composing control using dynamic motor primitives may be a strategy which humans use to enable their remarkable dexterity. A similar approach may contribute to improved robot control.

**Citing our work**
---
We kindly ask to cite our paper if you find this library useful:
```
@inproceedings{nah2020dynamic,
  title={Dynamic primitives facilitate manipulating a whip},
  author={Nah, Moses C and Krotov, Aleksei and Russo, Marta and Sternad, Dagmar and Hogan, Neville},
  booktitle={2020 8th IEEE RAS/EMBS International Conference for Biomedical Robotics and Biomechatronics (BioRob)},
  pages={685--691},
  year={2020},
  url={https://github.com/mosesnah-shared/whip-project-targeting},
  organization={IEEE}
}

@mastersthesis{moses2020master,
  title={Dynamic Primitives Facilitate Manipulating a Whip},
  author={Nah, Moses C.},
  year={2020},
  url={https://github.com/mosesnah-shared/whip-project-targeting},  
  school={Massachusetts Institute of Technology}
}

```

**How to setup virtual environment**
---
It is beneficial to setup the virtual environment before running the simulation.
Create `virtualenv` by typing the following line:
```
  python3 -m venv .
```
After the environment is generated, activate the virtual environment
```
  cd MuJoCo && source ./bin/activate
```
Download all the required packages for the mujoco-py simulation.
```
  python3 -m pip install -r requirements.txt
```
And... it's ready to go!


**How to use mujoco-py simulation**
---

```
Usage:
  python3 run.py --help
  python3 run.py --version
  python3 run.py --modelName="1_2D_model_w_N10.xml" --runTime=6
  python3 run.py --modelName="1_2D_model_w_N15.xml" --startTime=1   --runTime=6
  python3 run.py --modelName="1_2D_model_w_N15.xml" --startTime=0.1 --runTime=6 --saveData  
  python3 run.py --modelName="1_2D_model_w_N20.xml" --startTime=0.1 --runTime=6 --videoOFF
  python3 run.py --modelName="1_2D_model_w_N20.xml" --startTime=0.1 --runTime=6 --videoOFF --saveData
  python3 run.py --modelName="1_2D_model_w_N10.xml" --startTime=0.1 --runTime=3 --runOptimization
  python3 run.py --modelName="1_3D_model_w_N25_T1.xml" --startTime=0.1 --runTime=3 --runOptimization
  python3 run.py --modelName="1_3D_model_w_N25_T2.xml" --startTime=0.1 --runTime=3 --recordVideo --vidRate=0.5

Options:
  --saveData     Saving the essential simulations data as .txt file
  --videoOFF     Turning off the video of the mujoco-py simulation
  --recordVideo  Recording the simulation
  --modelName    Setting the xml model file name which will be used for the simulation.
  --runTime      The total time of the simulation
  --startTime    The start time of the movement, or controller
  --vidRate      Speed of the video to be saved, if vidRate=0.5, then video is 2 times slower
```


**Literatures**
---
pdf files of the papers are saved in [**Readings**](Readings/) folder.
1. [2020 BIOROB - Dynamic Primitives Facilitate Manipulating a Whip](https://ieeexplore.ieee.org/document/9224399)
2. [MIT Master's Thesis - Dynamic Primitives Facilitate Manipulating a Whip](https://dspace.mit.edu/handle/1721.1/127121)

**Presented Talks**
---
1. [2020 Neuromatch 3.0](https://www.youtube.com/watch?v=tE3y9LwvQKQ)
2. [2020 BIOROB](https://www.youtube.com/watch?v=PPzxmgUo0nY)
3. [2020 IROS Workshop - Impedance Learning](https://www.youtube.com/watch?v=OhSgroSByB4)

**Code Description**
---
The [**MuJoCo**](MuJoCo/) simulator consists of the xml model file and the python controller file. That’s simply it. If you have a xml model file and a corresponding controller file, it is ready to run a single-complete simulation. The xml model files are all saved in [**model**](MuJoCo/models) folder, and the python controller objects are all saved in [**controllers.py**](MuJoCo/modules/) python script.

**References**
---
1. [MuJoCo install link](https://www.roboti.us/index.html)
2. [MuJoCo xml model file documentation](http://mujoco.org/book/XMLreference.html)
3. [MuJoCo python controller file documentation](https://openai.github.io/mujoco-py/build/html/index.html)
4. [MuJoCo c++    controller file documentation](http://mujoco.org/book/APIreference.html)
5. [THE ERIC P. AND EVELYN E. NEWMAN LABORATORY FOR BIOMECHANICS AND HUMAN REHABILITATION](https://newmanlab.mit.edu/)
