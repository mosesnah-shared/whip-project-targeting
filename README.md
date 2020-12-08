**Whip Project - Targeting with a Whip**
---
Human dexterity far exceeds that of modern robots, despite a much slower neuromuscular system. Understanding how this is accomplished may lead to improved robot control. The slow neuromuscular system of humans implies that prediction based on
some form of internal model plays a prominent role. However, the nature of the model itself remains unclear. To address this problem, we focused on one of the most complex and exotic tools humans can manipulate—a whip. We tested (in simulation) whether a distant target could be reached with a whip using a (small) number of dynamic primitives, whose parameters could be learned through optimization. This approach was able to manage the complexity of an (extremely) high degree-of-freedom system and discovered the optimal parameters of the upper-limb movement that achieved the task. A detailed model of the whip dynamics was not needed for this approach, which thereby significantly relieved the computational burden of task representation and performance optimization. These results support our hypothesis that composing control using dynamic motor primitives may be a strategy which humans use to enable their remarkable dexterity. A similar approach may contribute to improved robot control.

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

**How to use mujoco-py simulation**
---

```
Usage:
  python3 run.py --modelName="2D_model.xml" --simType=1 --runTime=30
  python3 run.py --modelName="2D_model.xml" --simType=2
  python3 run.py --modelName="3D_model.xml" --simType=1 --saveData
  python3 run.py --modelName="3D_model.xml" --simType=2 --saveData --recordVideo --vidRate=0.5
  python3 run.py --modelName="3D_model.xml" --simType=2 --saveData --videoOFF

Options:
  --ZFTType      1: Joint space trajectory tracking task   2: Cartesian space trajectory tracking task
  --saveData     Saving the essential simulations data as .txt file
  --videoOFF     Turning off the video of the mujoco-py simulation
  --recordVideo  Recording the simulation
  --vidRate      Setting the speed of the video that you are saving, e.g., --vidRate=0.5 means the video is half the speed of original

```

**References**
---
1. [MuJoCo install link](https://www.roboti.us/index.html)
2. [MuJoCo xml model file documentation](http://mujoco.org/book/XMLreference.html)
3. [MuJoCo python controller file documentation](https://openai.github.io/mujoco-py/build/html/index.html)
4. [MuJoCo c++    controller file documentation](http://mujoco.org/book/APIreference.html)
5. [THE ERIC P. AND EVELYN E. NEWMAN LABORATORY FOR BIOMECHANICS AND HUMAN REHABILITATION](https://newmanlab.mit.edu/)
