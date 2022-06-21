# **Whip Project - Targeting with a Whip**
---
We show that designing a controller with [**Dynamic Primitives**](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3735361/) can dramatically simplify the control of complex objects. We show this by controlling a whip - an infinite dimensional object with significant dynamics. 

<p align="center">
<img src="./Assets/Videos/video1.gif" alt="drawing" width="700"/>
</p>


# **Literatures**
---
pdf files of the papers are saved in [**Readings**](Assets/Readings/) folder.
1. [2020 BIOROB - Dynamic Primitives Facilitate Manipulating a Whip](https://ieeexplore.ieee.org/document/9224399)
2. [MIT Master's Thesis - Dynamic Primitives Facilitate Manipulating a Whip](https://dspace.mit.edu/handle/1721.1/127121)
3. [2021 ICRA - Manipulating a Whip in 3D via Dynamic Primitives](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9636257)

# **Presented Talks**
---
1. [2020 Neuromatch 3.0](https://www.youtube.com/watch?v=tE3y9LwvQKQ)
2. [2020 BIOROB](https://www.youtube.com/watch?v=PPzxmgUo0nY)
3. [2020 IROS Workshop - Impedance Learning](https://www.youtube.com/watch?v=OhSgroSByB4)

# ** For Beginners - How to setup a Virtual environment**
---
It is beneficial to setup the virtual environment before running the simulation.
Create `virtualenv` by typing the following line:
```bash
  python3 -m venv .venv
```
After the environment is generated, activate the virtual environment
```
  source .venv/bin/activate
```
Download all the required packages for the mujoco-py simulation.
```
  python3 -m pip install -r requirements.txt
```
And... it's ready to go!

# **References**
---
1. [MuJoCo Documentation](https://mujoco.readthedocs.io/en/latest/overview.html)
