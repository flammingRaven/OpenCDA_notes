# OpenCDA for Deep Reinforcement Learning



CARLA-SUMO Co-simulation



CARLA-SUMO Co-simulation in OpenCDA



OpenCDA&RL:

OpenCDA RL版本：https://github.com/ucla-mobility/OpenCDA/tree/feature/reinforcement_learning

```markdown
Yes, OpenCDA supports RL development. Pass the RL states to the behavior agent through function `update_information()`, execute the action in `run_step()` and collect the rewards in `plan_debuger()` or `EvaluationManager()`. If you are using deep RL, you can put your model in `MLManager` and add it as an attribute in your behavior agent class. Check https://opencda-documentation.readthedocs.io/en/latest/md_files/customization.html to see more details.Yes, OpenCDA supports RL development. Pass the RL states to the behavior agent through function `update_information()`, execute the action in `run_step()` and collect the rewards in `plan_debuger()` or `EvaluationManager()`. If you are using deep RL, you can put your model in `MLManager` and add it as an attribute in your behavior agent class. Check https://opencda-documentation.readthedocs.io/en/latest/md_files/customization.html to see more details.
```





