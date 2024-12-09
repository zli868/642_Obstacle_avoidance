## project structure

```
/ros2_ws/
├── src/
│   ├── build/ 
│   ├── install/
│   ├── log/
│   ├── my_turtle_bot/
│   │   ├── my_turtle_bot/
│   │   │   ├── __init__.py
│   │   │   ├── avoidance.py
│   │   │   └── my_avoidance.py
│   │   ├── resource/
│   │   ├── test/
│   │   ├── setup.cfg
│   │   ├── setup.py
│   │   └── package.xml
```
- explanation:
only two files should be explained here. \
avoidance.py is for running simulation on obstacle avoidance using Evolutionary algorithm. \
my_avoidance.py is actually the testing script, which is used after running avoidance.py and the initial motor speed and final motor speed are showned on the console, to recreate and validate the results.
## Run the simulation
- Environment set up. first set up the gazebo environment according to the homework3 instruction, so that the ros2 services can be adopted. This can be setup directly in a linux system or using Docker. After installing gazebo, go to your turtlebot3_ws and run `colcon build &&\
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` to generate world environment. Turtlebot3_world is the default environment for this project.
- Optional (but highly recommended). Modify the RTF (real time factor) in simulation world for a faster simulation. For turtlebot3_world, the file should be in /turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_worlds. Look for burger.model(burger by default). The file should be look like this
```bash
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <scene>
      <shadows>false</shadows>
    </scene>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>0.319654 -0.235002 9.29441 0 1.5138 0.009599</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <physics type="ode">
      <real_time_update_rate>10000.0</real_time_update_rate>
      <max_step_size>0.005</max_step_size>
      <real_time_factor>50</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <model name="turtlebot3_world">
      <static>1</static>
      <include>
        <uri>model://turtlebot3_world</uri>
      </include>
    </model>

    <include>
      <pose>-2.0 -0.5 0.01 0.0 0.0 0.0</pose>
      <uri>model://turtlebot3_burger</uri>
    </include>

  </world>
</sdf>
```
Only three parameters are related here and they should have relationship as follow. real_time_update_rate * max_step_size = real_time_factor. real_time_factor determines how fast the simulation world is to the real world. Be careful, you don't want a too fast simulation as that can lower the accuracy score. After Modifying the file, Remember to rebuild the package using command given in the first step.

- Open another terminal. Then, paste the ros2_ws.zip in working directory and unzip it. Go to ros2_ws/src, run `colcon build &&\
source ../src/install/setup.bash &&\
ros2 run my_turtle_bot avoidance` to start simulation.
- You can also modify the specs in avoidance.py for variables like generation, population_size, etc. Remember to run, go to ros2_ws/src, run `colcon build &&\
source ~/ros2_ws/src/install/setup.bash` before running the simulation if you want the modification to take effect.
- Wait for the itertation ends. If you speed up the simulation, this should not take too long for an appropriate generation and population size. it prints out the best motor speed used in initial population and best motor speed for every five generations till the end.
- Grab the best inital and final motor speed pairs showned on console. First try the initial motor speed pairs and then try the final ones in my_avoidance.py. Using following command, go to /ros2_ws/src folder and run `colcon build &&\
source ../src/install/setup.bash &&\
ros2 run my_turtle_bot my_avoidance
` . Validate the results.

