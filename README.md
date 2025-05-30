# Lecture 4: State Machine, roslaunch2

## 1. Preparation

### 1.1 Downloading Exercise Materials and Package

Please access the links below to download the materials and exercise package.

- Download materials: [handout_ros2_lecture_04_en.pdf](https://drive.google.com/uc?export=download&id=1_A5LlQ9JvFae6hxGzyTKKDAMtNjRJAxL)
- Download exercise package: [lecture04_pkg.tar.gz](https://drive.google.com/uc?export=download&id=1cVOqf_yW94gildHvoCecZTcleShMjvki)

### 1.2 Creating the Package

- Terminal 1
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Create package

    ```Bash
    cd src/7_lectures/lecture04
    ```

    ```Bash
    ros2 pkg create lecture04_pkg --build-type ament_python
    ```

### 1.2 Moving README.md and Python Scripts

Follow the instructions below to move the downloaded exercise package.

- Extract the downloaded exercise package (`lecture04_pkg.tar.gz`).
- Move the extracted exercise package (`lecture04_pkg`) to `~/ros2_lecture_ws/src/7_lectures`.

### 1.3 Building the Package

Execute the following commands.

- Terminal 1
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Build the package

    ```Bash
    colcon build --symlink-install
    ```

After the build is complete, exit the virtual environment (`Ctrl+D`).

## 2. State Machine (YASMIN: Yet Another State MachINe)

### 2.1 YASMIN

- [arXiv paper](https://arxiv.org/pdf/2205.13284)
- [GitHub repository](https://github.com/uleroboticsgroup/yasmin.git)

### 2.2 Running State Machine Sample 1

#### 2.2.1 Execution Method

- Terminal 1
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Launch Yasmin Viewer node

    ```Bash
    ros2 run yasmin_viewer yasmin_viewer_node
    ```

    - Access [http://localhost:5000/](http://localhost:5000/)

- Terminal 2
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Run sm_sample1 node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture04_pkg sm_sample1
    ```

### 2.3 Exercise Task (sm_exercise)

Let's add a `HOGE` state to the sm_sample1 state machine.
Edit sm_exercise.py.

- Return value "outcome3" → Transition to state "FOO"
- Return value "outcome4" → Transition to state "BAR"

### 2.3.1 Execution Method

- Terminal 1 (<font color="Yellow">Skip if already done</font>)
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Launch Yasmin Viewer node

    ```Bash
    ros2 run yasmin_viewer yasmin_viewer_node
    ```

    - Access [http://localhost:5000/](http://localhost:5000/)

- Terminal 2
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Run sm_exercise node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture04_pkg sm_exercise
    ```

### 2.4 Running State Machine Sample 2

#### 2.4.1 Execution Method

- Terminal 1 (<font color="Yellow">Skip if already done</font>)
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Launch Yasmin Viewer node

    ```Bash
    ros2 run yasmin_viewer yasmin_viewer_node
    ```

    - Access [http://localhost:5000/](http://localhost:5000/) (Set Layout to `grid` and display in full screen)

- Terminal 2
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Run sm_sample2 node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture04_pkg sm_sample2
    ```

## 3. Launch

This section requires lecture01_pkg and the TurtleBot3 hardware.

### 3.1 Running Launch File Sample

Let's run the talker and listener nodes from the first lecture using a launch file.

#### 3.1.1 Execution Method

- Terminal 1
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Launch launch_sample1.launch.py

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 launch lecture04_pkg launch_sample1.launch.py
    ```

## 3.2 Running Launch File Sample with TurtleBot3

<font color="Yellow">From here, we will use TurtleBot3. Please prepare it.</font> \
Show something red to the camera mounted on the TURTLEBOT3 and try to control the robot.

#### 3.2.1 Execution Method

- Terminal 1
  - Time synchronization

    ```Bash
    turtlebot3_mode
    ```

- Terminal 2
  - Remote access to TurtleBot3

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    Password: turtlebot
    Note: The password will not be displayed when typing.

  - Launch the system

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- Terminal 3
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Set ROS_DOMAIN_ID and TurtleBot3 environment variables

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - Launch competition.launch.py

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    . 4a_turtlebot3_settings.sh​
    ```

    ```Bash
    ros2 launch lecture04_pkg launch_sample2.launch.py
    ```

    Access http://localhost:5000/

- Terminal 4
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Set ROS_DOMAIN_ID and TurtleBot3 environment variables

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - Launch sm_main node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    . 4a_turtlebot3_settings.sh​
    ```

    ```Bash
    ros2 run lecture04_pkg sm_main
    ```

    Click Enter to start the state machine.
