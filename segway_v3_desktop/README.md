# segway_v3_desktop

Desktop packages for remote visualization and control of Segway RMP V3 platforms

**This document runs through the basic setup required to get a remote desktop running to interface with a Segway RMP V3 robot**

**If you have purchased a Navigator or Navigator Elite package, you received a VM with this setup, and all sorts of extras, so no need to worry about going through this installation guide**

**For all you brave DIYers that decided they didn't need Stanley Innovation's expertise, and went with the barebones V3 system......ENJOY!!!**

**If you haven't decided between barebones and a Navigator Package; go through this and the segway_v3_robot setup before you decide......How valuable is your time??? I bet the development your working on is much more valuable than spending a bunch of time mucking around with setup......GO WITH A NAVIGATOR PACKAGE!!! That's the pitch, we'll leave the decision up to you**

# We provide fully integrated systems with support
**We provide a Navigator package, a Navigator Elite package and fully integrated custom solutions with all robot setup, networking, timing, sensor/peripheral integration, calibration, tailored navigation tuning, extended functionality, and remote desktop VM. Our integrated packages come with fully setup onboard PC (for robot control) and a VM for remote monitoring and control. This tutorial is for seasoned ROS integrators that can complete that work themselves with our base RMP V3 platforms. Please contact Stanley Innovation for pricing and information on fully integrated packages and base platforms http://stanleyinnovation.com/contact-us/. Stanley Innovation is the _ONLY_ supplier of Segway RMP V3 compatible hardware! Please do not expect any of this to work if you did not purchase the system or an upgrade from Stanley Innovation, Inc.**

**If you have a Segway RMP from another source it can be upgraded. Do not try and load the V3 software without an upgrade performed by Stanley Innovation!!!!! It will render the machine inoperable, even if you try and go back to the standard RMP Release. If you do this you will likely have no option other than buying the upgrade to get the machine running again. You've been warned here and all over the place....don't do it.....**

**_Make_ _your_ _intern_ _do_ _it_, and then blame him when you tell your boss you have to upgrade your system ;)**

**If you want one-on-one engineering support for a system with onboard PC, remote desktop VM, sensors, nav, etc...please buy a Navigator package from us or contact us for an engineering support quote. Otherwise if you plan to buy your own sensors and your own computer for integration it is assumed you know what you're doing. Please use the community for support in integrating your own hardware, we will only address RMP specific questions for these customers if contacted directly. For example if you buy a barebones RMP V3 mobility platform and you need to know how to pull a faultlog, feel free to ask us; if you are trying to setup your PC and all your sensors, please rely on the community. We may help as part of the community for non-platform related questions, but there is no gaurantee**

## Installation
Until we have released our packages in the ROS distro please follow these instructions for installing from source. The following instructions are valid for Ubuntu 14.04LTS and ROS Indigo. Before proceding please install Ubuntu 14.04LTS.

### Requirements
* **A Segway RMP V3 Robot that has been setup either by Stanley Innovation or by you**
  * See https://github.com/StanleyInnovation/segway_v3_robot
* **PC or VM Running Ubuntu 14.04 LTS**
  * Minimum 8GB RAM (16 GB preferably)
  * As many cores as you can get
  * Preferably a reasonably fast SSD
  * Atleast 1 USB 2.0 port 
    * For the remote joystick
  * Monitor, keyboard and mouse
  
### Install ROS Indigo
From a linux machine connected to the internet run the following commands

1. **Setup your ROS sources.list**
  * Setup your computer to accept software from packages.ros.org. ROS Indigo ONLY supports Saucy (13.10) and Trusty (14.04) for debian packages.
  ```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  ```
2. **Set up your ROS keys**
  * Use the following command
  ```
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
  ```
3. **ROS Installation**
  * First, make sure your Debian package index is up-to-date:
  ```
  sudo apt-get update
  sudo apt-get install ros-indigo-desktop-full
  ``` 
4. **Initialize rosdep**
  * You must initialize rosdep
  ```
  sudo rosdep init
  rosdep update
  ``` 
5. **Environment setup**
  * Edit the local bash environment to add a few useful aliases
  ```
  gedit ~/.bashrc
  ``` 
  * Add the following lines to the end of the file each provides a few shortcuts:
  ```
  function save_map()
  {
    if [ ! -z "$1" ]
    then
      local dest='/home/sibot/segway_ws/src/segway_v3/segway_navigation/segway_navigation_apps/maps/'$1
    else
      local dest='/home/sibot/segway_ws/src/segway_v3/segway_navigation/segway_navigation_apps/maps/mymap'
    fi

    local cmd_ssh="
    source '/home/sibot/.bashrc';
    rosrun map_server map_saver -f $dest"

    ssh sibot@SIBOT1 "$cmd_ssh"
  }
  source /opt/ros/indigo/setup.bash
  alias sws='source ./devel/setup.bash'
  alias clean_backups='find ./ -name '*~' | xargs rm'
  alias clean_pyc='find ./ -name '*.pyc' | xargs rm'
  alias clean_rosbuild='rm -rf build devel install'
  
  #change SIBOT1 to whatever you onboard PC name is
  export ROBOT_PC_NAME=SIBOT1
  export ROS_MASTER_URI=http://$ROBOT_PC_NAME:11311
  
  # This is the IP of the remote desktop it set eth0 to whatever NIC your using to connect to the platform
  # It grabs whatever IP is associated with that NIC
  export ROS_IP=$(ip -4 address show eth0 | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')
  
  #change SIBOT1 to whatever you onboard PC name is
  alias s1='ssh -X sibot@SIBOT1'
  ```
6. **Getting rosinstall**
  * rosinstall is a frequently used command-line tool in ROS that is distributed separately. It enables you to easily download many source trees for ROS packages with one command.
  ```
  sudo apt-get install python-rosinstall
  ```

### Install required packages
From a linux machine connected to the internet run the following commands

1. **Install useful linux utilities**
  * These tools are useful for monitoring system processes, setting up networking, and setting up NTPD for the remote computer. They are not neccessary but recommended.
  ```
  sudo apt-get install iperf chrony htop bridge-utils
  ```
2. **Install required ROS third party packages for segway_v3_robot**
  * These are the packages that RMP V3 depends on might as well have them for the remote machine
  ```
  sudo apt-get install ros-indigo-navigation ros-indigo-gmapping ros-indigo-robot-localization ros-indigo-yocs-cmd-vel-mux ros-indigo-joy ros-indigo-urg-node ros-indigo-lms1xx ros-indigo-pointgrey-camera-driver ros-indigo-cmake-modules daemontools openssh-server libpcap0.8-dev ros-indigo-imu-tools
  ```
4. **Create a workspace in your home directory and get sources**
  * This is what you need if you want to visualize
  ```
  mkdir -p ~/segway_remote_ws/src
  cd ~/segway_remote_ws/src
  catkin_init_workspace
  cd ..
  catkin_make
  cd ~/segway_remote_ws/src
  git clone https://github.com/StanleyInnovation/segway_v3.git
  git clone https://github.com/StanleyInnovation/segway_v3_desktop.git
  cd ..
  catkin_make
  ```
  
### Setup Network
You need to set the network up for the connection to the robot PC.
This is an outline but **we also provide fully integrated packages**.

**Please make sure you have completed the robot setup (https://github.com/StanleyInnovation/segway_v3_robot) and that the system is powered on with the networking all setup**

1. Connect to the wireless network.
2. You can either use a static IP or just use the DHCP that was setup on the wireless router
  * I usually just use DHCP
3. Make sure you can ping the wireless router and the robot PC
  * First the wireless router 
  ```
  ping 10.66.172.1
  ```
  * And now the Robot PC
  ```
  ping 10.66.172.4
  ```
  
The networking should be all setup now and you can take a quick breather.

### Setup an SHH connection
You need to be able to SSH into the onboard PC here are some basic instructions

Things to configure:
* **Robot PC user name: sibot**
* **Robot PC name: SIBOT1**
* **Robot PC IP address: 10.66.172.4**
* **Remote Desktop user name: si**
* **Remote Desktop PC name: SIDEV1**
* **password for both users: pswd**
* **ssh port (standard): 22**

**These are used in the examples below, substitute as you like**

#### Setup Robot PC Name Resolution
* We need to add the robot pc to our known hosts list so we can resolve it by name
```
sudo gedit /etc/hosts
```
* Add the following line
```
10.66.172.2	SIBOT1
```
* Save and close
  
#### Make sure you can ping the robot
From the terminal
```
ping sibot1
```
#### Setup your SSH Keys
Open a terminal and create the RSA key pair:

```
ssh-keygen -t rsa
```

Once you have entered the Gen Key command, you will get a few more questions:

```
Enter file in which to save the key (/home/si/.ssh/id_rsa):
```

You can press enter here, saving the file to the user home (in this case, my example user is called si).

```
Enter passphrase (empty for no passphrase):
```

It's up to you whether you want to use a passphrase. I don't the robot network generally isn't connected to public networks. Entering a passphrase does have its benefits: the security of a key, no matter how encrypted, still depends on the fact that it is not visible to anyone else. Should a passphrase-protected private key fall into an unauthorized users possession, they will be unable to log in to its associated accounts until they figure out the passphrase, buying the hacked user some extra time. The only downside, of course, to having a passphrase, is then having to type it in each time you use the Key Pair.

The entire key generation process looks like this:

```
ssh-keygen -t rsa
Generating public/private rsa key pair.
Enter file in which to save the key (/home/si/.ssh/id_rsa): 
Enter passphrase (empty for no passphrase): 
Enter same passphrase again: 
Your identification has been saved in /home/si/.ssh/id_rsa.
Your public key has been saved in /home/si/.ssh/id_rsa.pub.
The key fingerprint is:
4a:dd:0a:c6:35:4e:3f:ed:27:38:8c:74:44:4d:93:67 si@sidev1
The key's randomart image is:
+--[ RSA 2048]----+
|          .oo.   |
|         .  o.E  |
|        + .  o   |
|     . = = .     |
|      = S = .    |
|     o + = +     |
|      . o + o .  |
|           . o   |
|                 |
+-----------------+
```
#### Copy the public key
Once the key pair is generated, it's time to place the public key on the robot PC.

You can copy the public key into the new machine's authorized_keys file with the ssh-copy-id command. From the terminal:

```
ssh-copy-id sibot@SIBOT1
```

You should see something like this:

```
The authenticity of host '12.34.56.78 (12.34.56.78)' can't be established.
RSA key fingerprint is b1:2d:33:67:ce:35:4d:5f:f3:a8:cd:c0:c4:48:86:12.
Are you sure you want to continue connecting (yes/no)? yes
Warning: Permanently added '10.66.172.4' (RSA) to the list of known hosts.
sibot@SIBOT1's password:
```
Now enter the password you chose for the robot PC and you should see:

``` 
Now try logging into the machine, with "ssh 'sibot@SIBOT1'", and check in:

  ~/.ssh/authorized_keys

to make sure we haven't added extra keys that you weren't expecting.
```

You should now be able to ssh into the robot PC. Test it to make sure. From the terminal:

```
s1
```

# Assuming you followed the instructions up to this point you should have successfully established an SSH connection, a little more setup and your on your way
# Don't you wish you sprung for that fully integrated system???

### Setup Chrony
**Setup chrony**

* If you are going to be running ROS nodes on a remote computer it is a good idea to setup chrony to synchronize time between the machines
* The onboard robot PC should ideally run the server
* There is information on how to do this out there we will not cover it here
* **Fully integrated machines delivered by Stanley come with this all setup**

### Check your setup
Now its time to see if all the hardwork you did actually works. Assuming you followed the instructions to setup the robot
and the instructions to setup the remote desktop you should cruise right through this.

**Open a terminal**

1. **Ping Check**
  * Its always best to just run a quick ping to make sure the robot pc is accessible
  ```
  ping sibot1
  ```
2. **Restart the robot upstart service**
  * **You don't need to do this but it doesn't hurt**
  * SSH in
  ```
  s1
  ```
  * Stop the service
  ```
  segstop
  ```
  * Start the service
  ```
  segstart
  ```
  * Make sure everything starts fine you can repeatedly enter the following until the launch is finished
  ```
  segchk
  ```
  * You should hear 2 beeps when the configuration server is initialized and 2 more when the platform is ready to accept commands
  * The launch is time staged so it takes ~15 seconds to complete
  * Close the SSH connection
  ```
  exit
  ```
3. **Make sure you can visualize everything**
   * Close all the terminals and open a new one
   * Start robot visualization
   ```
   cd ~/segway_remote_ws
   sws
   roslaunch segway_viz view_robot.launch
   ```
   * You should see your robot model come up and be able to see all the different sensors
4. **Check out the reconfiguration GUI**
   * Open a new tab <Ctrl>+<Shft>+<t>
   ```
   sws
   rosrun rqt_reconfigure rqt_reconfigure
   ```
   * Expand Segway and mouse over the options to get a description
     * **Make sure you read the manual and understand what parameters do before changing them**
5. **See all the data**
  * Get familiar with RQT http://wiki.ros.org/rqt
  * Open a new tab <Ctrl>+<Shft>+<t>
  ```
  sws
  rqt
  ```
  * If you are not familiar with RQT just click <Plugins> in the menu and select a few useful ones
    * Robots->Segway_RMP_V3
    * Visualization->RVIZ
    * Topics->Topic Monitor
    * Configuration->Dynamic Reconfigure
    
  * Experiment, play around with it etc.

5. **Drive the system around**
  * If the platform is equipped with a wireless joystick you should be able to just drive it around
  * If you want to control it from the remote desktop
    * Get a compatible controller
      * XBOX360 style that have been tested: Logitech F310, Logitech F710 (only in XINPUT mode)
      * Joystick type that have been tested: Logitech Extreme 3D
    * Make sure that the 50.segway_config.sh file has the right controller configuration
      * Default is xbox360
      * change **SEGWAY_JOY_MAPPING** to extreme3D if you want to use the Extreme3D
      * Create your own configuration (see example yaml files on the robot PC)
        * ~/segway_ws/src/segway_robot/segway_bringup/launch/teleop/config
    * Connect the controller and make sure determine which device it is
      * For VM it is generally /dev/input/js2
      * For native machines it is generally /dev/input/js0
    * open a new tab <Ctrl>+<Shft>+<t>
    ```
    sws
    roslaunch segway_remote_teleop segway_remote_teleop.launch input:=/dev/input/js0
    ```
    
  * You should be able to drive the platform around
  * See the controller mapping in the manuals or in the configuration files
  * Basic xbox360 style operation
    * B button is Tractor Mode
    * X button is Standby Mode
    * A button is Balance Mode (only for the 220)
    * Left Trigger is deadman (must squeeze to command motion)
    * Right Trigger is manual override for assisted teleop
    * Left DPAD makes a catcall
    * Left joystick forward/back +/-x linear velocity
    * Left joystick left/right +/-y linear velocity (obviously only for OMNI)
    * Right joystick left/right +/-z angular
    * LB Decel to disable emergency response
    * RB Decel to zero speed emergency response
    * Start button Powerdown request
    
#You are now ready to play with the navigation demos, congratulations!
## One warning for the navigation demos. We assume that you understand that since we were not responsible for integrating your system we cannot gaurantee anything will work. If you purchased an integrated package from SI, you're inluck we already tested your system and its good to go. You must have been reading this for fun.
