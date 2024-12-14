# TowFish AUV

This github repository works, so it should work on you PC as well!!!

## Setting Up ROS Master-Slave Configuration

To configure your laptop as a ROS master or slave, follow these steps to add the required lines to the .bashrc file:

### 1. Add ROS Configuration Lines to .bashrc:
- Open a terminal and run the following commands to append the environment variables directly to the .bashrc file:

```bash
echo "export ROS_IP=192.168.178.154" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.178.154:11311" >> ~/.bashrc
```

- Source the updated .bashrc file to immediately apply the changes:

```bash
source ~/.bashrc
```

### 2. Allowing traffic on port 11311:
- We will need to explicitly allow traffic on port 11311, which tis the default port used by ROS Master. To do this run the following command on terminal:

```bash
sudo ufw allow 11311
```
- This is necessary for communication between the ROS Master and Slave nodes.

### 2. Verify the Configuration:
- Check if the variables have been set correctly by running:

```bash
echo $ROS_IP
echo $ROS_MASTER_URI
```

- The output should display:
```bash
192.168.178.154
http://192.168.178.154:11311
```

### 3. Run roscore to Start the ROS Master

- Start the ROS master node by running the following command in a terminal:
```bash
roscore
```
- IMPORTANT: The terminal needs to remain open to Make sure that the roscore is running for the entire time, 

## Running a Script on the Raspberry Pi via SSH

### 1. Physical Connection:
- Connect the Raspberry Pi to your laptop using a standard Ethernet cable.

### 2. Access Network Settings on Your Laptop:
- Navigate to the Wired Network Settings on your laptop.
- Locate and select the network associated with the Ethernet connection.

### 3. Configure IPv4 Settings:
- Open the settings for the selected network.
- Go to the IPv4 tab or section.
- Make the following changes as shown in the Image in the manual method
- Save the changes and apply the new settings.

![image](https://github.com/user-attachments/assets/4aa04a6e-1bcc-4dfd-bf40-ce31e1cdc48a)

### 4. Establish connection using SSH:
- Paste the following command into the terminal and press Enter:

```bash
sudo ssh -X towfish@192.168.178.123
```

- When prompted for the password, type the following password and press Enter:

```bash
qwerty
```
- Once authenticated, the SSH connection will be established, and you will have remote access to the Raspberry Pi's terminal.

### 5. Run the script:

- Once SSH is done, navigate to the catkin_ws directory by running the following command:
```bash
/home/towfish/Desktop/catkin_ws
```

- Source the setup.bash file and execute the Python script using python3 by pasting the following command in the home directory:
```bash
source devel/setup.bash
python3 src/run.py
```













