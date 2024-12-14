# TowFish_AUV

This github repository works, so it should work on you PC as well!!!

## Setting up ssh with Raspberry Pi 4B

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





