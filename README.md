# English Speech Commands Recognition



(See the [README.md](../README.md) file in the upper level 'examples' directory for more information about examples.)

## How to use this example


### Additional Hardware Required

\- A Speaker

### Configure, Build and Flash


##### set-target 

```
idf.py set-target esp32s3
```

##### configure

Select the default sdkconfig according to the development board module

For example:  

```
cp sdkconfig.defaults.esp32s3 sdkconfig
```

##### build&flash

Build the project and flash it to the board, then run the monitor tool to view the output via serial port:

```
idf.py -b 2000000 flash monitor 
```

(To exit the serial monitor, type ``Ctrl-]``.)

### Modify speech commands

We recommend using MultiNet6 or newer models.   
Here's a simple example to modify speech commands in the code.  
You can also modify the default command list, please refer to [document](https://docs.espressif.com/projects/esp-sr/en/latest/esp32s3/speech_command_recognition/README.html) for more details.

```
// MultiNet6
    // Note: Please create multinet handle before adding speech commands

    esp_mn_commands_clear();                       // Clear commands that already exist 
    esp_mn_commands_add(1, "turn on the light");   // add a command
    esp_mn_commands_add(2, "turn off the light");  // add a command
    esp_mn_commands_update();                      // update commands
    multinet->print_active_speech_commands(model_data);     // print active commands
```
# esp-Ai-digicam

Ai digicam

Firstly, lcd previews camera frame.

2 options to capture an image:

1. Pressing the boot button
2. Activating with voice command [AI]

For method 2:
1. Activate AI using wake word “Jarvis”
2. If wake word successfully recognised, the text “Yessir!” Will appear on the LCD screen
3. The following commands will capture an image:
    1. Take Picture
    2. Take a picture
    3. Skibidi toilet

Once image capture task has been activated, camera frame will be taken, and sent to a telegram bot, which would then send an image to telegram.	

