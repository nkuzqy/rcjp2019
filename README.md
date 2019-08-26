# rcjp2019
## OPL inspection & basic functionalities
### for inspection 
```
roslaunch inspection inspection.launch
```
we use astra for the door open detection, caution the height of door and your astra position

for basic functionalties:
```
roslaunch kamerider_control_core basic_func.launch 
```
for navigation: amcl & gmapping

problem happened when close one door, new path plan required. NEED TO CHECK

for details, see the rulebook of basic_func
https://github.com/RoboCupAtHomeJP/Rule2019

for human detection:
baidu API

problem: more than 1 person detected?
         when facing the wall -> 1 person detected

NEED TO CHECK

for speech recognition:
PocketSphinx need to be installed
probably some path error
CHECK

for different language model: update the .corpus, generate new language model online:
http://www.speech.cs.cmu.edu/tools/lmtool-new.html

remember to change the .lm in the sound_test.launch file








