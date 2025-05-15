- cartesian coordinates
  - schema
    - (mm, mm, mm, deg, deg, deg)
  - home position
    - (0, -150.8, 615.2, -90, 0, 180)
  - target position
    - (-350, -50, 100, 180, 0, 0)

current tasks
- get system id control policy to work
- collect tactile sensor camera readout
- collect force readout

MovL /w speed parameter doesn't work for rotations around a single position
- I could potentially call ServoP in a loop together /w sleep commands - but that's a lot of work

problem: the tactile sensor camera gets too much light and some markers aren't visible

maybe first collect the press-slide dataset and worry about press-twist-x and press-twist-z later

my system-id.py script terminated /w an error but the robot is still executing the desired command in a loop
- error: Error while closing socket: an integer is required (got type NoneType)
  - I can safely ignore this error - this is something that is Dobot's fault but it doesn't affect my task
- the script writes to a file right before termination - the write to file happens before the robot is done running
- most likely the robot command is async, i.e. I produce a job and don't wait until it's done

problem: the suturing phantom's bottom layer is made of foam - I cannot fix it to the optical board using the thick double-sided tape
if I just leave it on the optical board w/o any fixing, the phantom slides during press-slide motion

let's copy-paste the code from dobot demo

I want to collect as many logs and data as possible and I will analyse it / debug at home

to do later:
- control the robot's angular speed (maybe try from Dobot Studio)

timestamp from dobot feedback doesn't work - it shows 1970 and it shows that the whole motion took place within less than 1 second even though the whole motion lasted 1 minute
- so I need to get the timestamp from ubuntu system clock insted

also, the force value readings are all 0 - maybe I need to enable force readings in settings

I want to get dobot feedback and video recording simultaneously and then when I'm back home stich these 2 datasets together

- obtain pose and video readings simultaneously

I enabled the FT sensor via EnableFTSensor (I get status code 0) but when I run GetForce I get status code -1
Command execution failed
Out[3]: '-1,{},GetForce();'

maybe try using the GUI

it's quite likely that the force-torque readings require connecting a force-torque sensor
so if it doesn't work after a while, just assume it won't work and move on - the force-torque sensor isn't crucial - I can rely only on tactile sensor marker deformations

GUI tells me "FT sensor not connected" - so I won't be getting force-torque readings for now
- I will talk to Dandan later about whether it makes sense to use the force-torque sensor or not

one last try - no, let's not waste time - I know that FT sensing doesn't work already

problem: regardless of the room lighting conditions (bright - light from outside or lamps inside turned on; dark - dark outside, lamps indide turned off), the tactile video data has too much light and the markers aren't visible

I need separate gucview settings for different lighting conditions (mainly modify the gamma value)

