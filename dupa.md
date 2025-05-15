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
