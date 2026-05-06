# Final Report
# Carter-and-Anthony
# {Car Chase}
# 05/14/2026

Team Members:
- {Carter Gill}, {Cartergi@buffalo.edu}
- {Anthony Fazio}, {afazio@buffalo.edu}

### Motivation / Overview of your project.
- Are motivation for this project was a few things. The first is we wanted to continue on with the idea of detection from class. We wanted to build a reliable algorithim to dected a round object as well as a color. This is importenat becuase it builds on computer learning and helps to build on AI. On top of this we wanted to make our car dected and follow a purple ball and bring it back. This idea is so that if someone is practiceing golf they can putt and a robot will go and get the ball so you dont have to leave your spot.

- We also wanted to help out and make the racer car better for future classes, so we designed some new attachements for the car. The first being a pusher for the car to push a ball around. Also we designed a new cover for the car with aruco tag holders on all sides so that cameras can be used to know where the car is around a room.

- In this project we had a few main objectives. The first was we needed to design a 3d attachment to the car that it would allow the car to push a ball around (see figure 1) this was the second version we came up with and designed. The designed was intended at low speeeds to keep the ball trapped inside with the design of the traingular peiece in the middle and the curled sides keeping the ball moving in a circular formation when bumped around. On top of this we also designed a new houseing for the racer cars that alows aruco tages to be placed on all sides so cameras can be used to track the car (see figure 2 and 3) note: STL and fusion files on repo

- The next big thing we did was make our own algorithm to find, detect and track a ball based on shape and color. Also we needed the camera to detect and trcak and aruco tage as well. We then combined these two applications and 3d designed parts so that car would go track a rolling ball, catch it, then find an aruco tag and drives the ball to its goal. We designed our test to demonstate this working by having the car track and follow a ball, catch it, and when the ball is caught and out of sight the car will then begin to look for an aruco tag (aka our goal), once the aruco tag is found the code will dtermine the distance away and move the ball into the aruco tag/goal.
  
### Demonstration
Videos- https://youtu.be/bf6a4pulwDg

Pictures-
<img width="1536" height="2048" alt="IMG_8677" src="https://github.com/user-attachments/assets/df0a3699-c924-47f9-b737-322688116265" />
Figure 1: 3d pusher for Racer car
<img width="2880" height="1800" alt="Screenshot 2026-04-30 at 11 42 23 AM" src="https://github.com/user-attachments/assets/81bced10-1a47-4751-8fe7-6dfa42856572" />
Figure 2: Aruco tag car houseing view 1
<img width="2880" height="1800" alt="Screenshot 2026-04-30 at 11 42 43 AM" src="https://github.com/user-attachments/assets/3c2038a7-0f4f-480c-b48a-5ee8fb5e337b" />
Figure 3: Aruco tag car houseing view 2
<img width="4284" height="5712" alt="IMG_2518" src="https://github.com/user-attachments/assets/90cc6965-3e10-44b5-839e-0cbb480f77a8" />
Figure 4: Ball/shape detection


   
### Installation Instructions
- Python 3.10 or newer
- Install pip {numpy, opencv-python, opencv-contrib-python, flask, requests}
- Need (venvs) virtual enviorment from class
- Some type of text editor (ex: VS code)
- Need UB_Racer package
  
    -server.py (part of class)
  
    -controller_round_aruco.py (our edited code)

### How to Run the Code
- Now that your audience has installed the necessary software, how do they run it?

### References
-Claud

-Chat GPT

-IE 482 class notes

### Future Work
- In the future if we had more time we would be able to fill the room with aruco tags and assign them in the code so that the car knows its location around the room and can move more freely. On top of this we should edit the car so that it could make tighter turns within the confines of a room. Finals if time allowed we would have liked to add a robotic arm to actually grab the object rather than just push it.

- Also in the future we would choose a different ball to move. With the original one we had it was rubber and would roll up under the car due to friction between the ball and 3d printed catcher. We did run another test using a non-rubber green round object as seen in video and it worked much better.

Future note: Within the controller_round_aruco.py there is built in line following we have just ignored this and only added/edited our own algorithm for round object and color detection.
---

## Our Repo
- Proposal.md {this is our initail proposal}
- Progress_Report.md {this is our progess report}
- Progess Report {ignore this}
- controler_round.py {code for ball and color dection as well as car controls}
- controller_round_aruco.py {code for color/shape detection and goes to aruco goal}
- README.md {this is final report}
- aruco_case_car_v1.stl {printable file for case}
- aruco_case_car_vs.f3d {case openable in fusion}
- plate.stl {printable file for plate of car}
- pusher_2.stl {ball pusher attachment for car printable}
  


---


