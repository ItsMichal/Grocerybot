# Grocerybot (Formerly RoboShopper 3000)

Grocerybot is a proof-of-concept robot that utilizes inverse kinematics, object recognition, path planning and finding, and web server communication to receive a shopping list from a web app and autonmously navigate a standard grocery store, collecting specific items from a list and returning to checkout once done.

This is our final project submission for the Spring 2021 session of CSCI 3302 - Intro to Robotics at CU Boulder. This is a proof of concept, so please treat it as such, and thank you!

## Project Video (YouTube)

[![Grocerybot Demo](https://img.youtube.com/vi/YaHhRhEKA_I/0.jpg)](https://www.youtube.com/watch?v=YaHhRhEKA_I "Grocerybot Demo")

Click the above image to be taken to the project video (2m35s)

## Contributors

- **Michal Bodzianowski [@itsMichal](https://michal.us)**- Programming Lead, IK, Vision, Control Flow, Robot Design, Web Server
- **Dustin Ramsay**- Path planning, Math, Debug, Misc. Programming, Documentation
- **Max Nyffenegger**- Environment, Math, Debug, Misc. Programming, Documentation

## How to run

1. Install [Webots r2021a](https://github.com/cyberbotics/webots/releases/tag/R2021a). This was developed on `r2021a` and `r2021b` contains a fatal bug with camera recognition.
2. Clone or download this repo locally.
3. Run `pip install -r requirements.txt` at the root of this repo.
4. Run webots and open `world/grocery.wbt`
5. (Optional) Run the webserver, and then open the website locally
6. Run the simulation by pressing play in webots
7. (If running webserver) Make your selection in the website and click shop within 15s of the simulation starting.

## Instructions for Webserver

1. Run `server.py` on the same machine as Webots
    -   `py server.py`
2. Go to `localhost` in your browser for the shopping list.
3. Start the webots simulation
4. Click your shopping list and relax

If server isn't running, Webots defaults to oranges and beers. Additionally, you have about 30 seconds before it defaults as well.

## Known problems

- Slow in real-time. Could be sped up
- IK fails on certain objects
- Path planning is prone to failure, and overly complex
- Code and state machine could be cleaned up