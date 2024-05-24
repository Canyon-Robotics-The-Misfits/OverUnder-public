Team 15442c's program for the 2023-2024 season, VRC: Over Under. Uses PROS 4

Developed by [@iseau395](https://github.com/iseau395) and [@Snowplou](https://github.com/Snowplou)

A lot of the code is a little messy or inconsistently formatted from either us rushing to make the code work when we were low on time, or from old code still staying in the repository
despite not being used in forever (ie. We haven't touched motion profiling or pure pursuit in months and they probably don't work now because of changes elsewhere in the program)

## Features

### Odometry:
* 2 tracking wheel odom
* Uses arcs for theoreticaly higher accuracy (our tests for arc vs no arc were inconclusive, but it should be more accurate)
* We also added GPS-based odometry but never really used it

### Movement:
* We used drive/turn PIDs and a boomerang controller mostly
* The code also has Motion Profiles and Pure Pursuit, but we didn't really use them that much because it was a lot more work to tune when we can get similar results with easier methods
* We also use time-based movements a lot whenever we didn't need accuracy

### Pneumatic class:
* We made our own class to manage pneumatic solonoids, leting us make pneumatic groups

### Screen GUI
* A GUI for selecting autos before the match

## Future plans

While this is [@Snowplou](https://github.com/Snowplou)'s last year, I ([@iseau395](https://github.com/iseau395)) still have one more year left, so here's what I want to do (and have partially already started to do) for next year

### Parameters
We had issues this last year with inputting parameters into our drivetrain functions. Because of all the different variables we could tune for some functions, we found ourselves often having to write out the default values for like 8 parameters just so we could change the one at the end of the list, ending up with lines like this, which are really hard to understand what is going on:
```cpp
drivetrain->driveTo(pos(144 - 22, 144 - 22), 210, true, 110, 0.6, 0.75, false, 4, 2000, 15);
```
Because of this, midseason we added a system to set the most used parameters with setter functions before the actual movement. This turned the above line into something like this:
```cpp
drivetrain->setTimeout(2000);
drivetrain->driveTo(pos(144 - 22, 144 - 22), 210, true, 0.6, 4, 15);
```
This is a little more readable, but doesn't completely solve the problem. It also disconnects some of the parameters from the actual movement they're effecting, making it harder to find what parameter setters are effecting what. Next year I'm going to try define optional parameters with structs, which should hopefully solve the problem. Here's what that will look like:
```cpp
drivetrain->driveTo(pos(144 - 22, 144 - 22), 210, { reverse: true, angle_correction: 15 });
```

### Movement functions
I want to add more gradual acceleration/deceleration to prevent wheel slipping, which should make autos more consistent. I also want to experiement with other drive-to-point functions to see if they could work better then boomerang (I've found two other ones so far), and also maybe revist pure pursuit to see if I can get it easier to work with. 

I also want to add more configurable end conditions for movements, ie end when near a point or near an angle. This is technically possible in the current code but it is a little finicky/hacky, it would be good if it was more of a built in feature. I also want to see if I can get the end conditions to chain better into the movements after them to make routes smoother

### Odometry
If I have time I may also try to make a Kalman filter to combine data from the GPS strips and the tracking wheel odometry to get more accurate and less drifty odometry.

### On the fly rerouting
It would be cool to be able to modify the auto route using the brain screen immediately before a match based on what our alliance partner or opponents are doing. I don't know how complicated I want to make this (or if I will do it at all), but if I have time and come up with a good system for it I may do it

### Dependency Injection and Composition
I like how we used composition and dependancy injection to modularize and abstract the code. I'm going to try to use this more for new things I add, but also I'm going to simplify how it's done in the current code (ie the IErrorController class probably doesn't need to exist)

### Unit tests??
This might be too complicated for what it's worth, but I'm going to try to make the code able to be compiled and tested outside the brain. This will hopefully help with catching bugs before they waste hours of my time. If this ends up not working great I may stick with just running the tests on the brain instead.

---

If you have any questions you can find me on the Vex Forum `@iseau395`