# Quadcopter Simulation in Blender 2.8
## Overview
After a many years break away from Blender, it came as a surprise to me to find that the game engine has been binned in Blender 2.8 - I've always had the desire to try and implement some machine learning and was hoping to utilise the engine's physics and controllers, but alas it is no more. Oh well.

Although there are already some fantastic technologies to combine 3D modelling/simulation with code (e.g. Unity and Visual Studio Community) it pains me to admit that I don't like these tools partially because of their mainstream appeal - you wouldn't install GB's worth of tooling to use 0.1% of its use cases.

Hence the existence of this project. Whilst at university I remember coming across some MATLAB code that was designed to simulate the flight dynamics of a quadcopter - if you're comfortable with 3D dynamics, I would highly recommend taking a look at the [blog](http://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/) and [repo](https://github.com/gibiansky/experiments/tree/master/quadcopter). This project essentially takes the same setup, but implements it in Python and Blender.

I'm not entirely convinced the rotational transforms for angular velocity have been implemented correctly, but I am hoping my brain will eventually understand what's going wrong; Euler angles still confuse the hell out of me despite having studied Dynamics throughout university.

## Preview
This is still very much a work in progress:  
![](images/example_1.gif)