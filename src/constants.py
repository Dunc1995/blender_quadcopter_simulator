ANIMATION_FRAMES = 250
ANIMATION_FRAMES_PER_SECOND = 25
SECONDS_OF_SIMULATION = round(ANIMATION_FRAMES/ANIMATION_FRAMES_PER_SECOND) #Derived from Animation configuration for simplicity.
TIME_STEPS_PER_SECOND = 100 #Higher numbers will result in better simulation accuracy.
FRAME_RESOLUTION = round(TIME_STEPS_PER_SECOND/ANIMATION_FRAMES_PER_SECOND) #Used for mapping the simulation steps to animation frames.