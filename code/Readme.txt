Mount the board so:

X axis points forward (nose of the car)

Y axis points to the left side of the car

Z axis points up

On power-up:

Leave the car on level ground, not moving during the calibration countdown.

Watch Serial output: after calibration, it prints the biases it found.

Once you like the numbers:

Copy the printed ax_bias, ay_bias, az_bias, gx_bias, gy_bias, gz_bias

Paste them at the top of the file in place of the zeros so you don’t have to recalibrate every time.

While driving:

roll_f = body roll in corners (how much the car leans)

pitch_f = nose dive/squat under braking/accel

yaw_f ≈ heading change around the oval (will drift slowly over a long run)
