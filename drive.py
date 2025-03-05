def move_car(direction):
    if direction == "left":
        GPIO.output(left_motor_forward, GPIO.LOW)
        GPIO.output(left_motor_backward, GPIO.HIGH)
        GPIO.output(right_motor_forward, GPIO.HIGH)
        GPIO.output(right_motor_backward, GPIO.LOW)
    elif direction == "right":
        GPIO.output(left_motor_forward, GPIO.HIGH)
        GPIO.output(left_motor_backward, GPIO.LOW)
        GPIO.output(right_motor_forward, GPIO.LOW)
        GPIO.output(right_motor_backward, GPIO.HIGH)
    elif direction == "forward":
        GPIO.output(left_motor_forward, GPIO.HIGH)
        GPIO.output(left_motor_backward, GPIO.LOW)
        GPIO.output(right_motor_forward, GPIO.HIGH)
        GPIO.output(right_motor_backward, GPIO.LOW)
    else:
        GPIO.output(left_motor_forward, GPIO.LOW)
        GPIO.output(left_motor_backward, GPIO.LOW)
        GPIO.output(right_motor_forward, GPIO.LOW)
        GPIO.output(right_motor_backward, GPIO.LOW)


# Function to detect the direction of the person
def get_person_direction(landmarks, frame_width, frame_height):
    if landmarks:
        nose = landmarks[mp_pose.PoseLandmark.NOSE]
        nose_x = int(nose.x * frame_width)
        nose_y = int(nose.y * frame_height)

        # Direction towards the person (left, right, forward)
        if nose_x < frame_width // 3:
            return "left"
        elif nose_x > frame_width * 2 // 3:
            return "right"
        else:
            return "forward"
    return "no_person"
