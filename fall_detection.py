def detect_fall(landmarks, height):
    """
    Detects if a person has fallen based on pose landmarks.
    - Checks if the head and hips are at a low height relative to shoulders.
    """
    if landmarks is None:
        return False

    # Get key landmark positions
    nose_y = landmarks[mp_pose.PoseLandmark.NOSE].y * height
    left_shoulder_y = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y * height
    right_shoulder_y = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y * height
    left_hip_y = landmarks[mp_pose.PoseLandmark.LEFT_HIP].y * height
    right_hip_y = landmarks[mp_pose.PoseLandmark.RIGHT_HIP].y * height

    
    # Average shoulder and hip height
    avg_shoulder_y = (left_shoulder_y + right_shoulder_y) / 2
    avg_hip_y = (left_hip_y + right_hip_y) / 2

    """
    # Fall detection logic: If the head and hips are near the ground (below shoulders)
    if nose_y > avg_shoulder_y and avg_hip_y > avg_shoulder_y:
        return True  # Fall detected
    """
    if nose_y > left_shoulder_y and nose_y > left_hip_y:
        return True
    if nose_y > right_shoulder_y and nose_y > right_hip_y:
        return True
    if avg_hip_y <= avg_shoulder_y+20:
        return True

    return False  # No fall detected
